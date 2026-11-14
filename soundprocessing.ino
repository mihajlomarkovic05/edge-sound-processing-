#include <driver/i2s.h>
#include <arduinoFFT.h>


#define I2S_PORT      I2S_NUM_0
#define I2S_BCLK      26   // SCK
#define I2S_LRCL      25   // WS
#define I2S_DIN       33   // SD

#define SAMPLE_RATE   16000
#define SAMPLES       512    // mora biti stepen dvojke

// nizovi
int32_t raw[SAMPLES];
double sig[SAMPLES];
double vReal[SAMPLES];
double vImag[SAMPLES];

ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, SAMPLES, SAMPLE_RATE);

int mode = 0;              // 0 = čeka izbor algoritma
bool menuShown = false;
double avgEnergy = 0;      // za detekciju pljeska
double lpState = 0;        // stanje niskopropusnog filtera

// ---------- I2S ----------
void setupI2S() {
  i2s_config_t config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    .channel_format = I2S_CHANNEL_FMT_ONLY_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 8,
    .dma_buf_len = 256,
    .use_apll = false
  };

  i2s_pin_config_t pins = {
    .bck_io_num = I2S_BCLK,
    .ws_io_num = I2S_LRCL,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_DIN
  };

  i2s_driver_install(I2S_PORT, &config, 0, NULL);
  i2s_set_pin(I2S_PORT, &pins);
  i2s_zero_dma_buffer(I2S_PORT);
}

// m 
void printMenu() {
  Serial.println("\n=== IZBOR ALGORITMA ===");
  Serial.println("1 - Nivo zvuka (RMS i dB)");
  Serial.println("2 - Dominantna frekvencija");
  Serial.println("3 - Bas / sredina / visoke");
  Serial.println("4 - Detekcija pljeska");
  Serial.println("5 - Zero-crossing rate");
  Serial.println("6 - Original + filtrirani signal (oba odjednom)");
  Serial.println("7 - FFT spektar (16 opsega)");
  Serial.println("------------------------");
  Serial.println("Tokom rada: ukucaj 0 (ili m) za STOP i povratak u meni");
  Serial.println("========================");
}

void readInput() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c >= '1' && c <= '7') {
      mode = c - '0';
      menuShown = false;
      Serial.print("\nIzabran algoritam: ");
      Serial.println(mode);
      delay(500);
    } else if (c == '0' || c == 'm' || c == 'M') {
      mode = 0;              // prekid
      menuShown = false;
    }
  }
}

double bandEnergy(double f1, double f2) {
  double binHz = (double)SAMPLE_RATE / SAMPLES;
  int i1 = max(1, (int)(f1 / binHz));
  int i2 = min(SAMPLES / 2 - 1, (int)(f2 / binHz));
  double sum = 0;
  for (int i = i1; i <= i2; i++) sum += vReal[i];
  return sum;
}


void setup() {
  Serial.begin(115200);
  setupI2S();
  delay(500);
}


void loop() {
  readInput();

  // meni
  if (mode == 0) {
    if (!menuShown) {
      printMenu();
      Serial.println("Ukucaj broj 1-7 i pritisni Enter...");
      menuShown = true;
    }
    delay(50);
    return;
  }

  // 1) čitanje SAMPLES uzoraka
  size_t total = 0;
  while (total < sizeof(raw)) {
    size_t n = 0;
    i2s_read(I2S_PORT, (uint8_t*)raw + total, sizeof(raw) - total, &n, portMAX_DELAY);
    total += n;
  }

  // 2) signal i prosek
  double mean = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sig[i] = (double)(raw[i] >> 14);
    mean += sig[i];
  }
  mean /= SAMPLES;

  // 3) uklanjanje DC offseta i RMS
  double sumSq = 0;
  for (int i = 0; i < SAMPLES; i++) {
    sig[i] -= mean;
    sumSq += sig[i] * sig[i];
  }
  double rms = sqrt(sumSq / SAMPLES);

  // 4) FFT za algoritme 2, 3 i 7
  if (mode == 2 || mode == 3 || mode == 7) {
    for (int i = 0; i < SAMPLES; i++) { vReal[i] = sig[i]; vImag[i] = 0; }
    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
  }

  // 5) izabrani algoritam
  switch (mode) {
    case 1:
      Serial.print("RMS:");
      Serial.print(rms);
      Serial.print(",dB:");
      Serial.println(20 * log10(rms + 1));
      break;

    case 2:
      Serial.print("Freq:");
      Serial.println(FFT.majorPeak());
      break;

    case 3:
      Serial.print("Bas:");
      Serial.print(bandEnergy(20, 250) / 1000);
      Serial.print(",Sredina:");
      Serial.print(bandEnergy(250, 2000) / 1000);
      Serial.print(",Visoke:");
      Serial.println(bandEnergy(2000, 8000) / 1000);
      break;

    case 4:
      if (rms > avgEnergy * 4 && rms > 100) Serial.println("PLJESAK!");
      avgEnergy = 0.95 * avgEnergy + 0.05 * rms;
      break;

    case 5: {
      int crossings = 0;
      for (int i = 1; i < SAMPLES; i++)
        if ((sig[i - 1] < 0) != (sig[i] < 0)) crossings++;
      Serial.print("ZCR:");
      Serial.println(crossings * 100.0 / SAMPLES);
      break;
    }

    case 6: {  // oba signala odjednom
      const double alpha = 0.1;             // manji procenat znaci jace filtriranje
      for (int i = 0; i < SAMPLES; i++) {
        lpState += alpha * (sig[i] - lpState);   // filtar ide preko svih uzoraka
        if (i % 4 == 0) {                        // ispisuje se svaki 4. uzorak
          Serial.print("Original:");
          Serial.print(sig[i]);
          Serial.print(",Filtrirano:");
          Serial.println(lpState);
        }
      }
      break;
    }

    case 7: {  // FFT spektar u 16 opsega (svaki ~500 Hz)
      for (int b = 0; b < 16; b++) {
        double sum = 0;
        for (int i = b * 16 + 1; i <= b * 16 + 16 && i < SAMPLES / 2; i++) sum += vReal[i];
        Serial.print("B");
        Serial.print(b + 1);
        Serial.print(":");
        Serial.print(sum / 1600);
        Serial.print(b < 15 ? "," : "\n");
      }
      break;
    }
  }
}