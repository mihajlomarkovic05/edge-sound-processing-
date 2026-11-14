A hands-on exercise in capturing audio from an I2S MEMS microphone and applying basic digital signal processing algorithms, selectable at runtime through the Serial Monitor.

Hardware:
  ESP32 development board
  INMP441 I2S MEMS microphone

INMP441	ESP32 pinout:
  VDD	3V3
  GND	GND
  SCK	GPIO 26
  WS	GPIO 25  
  SD	GPIO 33
  L/R	GND



Software:
  - Arduino IDE with ESP32 board support
  - Library: arduinoFFT (v2.x, by Enrique Condes)

How it works:
The ESP32 reads audio over I2S at 16 kHz, in blocks of 512 samples (about 32 ms).
Each 32-bit sample is shifted right by 14 bits to get a usable value.
The DC offset is removed by subtracting the block mean, and the RMS level is computed.
The selected algorithm is applied. The FFT runs only for the algorithms that need it.

Algorithms:
#	Algorithm	Domain	Output

  1	Sound level	Time	RMS and dB
  2	Dominant frequency	FFT	Peak frequency in Hz
  3	Bass / mid / treble	FFT	Energy in 3 bands
  4	Clap detection	Time	“PLJESAK!” on sudden energy spike
  5	Zero-crossing rate	Time	Sign changes per frame (%)
  6	Original + low-pass filtered	Time	Two signals plotted together
  7	FFT spectrum	FFT	16 bands, about 500 Hz each
  
Usage:

Upload the sketch and open Serial Monitor at 115200 baud.
Type a number 1-7 and press Enter to start an algorithm.
Type 0 (or m) at any time to stop and return to the menu.
For graphs, use Serial Plotter (algorithms 1, 2, 3, 5, 6, 7 output Name:value pairs and plot well). Algorithm 4 outputs text only.

Serial Plotter cannot send input, so choose the algorithm in Serial Monitor first. The ESP32 may reset when the port is reopened, which brings the menu back.
Thresholds (clap detection, alpha of the low-pass filter) depend on the microphone and environment. Use algorithm 1 in silence and with loud sounds to calibrate them.
Frequency resolution is SAMPLE_RATE / SAMPLES = 31.25 Hz per bin. Increase SAMPLES for finer resolution, at the cost of RAM.
