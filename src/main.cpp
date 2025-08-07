#include <Arduino.h>
#include <ArduinoFFT.h>
#include <driver/i2s.h>
#include <driver/adc.h>

#define SAMPLES       2048
#define SAMPLE_RATE   50000          // Fixed sampling rate enforced by pacing loop
#define ADC_CHANNEL   ADC1_CHANNEL_0 // GPIO36
#define MODE_PIN      4
#define DAC_PIN       25
#define BinRange      5              // ± bin range for smoothing

ArduinoFFT<float> FFT;

float vReal[SAMPLES];
float vImag[SAMPLES];

bool outputState;
// I2S configuration
void setupADC_DMA()
{
  i2s_config_t i2s_config = {
      .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX | I2S_MODE_ADC_BUILT_IN),
      .sample_rate = SAMPLE_RATE,
      .bits_per_sample = I2S_BITS_PER_SAMPLE_16BIT,
      .channel_format = I2S_CHANNEL_FMT_ONLY_RIGHT,
      .communication_format = I2S_COMM_FORMAT_I2S_MSB,
      .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
      .dma_buf_count = 4,
      .dma_buf_len = 512,
      .use_apll = false,
      .tx_desc_auto_clear = false,
      .fixed_mclk = 0};

  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_adc_mode(ADC_UNIT_1, ADC_CHANNEL);
  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(ADC_CHANNEL, ADC_ATTEN_DB_11);
  i2s_adc_enable(I2S_NUM_0);
}

void setup()
{
  Serial.begin(115200);
  pinMode(MODE_PIN, INPUT_PULLDOWN);
  pinMode(DAC_PIN, OUTPUT);  
  setupADC_DMA();
  outputState = false;
}

void loop()
{
  static const unsigned long desiredCycleUs = (unsigned long)(1000000.0f * SAMPLES / SAMPLE_RATE);
  unsigned long cycleStart = micros();

  int16_t rawData[SAMPLES];
  size_t bytesRead = 0;

  // Read raw ADC data via I2S
  i2s_read(I2S_NUM_0, (void *)rawData, sizeof(rawData), &bytesRead, portMAX_DELAY);

  // Convert raw 12-bit ADC values (stored in 16-bit ints) to float
  for (int i = 0; i < SAMPLES; i++)
  {
    vReal[i] = (float)(rawData[i] & 0x0FFF); // 12-bit mask
  }

  // Remove DC offset
  float mean = 0;
  for (int i = 0; i < SAMPLES; i++)
    mean += vReal[i];
  mean /= SAMPLES;
  for (int i = 0; i < SAMPLES; i++)
    vReal[i] -= mean;

  // Zero imaginary parts before FFT
  memset(vImag, 0, sizeof(vImag));

  // FFT processing
  FFT.windowing(vReal, SAMPLES, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(vReal, vImag, SAMPLES, FFT_FORWARD);
  FFT.complexToMagnitude(vReal, vImag, SAMPLES);

  // Clamp target bin within valid FFT output range
  int maxBinIndex = SAMPLES / 2 - 1;
  bool isHigh = digitalRead(MODE_PIN);
  int targetBin = isHigh ? 41 : 613; // Precomputed bin index for 10kHz / 1kHz

  // Find maximum value in ±BinRange bins around targetBin (within valid range)
  float smoothedValue = 0;
  for (int i = targetBin - BinRange; i <= targetBin + BinRange; i++)
  {
    if (i >= 0 && i <= maxBinIndex)
    {
      if (vReal[i] > smoothedValue)
      {
        smoothedValue = vReal[i];
      }
    }
  }

    // Hysteresis thresholds
  const float thresholdHigh = 75000.0f; // Trigger ON above this value
  const float thresholdLow  = 70000.0f; // Trigger OFF below this value
  // bool outputState = false;      // Remember last state

  // Update state with hysteresis
  if (!outputState && smoothedValue >= thresholdHigh)
  {
    outputState = true;
  }
  else if (outputState && smoothedValue <= thresholdLow)
  {
    outputState = false;
  }

  // Write digital output
  digitalWrite(DAC_PIN, outputState ? HIGH : LOW);

  Serial.printf("Mode: %s | Smoothed (Max in Range): %.2f | Output: %d\n",
                isHigh ? "1kHz" : "10kHz", smoothedValue, outputState);
  // Enforce fixed loop timing
  unsigned long cycleTime = micros() - cycleStart;
  if (cycleTime < desiredCycleUs)
    delayMicroseconds(desiredCycleUs - cycleTime);
}
