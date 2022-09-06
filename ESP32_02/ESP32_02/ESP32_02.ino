#include "driver/adc.h"
#include "esp_adc_cal.h"

#define SCT013  34
#define ZMPT10  35

unsigned int I_dcOffsetSamples = 0;
unsigned int V_dcOffsetSamples = 0;
float sensitivity = 0.005844;
float ADCScale = 1023.0;
float Vref = 3.3;

void setup() {
  pinMode(SCT013, INPUT);
  pinMode(ZMPT10, INPUT);
  Serial.begin(115200);
}

void loop() {

  adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
  adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
  adc1_config_width(ADC_WIDTH_10Bit);

  unsigned int V_accum = 0;
  unsigned int I_accum = 0;
  
  for(int i=0; i<100; i++)
  {
    I_accum += adc1_get_raw(ADC1_CHANNEL_6);
    V_accum += adc1_get_raw(ADC1_CHANNEL_7);
    delayMicroseconds(1000);
  }

  I_dcOffsetSamples = I_accum/100;
  V_dcOffsetSamples = V_accum/100;

  float Vrms = getVoltageAC(60, V_dcOffsetSamples);

  Serial.println("Current Offset:");
  Serial.println(I_dcOffsetSamples);
  Serial.println("Voltage Offset:");
  Serial.println(V_dcOffsetSamples);
  Serial.println("Vrms:");
  Serial.println(Vrms);

  delay(2000);
}


float getVoltageAC(unsigned int frequency, unsigned int offset) {
  uint32_t period = 1000000 / frequency;
  uint32_t t_start = micros();

  uint32_t Vsum = 0, measurements_count = 0;
  int32_t Vnow;

  while (micros() - t_start < period) {
    Vnow = adc1_get_raw(ADC1_CHANNEL_7) - offset;
    Vsum += Vnow*Vnow;
    measurements_count++;
  }

  float Vrms = sqrt(Vsum / measurements_count) / ADCScale * Vref / sensitivity;
  return Vrms;
}
