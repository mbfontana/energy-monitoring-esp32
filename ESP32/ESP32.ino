// https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json

#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "arduinoFFT.h"

#define SCT013  34
#define ZMPT101  35

unsigned int I_offset = 0;
unsigned int V_offset = 0;
float Vsensitivity = 0.00188679245; // Calibração
float Isensitivity = 0.01; // Calibração
float ADCScale = 1023.0;
float Vref = 3.25;

float V_conversion_factor = ADCScale * Vref / Vsensitivity;
float I_conversion_factor = ADCScale * Vref / Isensitivity;

struct RMS {
  float I;
  float V;
  float P;
};

void setup() {
  adc1_config_width(ADC_WIDTH_10Bit);
  pinMode(SCT013, INPUT);
  pinMode(ZMPT101, INPUT);
  Serial.begin(115200);
}

void loop() {

  unsigned int V_accum = 0;
  unsigned int I_accum = 0;
  
  for(int i=0; i<100; i++)
  {
    I_accum += adc1_get_raw(ADC1_CHANNEL_6);
    V_accum += adc1_get_raw(ADC1_CHANNEL_7);
    delayMicroseconds(1000); // 128 amostras por ciclo
  }

  I_offset = I_accum/100;
  V_offset = V_accum/100;
  struct RMS valuesRMS = getRMS(V_offset, I_offset);
  float Irms = valuesRMS.I;
  float Vrms = valuesRMS.V;
  float P = valuesRMS.P; // Potência Ativa
  float S = Vrms * Irms; // Potência Aparente
  float Q = sqrt((S*S)-(P*P)); // Potência Reativa
  float FP = S / P; // Fator de potência

  Serial.print("Irms: ");
  Serial.println(Irms);
  Serial.print("Vrms: ");
  Serial.println(Vrms);
  Serial.print("S: ");
  Serial.println(S);
  Serial.print("P: ");
  Serial.println(P);
  Serial.print("Q: ");
  Serial.println(Q);
  Serial.print("FP: ");
  Serial.println(FP);
  Serial.println("");

  delay(5000);
}

struct RMS getRMS(unsigned int V_offset, unsigned int I_offset) {

  struct RMS valueRMS;
  uint32_t Vsum = 0, Isum = 0, Psum = 0, measurements_count = 0;
  int32_t Vnow, Inow, Pnow;

  while (measurements_count < 64) {
    Inow = (adc1_get_raw(ADC1_CHANNEL_6) - I_offset);
    Vnow = (adc1_get_raw(ADC1_CHANNEL_7) - V_offset);     
    Inow = Inow / ADCScale * Vref / Isensitivity;
    Vnow = Vnow / ADCScale * Vref / Vsensitivity;
    Pnow = Vnow*Inow;
    Isum += Inow*Inow;
    Vsum += Vnow*Vnow;
    Psum += Pnow*Pnow;
    measurements_count++;
    delayMicroseconds(266);
  } // 3 ciclos completos (50ms)

  float Irms = sqrt(Isum / measurements_count) - 1;
  float Vrms = sqrt(Vsum / measurements_count);
  float Prms = sqrt(Psum / measurements_count);
 
  valueRMS.I = Irms;  
  valueRMS.V = Vrms; 
  valueRMS.P = Prms; 

  return valueRMS;
}
