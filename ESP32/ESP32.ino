#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "math.h"

#define SCT013  34
#define ZMPT10  35
#define FILTER_LEN  15

uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Raw = 0;
int AN_Pot1_Filtered = 0;
float current_samples[384];
float voltage_samples[384];
int i = 0, j = 0, aux=0;
unsigned long begin_time=0, delta=0;
long max_current_value;
long max_voltage_value;
float peak_current;
float RMS_current;
float peak_voltage;
float RMS_voltage;

void setup() {
  pinMode(SCT013, INPUT);
  pinMode(ZMPT10, INPUT);
  Serial.begin(115200);
}

void loop() {
      max_current_value = 0;
      max_voltage_value = 0;
      
      adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
      adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
      adc1_config_width(ADC_WIDTH_10Bit);

      // Current and Voltage sampling
      for (i = 0; i < 384; i++) {
        begin_time = micros();//ADC_ATTEN_6db
        current_samples[i] = adc1_get_raw(ADC1_CHANNEL_6);
        //current_samples[i] = readADC_Avg(current_samples[i]);
        
        if (max_current_value < current_samples[i]) max_current_value = current_samples[i];
        
        voltage_samples[i] = adc1_get_raw(ADC1_CHANNEL_7);
        //voltage_samples[i] = readADC_Avg(voltage_samples[i]);
        
        if (max_voltage_value < voltage_samples[i]) max_voltage_value = voltage_samples[i];
        
        while((micros() - begin_time) < 130);
      }

       Serial.println("Max Current: ");
       Serial.println(max_current_value);
       //Serial.println("Max Voltage: ");
       //Serial.println(max_voltage_value);
      
      
       // peak_current = max_current_value - 456;
       // peak_current = peak_current * 0.32258065; // (3.3 / 1023) * 100
       RMS_current = get_I_RMS(current_samples);
       //Serial.println("RMS Current: ");
       //Serial.println(RMS_current);

       // peak_voltage = max_voltage_value - 453;
       // peak_voltage = peak_voltage * ;
       RMS_voltage = get_V_RMS(voltage_samples);
       Serial.println("RMS Voltage: ");
       Serial.println(RMS_voltage);
       
       delay(2000);
}

uint32_t readADC_Avg(int ADC_Raw)
{
  int i = 0;
  uint32_t Sum = 0;
  
  AN_Pot1_Buffer[AN_Pot1_i++] = ADC_Raw;
  if(AN_Pot1_i == FILTER_LEN)
  {
    AN_Pot1_i = 0;
  }
  for(i=0; i<FILTER_LEN; i++)
  {
    Sum += AN_Pot1_Buffer[i];
  }
  return (Sum/FILTER_LEN);
}

uint32_t get_I_RMS(float samples[])
{
  int i = 0;
  int one_wave_cycle = 384;
  float sum = 0;
  
  for(i = 0; i < one_wave_cycle; i++){
    sum += pow(((samples[i]-463)*0.322581), 2);
  }
  return (sqrt(sum/one_wave_cycle));
}

uint32_t get_V_RMS(float samples[])
{
  int i = 0;
  int one_wave_cycle = 384;
  float sum = 0;
  
  for(i = 0; i < one_wave_cycle; i++){
    sum += pow(((samples[i]-463)*2.4), 2);
  }
  return (sqrt(sum/one_wave_cycle));
}
