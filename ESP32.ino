#include "driver/adc.h"
#include "esp_adc_cal.h"

#define SCT013  34
#define ZMPT10  35
#define FILTER_LEN  50

uint32_t AN_Pot1_Buffer[FILTER_LEN] = {0};
int AN_Pot1_i = 0;
int AN_Pot1_Raw = 0;
int AN_Pot1_Filtered = 0;
int leitura_corrente[384];
int leitura_tensao[384];
int i = 0, j = 0, aux=0;
unsigned long tempo_inicio=0, delta=0;
long max_valor_corrente;
long max_valor_tensao;
float corrente_pico;
float corrente_eficaz;
float tensao_pico;
float tensao_eficaz;

void setup() {
  pinMode(SCT013, INPUT);
  pinMode(ZMPT10, INPUT);
  Serial.begin(115200);
}

void loop() {
      max_valor_corrente = 0;
      max_valor_tensao = 0;
      adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
      adc1_config_channel_atten(ADC1_CHANNEL_7, ADC_ATTEN_DB_11);
      adc1_config_width(ADC_WIDTH_10Bit);

      // Amostragem da corrente
      for (i = 0; i < 384; i++) {
        tempo_inicio = micros();//ADC_ATTEN_6db
        leitura_corrente[i] = adc1_get_raw(ADC1_CHANNEL_6);
        leitura_corrente[i] = readADC_Avg(leitura_corrente[i]);
        if (max_valor_corrente < leitura_corrente[i]) max_valor_corrente = leitura_corrente[i];
        while((micros() - tempo_inicio)<130);
      }

      // Amostragem da tensao
      for (i = 0; i < 384; i++) {
        tempo_inicio = micros();//ADC_ATTEN_6db
        leitura_tensao[i] = adc1_get_raw(ADC1_CHANNEL_7);
        leitura_tensao[i] = readADC_Avg(leitura_tensao[i]);
        if (max_valor_tensao < leitura_tensao[i]) max_valor_tensao = leitura_tensao[i];
        while((micros() - tempo_inicio)<130);
      }
      
       corrente_pico = max_valor_corrente - 456;
       corrente_pico = corrente_pico * 0.32258065; // (3.3 / 1023) * 100
       corrente_eficaz = corrente_pico / 1.4;
       Serial.println("Corrente RMS: ");
       Serial.println(corrente_eficaz);

       tensao_pico = max_valor_tensao - 453;
       tensao_pico = tensao_pico * ;
       tensao_eficaz = tensao_pico / 1.4;
       Serial.println("Tensao RMS: ");
       Serial.println(tensao_eficaz);
       
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
