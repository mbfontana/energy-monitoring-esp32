#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

int readings[384];
int i = 0, j = 0, aux = 0, samples = 384, offset = 1830;
float convertFactor = 0.0135; // Try and error the get the right calibration factor
unsigned long startTime=0, delta=0;

void setup() {
    Serial.begin(115200);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);
    adc1_config_width(ADC_WIDTH_12Bit);
}

void loop() {
    float sum = 0;
      
    for (i = 0; i < samples; i++) {
        startTime = micros();
        readings[i] = adc1_get_raw(ADC1_CHANNEL_6) - offset;
        while((micros() - startTime)<130);
    }

    for (i = 0; i < samples; i++) {
        float raw =  readings[i] * convertFactor;
        sum += raw * raw;
    }

    float rms = sqrt(sum / samples);

    Serial.println(rms);
             
    delay(2000);
}