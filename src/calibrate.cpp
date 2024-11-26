#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"

// Constants
const int SAMPLES = 384;           // Number of samples
const int OFFSET = 1830;           // Calibration offset
const float CONVERT_FACTOR = 0.0135; // Calibration factor
const int SAMPLING_INTERVAL_US = 130; // Sampling interval in microseconds

// Variables
int readings[SAMPLES];             // ADC readings

// Function Prototypes
void setupADC();
void collectReadings();
float calculateRMS();

void setup() {
    Serial.begin(115200);
    setupADC();
}

void loop() {
    collectReadings();
    float rms = calculateRMS();
    Serial.println(rms);
    delay(2000);
}

void setupADC() {
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11); // 3.3V range
    adc1_config_width(ADC_WIDTH_BIT_12);                       // 12-bit resolution
}

void collectReadings() {
    for (int i = 0; i < SAMPLES; i++) {
        unsigned long startTime = micros();
        readings[i] = adc1_get_raw(ADC1_CHANNEL_6) - OFFSET;
        while ((micros() - startTime) < SAMPLING_INTERVAL_US);
    }
}

float calculateRMS() {
    float sum = 0;
    for (int i = 0; i < SAMPLES; i++) {
        float calibratedValue = readings[i] * CONVERT_FACTOR;
        sum += calibratedValue * calibratedValue;
    }
    return sqrt(sum / SAMPLES);
}
