#include <Arduino.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <WiFi.h>
#include <HTTPClient.h>
#include <arduinoFFT.h>

#define WIFI_SSID "Fontana"
#define WIFI_PASSWORD "@678fontana786"
#define READINGS_BUFFER_SIZE 384
#define OFFSET 1830
#define CONVERSION_FACTOR 0.0135
#define READINGS_TASK_PRIORITY 3
#define READINGS_TASK_STACK_SIZE 2048
#define SAMPLING_FREQUENCY 20

// Handlers for semaphores and critical state
static TaskHandle_t handlerCurrentTask = NULL;
SemaphoreHandle_t readingsSemaphore;

// Definitions for dual cores
static const BaseType_t proCPU = 0;
static const BaseType_t appCPU = 1;

// Array to store raw current samples
double currentRaw[READINGS_BUFFER_SIZE];

// Auxiliar variables
unsigned long startTime = 0, delta = 0;
float iRMS = 0;

// FFT variables
arduinoFFT FFT;
double vReal[READINGS_BUFFER_SIZE];
double vImag[READINGS_BUFFER_SIZE];

/***************************************************/
/*                  PROCEDURES                     */
/***************************************************/

/***************************************************/
/*                    TASKS                        */
/***************************************************/
void interfaceTask(void *parameters)
{
    while (1)
    {
        // Get user input via serial monitor
        if (Serial.available() > 0)
        {
            String input = Serial.readStringUntil('\n');
            input.trim();
            // Call other functions based on user input
            if (input == "rms")
            {
                Serial.println(iRMS);
            }
            else if (input == "fft")
            {
                FFT = arduinoFFT(vReal, vImag, 32, SAMPLING_FREQUENCY);
                FFT.Windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
                FFT.Compute(FFT_FORWARD);
                FFT.ComplexToMagnitude();

                for (int i = 0; i < 32; i++)
                {
                    Serial.print("vReal: ");
                    Serial.println(vReal[i]);
                    Serial.print("vImag: ");
                    Serial.println(vImag[i]);
                }
            }
        }
        vTaskDelay(50 / portTICK_PERIOD_MS); // Wait for 50ms
    }
}

void samplingTask(void *parameter)
{
    int samples = *(int *)parameter;
    while (true)
    {
        float sum = 0;
        for (int i = 0; i < samples; i++)
        {
            startTime = micros();
            currentRaw[i] = adc1_get_raw(ADC1_CHANNEL_6) - OFFSET;
            while ((micros() - startTime) < 130)
                ;
        }

        for (int i = 0; i < samples; i++)
        {
            float raw = currentRaw[i] * CONVERSION_FACTOR;
            sum += raw * raw;
            vReal[i] = raw;
            vImag[i] = 0; // Reset the imaginary values vector for each new frequency
        }

        iRMS = sqrt(sum / samples);

        // Take the semaphore to read the current samples
        // xSemaphoreTake(readingsSemaphore, portMAX_DELAY);
        // Release the semaphore to allow other tasks to access the samples
        // xSemaphoreGive(readingsSemaphore);

        vTaskDelay(50 / portTICK_PERIOD_MS); // Wait for 50ms
    }
}

void setup()
{
    // Start serial with baud rate 115200
    Serial.begin(115200);

    // Configure ADC1
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_6, ADC_ATTEN_DB_11);

    // Create a semaphore to handle shared resources (samples array)
    readingsSemaphore = xSemaphoreCreateMutex();

    // Initialize samples integer
    int samples = READINGS_BUFFER_SIZE;

    // Await a moment to start because of Serial initialization
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println();
    Serial.println("--- Non-Intrusive Load Monitoring ESP32 ---");

    // Connecting to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("[WiFi] Connecting to WiFi...");
    }
    Serial.println("[WiFi] Connected to WiFi");

    // Print the SSID of the network
    Serial.print("[WiFi] SSID: ");
    Serial.println(WiFi.SSID());

    // Print WiFi shield's IP address
    IPAddress ip = WiFi.localIP();
    Serial.print("[WiFi] IP Address: ");
    Serial.println(ip);

    // Task that process user input (via Serial)
    xTaskCreatePinnedToCore(interfaceTask, "Serial User Interface",
                            8000, NULL, 1,
                            NULL, appCPU);

    xTaskCreatePinnedToCore(samplingTask, "ReadingsTask",
                            READINGS_TASK_STACK_SIZE, &samples,
                            READINGS_TASK_PRIORITY, NULL, proCPU);
}

void loop()
{
    // Nothing here!
}