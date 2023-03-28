#include <WiFi.h>
#include <HTTPClient.h>

#define WIFI_SSID "Fontana"
#define WIFI_PASSWORD "@678fontana786"

// Core definition
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

HTTPClient httpClient;
WiFiClient wifiClient;

// Function prototypes
void userInterfaceTask(void *pvParameters);
void handleCommand(String command);

// Tasks
void userInterfaceTask(void *pvParameters)
{
    while (1)
    {
        if (Serial.available())
        {
            String command = Serial.readStringUntil('\n');
            command.trim();
            handleCommand(command);
        }

        // Wait for 10ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// Procedures
void handleCommand(String command)
{
    if (command == "hello")
    {
        Serial.println("Hello, World!");
    }
    else
    {
        Serial.println("Invalid command.");
    }
}

void setup()
{
    // Starting Serial
    Serial.begin(115200);
    // Await a moment to start because of Serial initialization
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    Serial.println("\n--- Energy Monitoring ESP32 ---\n");

    // Connect to WiFi
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    while (WiFi.status() != WL_CONNECTED)
    {
        delay(1000);
        Serial.println("Connecting to WiFi...");
    }
    Serial.println("Connected to WiFi");
    Serial.println("ESP32 IP address: %d\n", WiFi.localIP());

    // Task to interact with user via serial monitor
    xTaskCreatePinnedToCore(userInterfaceTask, "UserInterface", 2048, NULL, 1, NULL, app_cpu);
}

void loop()
{
    // Nothing here
}