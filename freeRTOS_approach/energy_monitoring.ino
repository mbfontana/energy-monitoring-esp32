#include <WiFi.h>
#include <HTTPClient.h>

#define WIFI_SSID "Fontana"
#define WIFI_PASSWORD "@678fontana786"

HTTPClient httpClient;
WiFiClient wifiClient;

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
}

void loop()
{
    // Nothing here
}