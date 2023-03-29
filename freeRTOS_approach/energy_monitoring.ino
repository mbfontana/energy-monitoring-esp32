#include <WiFi.h>
#include <HTTPClient.h>

// Credentials
const char *wifiSSID = "Fontana";
const char *wifiPassword = "@678fontana786";
const char *serverUrl = "http://192.168.0.156";
const int serverPort = 3001;

// Core definition
static const BaseType_t pro_cpu = 0;
static const BaseType_t app_cpu = 1;

// Function prototypes
void sendPostRequest(String endpoint, String body);
void handleCommand(String command);
void userInterfaceTask(void *pvParameters);

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
    else if (command == "post_request")
    {
        String json = "";
        json += "{";
        json += "\"fft01\": 10.8,";
        json += "\"fft03\": 10.8,";
        json += "\"fft05\": 10.8,";
        json += "\"fft07\": 10.8,";
        json += "\"fft09\": 10.8,";
        json += "\"fft11\": 10.8,";
        json += "\"fft13\": 10.8,";
        json += "\"fft15\": 10.8,";
        json += "\"fft17\": 10.8,";
        json += "\"fft19\": 10.8,";
        json += "\"fft20\": 10.8,";
        json += "\"fft21\": 10.8,";
        json += "\"fft23\": 10.8,";
        json += "\"fft25\": 10.8,";
        json += "\"fft27\": 10.8,";
        json += "\"fft29\": 10.8,";
        json += "\"fft31\": 10.8,";
        json += "\"applianceId\": 1";
        json += "}";

        Serial.println("JSON: ");
        Serial.println(json);

        sendPostRequest("/sample", json);
    }
    else
    {
        Serial.println("Invalid command.");
    }
}

void sendPostRequest(String endpoint, String body)
{
    HTTPClient http;

    String webServer = "";
    webServer += serverUrl;
    webServer += ":";
    webServer += serverPort;
    webServer += endpoint;

    Serial.println("Connecting to server...");
    if (http.begin(webServer))
    {
        Serial.println("Connected to server");

        // Set the content type header
        http.addHeader("Content-Type", "application/json");

        // Send the POST request with the JSON body
        Serial.print("Making a POST request to: ");
        Serial.print(serverUrl);
        Serial.print(":");
        Serial.print(serverPort);
        Serial.print(endpoint);

        int httpResponseCode = http.POST(body);

        // Check if the request was successful
        if (httpResponseCode > 0)
        {
            // Print the response payload
            Serial.println(http.getString());
        }
        else
        {
            Serial.print("Error sending HTTP request: ");
            Serial.println(http.getString());
        }

        // Close the connection
        http.end();
        Serial.println("Connection closed");
    }
    else
    {
        Serial.println("Connection failed");
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
    WiFi.begin(wifiSSID, wifiPassword);
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

    // Task to interact with user via serial monitor
    xTaskCreatePinnedToCore(userInterfaceTask, "UserInterface", 8000, NULL, 1, NULL, app_cpu);
}

void loop()
{
    // Nothing here
}