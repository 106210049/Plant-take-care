#include <Arduino.h>
#include <WiFi.h>
#include "ESPAsyncWebServer.h"
#include <Arduino_JSON.h>
#include <SPI.h>
#include <mcp2515.h>
#include "DHTesp.h"

// --- Cấu hình phần cứng ---
#define DHT_PIN 4
#define SPI_CS_PIN 5

// WiFi
const char *ssid = "Long";
const char *password = "00000000";

// HTML giao diện (giữ nguyên như bạn có thể đưa ra ngoài 2 class)
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <title>Car Sensor Dashboard - Team 2</title>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <link rel="icon" href="data:,">
  <style>
    body {
      font-family: Arial, sans-serif;
      margin: 0;
      text-align: center;
      background-color: #f7f7f7;
    }
    h1 {
      background-color: #4CAF50;
      color: white;
      padding: 20px;
      margin: 0;
    }
    .container {
      display: grid;
      grid-template-columns: repeat(2, 1fr);
      grid-gap: 20px;
      margin: 20px auto;
      padding: 0 20px;
      max-width: 600px;
    }
    .card {
      background-color: white;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.2);
      border-radius: 8px;
      padding: 20px;
      text-align: center;
    }
    .card .title {
      font-size: 1.2rem;
      font-weight: bold;
      margin-bottom: 10px;
    }
    .card .value {
      font-size: 2rem;
      font-weight: bold;
    }
    .temperature { border-top: 5px solid #FF5722; }
    .humidity    { border-top: 5px solid #2196F3; }
    .light       { border-top: 5px solid #FFC107; }
    .rain        { border-top: 5px solid #9C27B0; }
  </style>
</head>
<body>
  <h1>Car Sensor Dashboard - Team 2</h1>
  <div class="container">
    <div class="card temperature">
      <div class="title">Temperature</div>
      <div class="value"><span id="temperature">0</span> &deg;C</div>
    </div>
    <div class="card humidity">
      <div class="title">Humidity</div>
      <div class="value"><span id="humidity">0</span> %</div>
    </div>
    <div class="card light">
      <div class="title">Light</div>
      <div class="value"><span id="light">0</span></div>
    </div>
    <div class="card rain">
      <div class="title">Rain</div>
      <div class="value"><span id="rain">0</span></div>
    </div>
  </div>
  <script>
    if (!!window.EventSource) {
      var source = new EventSource('/events');
      source.addEventListener('sensor_data', function(e) {
        var data = JSON.parse(e.data);
        document.getElementById("temperature").innerHTML = data.temperature;
        document.getElementById("humidity").innerHTML = data.humidity;
        document.getElementById("light").innerHTML = data.light;
        document.getElementById("rain").innerHTML = data.rain;
      }, false);
    }
  </script>
</body>
</html>
)rawliteral";

AsyncWebServer server(80);
AsyncEventSource events("/events");

class DataHandler; // forward declaration

class SensorManager
{
  friend class DataHandler;

private:
  DHTesp dhtSensor;
  MCP2515 mcp2515;
  struct can_frame canMsg;

  // Data
  float temperature = 0.0;
  float humidity = 0.0;
  int rainStatus = 0;
  int lightStatus = 0;

  SemaphoreHandle_t xMutex;

public:
  SensorManager() : mcp2515(SPI_CS_PIN) {}

  void begin(SemaphoreHandle_t mutex)
  {
    xMutex = mutex;

    dhtSensor.setup(DHT_PIN, DHTesp::DHT22);

    SPI.begin();
    mcp2515.reset();
    mcp2515.setBitrate(CAN_125KBPS);
    mcp2515.setNormalMode();
  }

  void taskReadDHT(void *parameter)
  {
    while (1)
    {
      TempAndHumidity data = dhtSensor.getTempAndHumidity();
      if (!isnan(data.temperature) && !isnan(data.humidity))
      {
        if (xSemaphoreTake(xMutex, portMAX_DELAY))
        {
          temperature = data.temperature;
          humidity = data.humidity;
          xSemaphoreGive(xMutex);
        }
      }
      else
      {
        // Serial.println("Failed to read from DHT22");
        temperature = 0;
        humidity = 0;
      }
      vTaskDelay(2000 / portTICK_PERIOD_MS);
    }
  }

  void taskCANReceive(void *parameter)
  {
    while (1)
    {
      if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK)
      {
        if (canMsg.can_id == 0x100)
        {
          if (xSemaphoreTake(xMutex, portMAX_DELAY))
          {
            rainStatus = canMsg.data[0];
            lightStatus = canMsg.data[3];
            Serial.println("Rain: " + String(rainStatus));
            Serial.println("Light: " + String(lightStatus));
            xSemaphoreGive(xMutex);
          }
        }
        else
        {
          Serial.println("CAN NOT RECV");
        }
      }
      vTaskDelay(500 / portTICK_PERIOD_MS);
    }
  }
};

class DataHandler
{
  friend class SensorManager;

private:
  QueueHandle_t jsonQueue;
  SemaphoreHandle_t xMutex;
  SemaphoreHandle_t xDataReadySemaphore;
  SensorManager *sensorMgr;

public:
  DataHandler() {}

  void begin(SensorManager *sensor, SemaphoreHandle_t mutex, QueueHandle_t queue, SemaphoreHandle_t dataReady)
  {
    sensorMgr = sensor;
    xMutex = mutex;
    jsonQueue = queue;
    xDataReadySemaphore = dataReady;
  }

  void taskDataCollector(void *parameter)
  {
    while (1)
    {
      float tempCopy, humCopy;
      int rainCopy, lightCopy;

      if (xSemaphoreTake(xMutex, portMAX_DELAY))
      {
        tempCopy = sensorMgr->temperature;
        humCopy = sensorMgr->humidity;
        rainCopy = sensorMgr->rainStatus;
        lightCopy = sensorMgr->lightStatus;
        xSemaphoreGive(xMutex);
      }

      JSONVar data;
      data["temperature"] = String(tempCopy, 1);
      data["humidity"] = String(humCopy, 1);
      data["light"] = lightCopy ? "Dark" : "Bright";
      data["rain"] = rainCopy ? "Raining" : "Dry";

      String jsonStr = JSON.stringify(data);

      if (uxQueueSpacesAvailable(jsonQueue) > 0)
      {
        xQueueSend(jsonQueue, &jsonStr, portMAX_DELAY);
        xSemaphoreGive(xDataReadySemaphore); // notify SendData task
      }
      else
      {
        Serial.println("[Queue] Overflow prevented: message dropped");
      }

      vTaskDelay(10000 / portTICK_PERIOD_MS);
    }
  }

  void taskSendData(void *parameter)
  {
    String msg;
    while (1)
    {
      if (xSemaphoreTake(xDataReadySemaphore, portMAX_DELAY) == pdTRUE)
      {
        if (xQueueReceive(jsonQueue, &msg, 0) == pdTRUE)
        {
          if (events.count() > 0)
          {
            events.send(msg.c_str(), "sensor_data", millis());
          }
        }
      }
    }
  }
};

SensorManager sensorManager;
DataHandler dataHandler;

SemaphoreHandle_t xMutex;
QueueHandle_t jsonQueue;
SemaphoreHandle_t xDataReadySemaphore;

// void connectWifi()
// {
//   Serial.println("Setting up Access Point...");
//   WiFi.softAP(ssid, password);
//   IPAddress IP = WiFi.softAPIP();
//   Serial.print("AP IP address: ");
//   Serial.println(IP);
// }
void connectWifi()
{
  Serial.println("Connecting to WiFi...");
  WiFi.begin(ssid, password);

  int maxRetries = 20;
  while (WiFi.status() != WL_CONNECTED && maxRetries > 0)
  {
    delay(500);
    Serial.print(".");
    maxRetries--;
  }

  if (WiFi.status() == WL_CONNECTED)
  {
    Serial.println("\nWiFi connected.");
    Serial.print("IP address: ");
    Serial.println(WiFi.localIP());
  }
  else
  {
    Serial.println("\nFailed to connect to WiFi.");
  }
}

void setup()
{
  Serial.begin(115200);
  connectWifi();

  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });
  events.onConnect([](AsyncEventSourceClient *client)
                   { Serial.println("Client connected to SSE"); });
  server.addHandler(&events);
  server.begin();

  xMutex = xSemaphoreCreateMutex();
  jsonQueue = xQueueCreate(10, sizeof(String));
  xDataReadySemaphore = xSemaphoreCreateBinary();

  sensorManager.begin(xMutex);
  dataHandler.begin(&sensorManager, xMutex, jsonQueue, xDataReadySemaphore);

  // Tạo task theo từng method của class, dùng lambda để gọi member function (cần truyền this pointer)
  xTaskCreatePinnedToCore(
      [](void *param)
      {
        ((SensorManager *)param)->taskReadDHT(param);
      },
      "Read_DHT", 2048, &sensorManager, 1, NULL, 1);

  xTaskCreatePinnedToCore(
      [](void *param)
      {
        ((SensorManager *)param)->taskCANReceive(param);
      },
      "CAN_RECV", 2048, &sensorManager, 1, NULL, 0);

  xTaskCreatePinnedToCore(
      [](void *param)
      {
        ((DataHandler *)param)->taskDataCollector(param);
      },
      "DataCollector", 4096, &dataHandler, 1, NULL, 1);

  xTaskCreatePinnedToCore(
      [](void *param)
      {
        ((DataHandler *)param)->taskSendData(param);
      },
      "SendData", 4096, &dataHandler, 1, NULL, 0);
}

void loop()
{
  vTaskDelay(portMAX_DELAY);
}
