/*
  ──────────────────────────────────────────────────────────────
  ESP32 | GPS NEO-6M | DHT22 | MCP2515            – Team 2 Demo
  ──────────────────────────────────────────────────────────────
  - GPS   :  RX GPIO16  ←  TX pin NEO-6M
             TX GPIO17  →  (thường bỏ trống)
  - DHT22 :  GPIO4
  - MCP2515 CS : GPIO5  (SCK=18, MISO=19, MOSI=23 mặc định)
  ──────────────────────────────────────────────────────────────
  Wi-Fi station mode → REST JSON   /coords
                       SSE stream  /events
*/
#if defined(ESP32)
#include <WiFiMulti.h>
WiFiMulti wifiMulti;
#define DEVICE "ESP32"
#elif defined(ESP8266)
#include <ESP8266WiFiMulti.h>
ESP8266WiFiMulti wifiMulti;
#define DEVICE "ESP8266"
#endif

#include <InfluxDbClient.h>
#include <InfluxDbCloud.h>
#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

#include <TinyGPSPlus.h>
#include <ArduinoJson.h>  // JSON cho /coords
#include <Arduino_JSON.h> // JSONVar cho dashboard

#include <SPI.h>
#include <mcp2515.h>
#include "DHTesp.h"

#define INFLUXDB_URL "https://us-east-1-1.aws.cloud2.influxdata.com"
#define INFLUXDB_TOKEN "tmZSNP8QORsplStzzkoeBPC8RIhqiNCvEIBOLT_F2-JHAw84lPN_HF3zRN0pKyAjOEh-kpmzRsOCUFG8uWEj4g=="
#define INFLUXDB_ORG "ab0bae454287015a"
#define INFLUXDB_BUCKET "SNPS@1234@"
// Time zone info
#define TZ_INFO "UTC7"
// Tạo client InfluxDB với chứng chỉ cloud có sẵn
InfluxDBClient client(INFLUXDB_URL, INFLUXDB_ORG, INFLUXDB_BUCKET, INFLUXDB_TOKEN, InfluxDbCloud2CACert);
// Tạo điểm dữ liệu
Point Sensor("environment");
/* ─── Wi-Fi cấu hình ───────────────────────────────────────── */
const char *ssid = "Long";
const char *password = "00000000";

/* ─── GPS ───────────────────────────────────────────────────── */
constexpr uint8_t GPS_RX = 16;
constexpr uint8_t GPS_TX = 17;
HardwareSerial GPSSerial(2);
TinyGPSPlus gps;
SemaphoreHandle_t xGpsMutex;

/* ─── Web server & SSE ──────────────────────────────────────── */
AsyncWebServer server(80);
AsyncEventSource events("/events"); // /events – Server-Sent Events

/* ─── Dashboard trang HTML ──────────────────────────────────── */
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html>
<head>
  <meta charset="utf-8">
  <title>Car Sensor Dashboard - Team 2</title>
  <meta name="viewport" content="width=device-width,initial-scale=1">
  <style>
    body{font-family:Arial;margin:0;text-align:center;background:#f7f7f7}
    h1{background:#4CAF50;color:#fff;padding:20px;margin:0}
    .grid{display:grid;grid-template-columns:repeat(2,1fr);gap:20px;
          max-width:600px;margin:20px auto;padding:0 20px}
    .card{background:#fff;border-radius:8px;box-shadow:0 4px 8px #0003;
          padding:20px}
    .title{font-size:1.1rem;font-weight:bold;margin-bottom:10px}
    .value{font-size:2rem;font-weight:bold}
    .temperature{border-top:5px solid #FF5722}
    .humidity   {border-top:5px solid #2196F3}
    .light      {border-top:5px solid #FFC107}
    .rain       {border-top:5px solid #9C27B0}
  </style>
</head><body>
<h1>Car Sensor Dashboard - Team 2</h1>
<div class="grid">
  <div class="card temperature"><div class="title">Temperature</div>
       <div class="value"><span id="temperature">--</span> &deg;C</div></div>
  <div class="card humidity"><div class="title">Humidity</div>
       <div class="value"><span id="humidity">--</span> %</div></div>
  <div class="card light"><div class="title">Light</div>
       <div class="value"><span id="light">--</span></div></div>
  <div class="card rain"><div class="title">Rain</div>
       <div class="value"><span id="rain">--</span></div></div>
</div>
<script>
if (!!window.EventSource){
  const src=new EventSource('/events');
  src.addEventListener('sensor_data',e=>{
    const d=JSON.parse(e.data);
    temperature.textContent=d.temperature;
    humidity.textContent   =d.humidity;
    light.textContent      =d.light;
    rain.textContent       =d.rain;
  });
}
</script></body></html>
)rawliteral";

/* ─── DHT22 + CAN (MCP2515) quản lý qua class ──────────────── */
#define DHT_PIN 4
#define SPI_CS_PIN 5

class SensorManager
{
public:
  void begin(SemaphoreHandle_t mutex)
  {
    xMutex = mutex;
    dht.setup(DHT_PIN, DHTesp::DHT22);

    SPI.begin(); // SCK=18, MISO=19, MOSI=23
    mcp.reset();
    mcp.setBitrate(CAN_125KBPS);
    mcp.setNormalMode();
  }

  /* Task đọc DHT22 */
  static void taskReadDHT(void *pv)
  {
    auto *self = static_cast<SensorManager *>(pv);
    for (;;)
    {
      auto d = self->dht.getTempAndHumidity();
      if (!isnan(d.temperature))
      {
        xSemaphoreTake(self->xMutex, portMAX_DELAY);
        self->temperature = d.temperature;
        self->humidity = d.humidity;
        xSemaphoreGive(self->xMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(2000));
    }
  }

  /* Task nhận CAN bus */
  static void taskCANRecv(void *pv)
  {
    auto *self = static_cast<SensorManager *>(pv);
    for (;;)
    {
      struct can_frame f;
      if (self->mcp.readMessage(&f) == MCP2515::ERROR_OK && f.can_id == 0x100)
      {
        xSemaphoreTake(self->xMutex, portMAX_DELAY);
        self->rain = f.data[0];
        self->light = f.data[1];
        Serial.printf("CAN: %d %d\n", f.data[0], f.data[1]);
        xSemaphoreGive(self->xMutex);
      }
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }

  /* Truy cập an toàn từ thread khác */
  void snapshot(float &t, float &h, int &l, int &r)
  {
    xSemaphoreTake(xMutex, portMAX_DELAY);
    t = temperature;
    h = humidity;
    l = light;
    r = rain;
    xSemaphoreGive(xMutex);
  }

private:
  DHTesp dht;
  MCP2515 mcp{SPI_CS_PIN};
  SemaphoreHandle_t xMutex;

  /* shared data */
  float temperature{0}, humidity{0};
  int light{0}, rain{0};
};

/* ─── Thu thập & đẩy dữ liệu qua SSE ───────────────────────── */
class DataHandler
{
public:
  void begin(SensorManager *sm, QueueHandle_t q, SemaphoreHandle_t sem)
  {
    sensor = sm;
    queue = q;
    xSem = sem;
  }

  static void taskCollect(void *pv)
  {
    auto *self = static_cast<DataHandler *>(pv);
    for (;;)
    {
      float t, h;
      int l, r;
      self->sensor->snapshot(t, h, l, r);

      JSONVar j;
      j["temperature"] = String(t, 1);
      j["humidity"] = String(h, 1);
      j["light"] = l ? "Dark" : "Bright";
      j["rain"] = r ? "Raining" : "Dry";

      String out = JSON.stringify(j);
      if (uxQueueSpacesAvailable(self->queue))
      {
        xQueueSend(self->queue, &out, portMAX_DELAY);
        xSemaphoreGive(self->xSem);
      }
      Sensor.addField("temperature", t);
      Sensor.addField("humidity", h);
      Sensor.addField("light", l);
      Sensor.addField("rain", r);

      Serial.print("Writing: ");
      Serial.println(Sensor.toLineProtocol());

      if (wifiMulti.run() != WL_CONNECTED)
      {
        Serial.println("Wifi connection lost");
      }

      if (!client.writePoint(Sensor))
      {
        Serial.print("InfluxDB write failed: ");
        Serial.println(client.getLastErrorMessage());
      }
      Serial.println("Waitting 10s");
      vTaskDelay(pdMS_TO_TICKS(10000));
    }
  }

  static void taskSend(void *pv)
  {
    auto *self = static_cast<DataHandler *>(pv);
    String msg;
    for (;;)
    {
      if (xSemaphoreTake(self->xSem, portMAX_DELAY) == pdTRUE &&
          xQueueReceive(self->queue, &msg, 0) == pdTRUE)
      {
        if (events.count())
          events.send(msg.c_str(), "sensor_data", millis());
      }
    }
  }

private:
  SensorManager *sensor;
  QueueHandle_t queue;
  SemaphoreHandle_t xSem;
};

/* ─── Hàm gửi JSON GPS (/coords) ───────────────────────────── */
static void sendCoords(AsyncWebServerRequest *req)
{
  StaticJsonDocument<128> doc;
  if (xSemaphoreTake(xGpsMutex, portMAX_DELAY))
  {
    doc["lat"] = gps.location.isValid() ? gps.location.lat() : 0.0;
    doc["lon"] = gps.location.isValid() ? gps.location.lng() : 0.0;
    doc["sat"] = gps.satellites.value();
    doc["time"] = gps.time.value(); // HHMMSSCC UTC
    xSemaphoreGive(xGpsMutex);
  }
  String json;
  serializeJson(doc, json);

  auto *res = req->beginResponse(200, "application/json", json);
  res->addHeader("Access-Control-Allow-Origin", "*");
  res->addHeader("Access-Control-Allow-Private-Network", "true");
  req->send(res);
}

/* ─── TASK: đọc và giải mã NMEA GPS ────────────────────────── */
void taskGPS(void *pv)
{
  (void)pv;
  for (;;)
  {
    while (GPSSerial.available())
    {
      uint8_t c = GPSSerial.read();
      xSemaphoreTake(xGpsMutex, portMAX_DELAY);
      gps.encode(c);
      xSemaphoreGive(xGpsMutex);
    }
    /* giữ CPU không quá 100% nhưng vẫn đủ nhanh */
    vTaskDelay(pdMS_TO_TICKS(5));
  }
}

/* ─── Biến toàn cục FreeRTOS ───────────────────────────────── */
SensorManager sensorMgr;
DataHandler dataHdl;

SemaphoreHandle_t xMutex; // cho cảm biến
QueueHandle_t jsonQueue;
SemaphoreHandle_t xDataReady;

/* ─── Wi-Fi helper ─────────────────────────────────────────── */
void connectWifi()
{
  WiFi.mode(WIFI_STA);
  wifiMulti.addAP(ssid, password); // <--- sử dụng đúng WiFiMulti
  Serial.print("Wi-Fi…");
  int retry = 20;
  while (wifiMulti.run() != WL_CONNECTED && retry--)
  {
    delay(500);
    Serial.print('.');
  }
  Serial.println(WiFi.status() == WL_CONNECTED ? "\nConnected ✓" : "\nConnect failed ✗");
  if (WiFi.isConnected())
    Serial.println(WiFi.localIP());
}

/* ─── setup() ──────────────────────────────────────────────── */
void setup()
{
  Serial.begin(115200);
  connectWifi();

  /* GPS UART */
  GPSSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);
  xGpsMutex = xSemaphoreCreateMutex();

  /* Web routes */
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request)
            { request->send_P(200, "text/html", index_html); });

  /* REST GPS JSON */
  server.on("/coords", HTTP_GET, sendCoords);
  server.on("/coords", HTTP_OPTIONS, [](AsyncWebServerRequest *request)
            {
    AsyncWebServerResponse *res = request->beginResponse(204);
    res->addHeader("Access-Control-Allow-Origin", "*");
    res->addHeader("Access-Control-Allow-Methods", "GET, POST, OPTIONS");
    res->addHeader("Access-Control-Allow-Headers", "Content-Type");
    request->send(res); });

  /* SSE */
  events.onConnect([](AsyncEventSourceClient *c)
                   { Serial.println("SSE client joined"); });
  server.addHandler(&events);
  server.begin();

  /* --- RTOS primitives --- */
  xMutex = xSemaphoreCreateMutex();
  jsonQueue = xQueueCreate(10, sizeof(String));
  xDataReady = xSemaphoreCreateBinary();

  /* --- Initialisation --- */
  sensorMgr.begin(xMutex);
  dataHdl.begin(&sensorMgr, jsonQueue, xDataReady);
  // Đồng bộ thời gian
  timeSync(TZ_INFO, "pool.ntp.org", "time.nis.gov");

  // Kiểm tra kết nối InfluxDB
  if (client.validateConnection())
  {
    Serial.print("Connected to InfluxDB: ");
    Serial.println(client.getServerUrl());
  }
  else
  {
    Serial.print("InfluxDB connection failed: ");
    Serial.println(client.getLastErrorMessage());
  }
  /* --- Tasks --- */
  xTaskCreatePinnedToCore(SensorManager::taskReadDHT, "ReadDHT", 2048,
                          &sensorMgr, 1, nullptr, 1);
  xTaskCreatePinnedToCore(SensorManager::taskCANRecv, "CANRecv", 2048,
                          &sensorMgr, 1, nullptr, 0);

  xTaskCreatePinnedToCore(DataHandler::taskCollect, "Collector", 8192,
                          &dataHdl, 1, nullptr, 1);
  xTaskCreatePinnedToCore(DataHandler::taskSend, "SendSSE", 4096,
                          &dataHdl, 1, nullptr, 0);

  xTaskCreatePinnedToCore(taskGPS, "GPS", 4096, nullptr, 1, nullptr, 1);
}

/* ─── loop() (nhàn rỗi) ───────────────────────────────────── */
void loop()
{

  vTaskDelay(portMAX_DELAY); // tất cả logic nằm trong task
}
