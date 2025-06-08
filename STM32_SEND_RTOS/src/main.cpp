#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <STM32FreeRTOS.h>

// Cảm biến
#define RAIN_SENSOR_PIN PA0
#define LDR_PIN PB0
#define TRIG_PIN PB15
#define ECHO_PIN PB14

#define SPI_CS_PIN PA4
MCP2515 mcp2515(SPI_CS_PIN);
struct can_frame canMsg;

// Biến sensor dùng chung
typedef struct
{
  int rain;
  int light;
  int distanceStatus;
} SensorData;

SensorData sharedData; // Dữ liệu chia sẻ giữa các task

// Mutex bảo vệ truy cập sharedData
SemaphoreHandle_t dataMutex;
SemaphoreHandle_t serialMutex;

void RainTask(void *pvParameters)
{
  for (;;)
  {
    int rain = digitalRead(RAIN_SENSOR_PIN) == LOW ? 1 : 0;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      sharedData.rain = rain;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void LightTask(void *pvParameters)
{
  for (;;)
  {
    int level = analogRead(LDR_PIN);
    int light = (level > 50) ? 1 : 0;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      sharedData.light = light;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void DistanceTask(void *pvParameters)
{
  for (;;)
  {
    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    long duration = pulseIn(ECHO_PIN, HIGH);
    long distance = (duration * 0.0343) / 2;
    int status = (distance < 30) ? 1 : 0;

    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      sharedData.distanceStatus = status;
      xSemaphoreGive(dataMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void CanSendTask(void *pvParameters)
{
  SensorData localData;

  for (;;)
  {
    // Copy an toàn từ biến chung
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      localData = sharedData;
      xSemaphoreGive(dataMutex);
    }

    // Gửi qua CAN
    canMsg.can_id = 0x100;
    canMsg.can_dlc = 5;
    canMsg.data[0] = localData.rain;
    canMsg.data[3] = localData.light;
    canMsg.data[4] = localData.distanceStatus;

    mcp2515.sendMessage(&canMsg);

    // In serial
    if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
    {
      Serial.print("Rain: ");
      Serial.print(localData.rain == 1 ? "Rain" : "Dry");
      Serial.print(" | Light: ");
      Serial.print(localData.light == 1 ? "Dark" : "Bright");
      Serial.print(" | Distance: ");
      Serial.println(localData.distanceStatus == 1 ? "Dangerous" : "Safe");
      xSemaphoreGive(serialMutex);
    }

    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

void setup()
{
  Serial.begin(115200);
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  dataMutex = xSemaphoreCreateMutex();
  serialMutex = xSemaphoreCreateMutex();

  if (dataMutex == NULL || serialMutex == NULL)
  {
    Serial.println("Failed to create mutexes");
    while (1)
      ;
  }

  // Khởi tạo shared data lần đầu
  sharedData = {0, 0, 0};

  xTaskCreate(RainTask, "Rain", 128, NULL, 1, NULL);
  xTaskCreate(LightTask, "Light", 128, NULL, 1, NULL);
  xTaskCreate(DistanceTask, "Distance", 128, NULL, 1, NULL);
  xTaskCreate(CanSendTask, "CAN", 256, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop()
{
  // Không dùng trong FreeRTOS
}