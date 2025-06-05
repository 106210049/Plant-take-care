#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <STM32FreeRTOS.h>

// Cảm biến mưa
#define RAIN_SENSOR_PIN PA0
int rainStatus = 0;

// Cảm biến LDR
#define LDR_PIN PB0
int lightLevel = 0;

// Cảm biến khoảng cách
#define TRIG_PIN PB15
#define ECHO_PIN PB14
long duration, distance;

// MCP2515
#define SPI_CS_PIN PA4
MCP2515 mcp2515(SPI_CS_PIN);
struct can_frame canMsg;

// Queue và semaphore
QueueHandle_t sensorQueue;
SemaphoreHandle_t serialMutex;

// Kiểu dữ liệu gói tin sensor
typedef struct
{
  int rain;
  int light;
  int distanceStatus;
} SensorData;

// Task đo mưa
void RainTask(void *pvParameters)
{
  for (;;)
  {
    SensorData data;
    data.rain = digitalRead(RAIN_SENSOR_PIN) == LOW ? 1 : 0;
    xQueueSend(sensorQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task đo ánh sáng
void LightTask(void *pvParameters)
{
  for (;;)
  {
    SensorData data;
    int level = analogRead(LDR_PIN);
    data.light = (level > 50) ? 1 : 0;
    xQueueSend(sensorQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task đo khoảng cách
void DistanceTask(void *pvParameters)
{
  for (;;)
  {
    SensorData data;

    digitalWrite(TRIG_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIG_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIG_PIN, LOW);
    duration = pulseIn(ECHO_PIN, HIGH);
    distance = (duration * 0.0343) / 2;
    data.distanceStatus = (distance < 30) ? 1 : 0;

    xQueueSend(sensorQueue, &data, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(500));
  }
}

// Task gửi CAN
void CanSendTask(void *pvParameters)
{
  SensorData data;

  for (;;)
  {
    // Lấy dữ liệu mới nhất từ queue
    if (xQueueReceive(sensorQueue, &data, portMAX_DELAY) == pdPASS)
    {
      canMsg.can_id = 0x100;
      canMsg.can_dlc = 5;
      canMsg.data[0] = data.rain;
      canMsg.data[3] = data.light;
      canMsg.data[4] = data.distanceStatus;

      mcp2515.sendMessage(&canMsg);

      if (xSemaphoreTake(serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        Serial.print("Rain: ");
        Serial.print(data.rain == 1 ? "Rain" : "Dry");
        Serial.print(" | Light: ");
        Serial.print(data.light == 1 ? "Dark" : "Bright");
        Serial.print(" | Distance: ");
        Serial.println(data.distanceStatus == 1 ? "Dangerous" : "Safe");
        xSemaphoreGive(serialMutex);
      }
    }
  }
}

void setup()
{
  Serial.begin(9600);
  pinMode(RAIN_SENSOR_PIN, INPUT);
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  sensorQueue = xQueueCreate(10, sizeof(SensorData));
  serialMutex = xSemaphoreCreateMutex();

  if (sensorQueue == NULL || serialMutex == NULL)
  {
    Serial.println("Failed to create queue or mutex");
    while (1)
      ;
  }

  // Tạo các task
  xTaskCreate(RainTask, "Rain", 128, NULL, 1, NULL);
  xTaskCreate(LightTask, "Light", 128, NULL, 1, NULL);
  xTaskCreate(DistanceTask, "Distance", 128, NULL, 1, NULL);
  xTaskCreate(CanSendTask, "CAN", 256, NULL, 2, NULL);

  // Khởi động hệ điều hành RTOS
  vTaskStartScheduler();
}

void loop()
{
  // Không dùng loop trong FreeRTOS
}
