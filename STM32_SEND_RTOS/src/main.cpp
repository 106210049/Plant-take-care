#include <SPI.h>
#include <mcp2515.h>
#include <Arduino.h>
#include <STM32FreeRTOS.h>

#define RAIN_SENSOR_PIN PA0
#define LDR_PIN PB0
#define TRIG_PIN PB15
#define ECHO_PIN PB14
#define SPI_CS_PIN PA4

// ---------------- Shared Sensor Data ----------------
struct SensorData
{
  int rain;
  int light;
  int distanceStatus;
};

// ---------------- Sensor Manager ----------------
class SensorManager
{
public:
  SensorData data;
  SemaphoreHandle_t dataMutex;
  SemaphoreHandle_t serialMutex;

  SensorManager()
  {
    data = {0, 0, 0};
    dataMutex = xSemaphoreCreateMutex();
    serialMutex = xSemaphoreCreateMutex();
  }

  ~SensorManager()
  {
    vSemaphoreDelete(dataMutex);
    vSemaphoreDelete(serialMutex);
  }

  void updateRain(int value)
  {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      data.rain = value;
      xSemaphoreGive(dataMutex);
    }
  }

  void updateLight(int value)
  {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      data.light = value;
      xSemaphoreGive(dataMutex);
    }
  }

  void updateDistance(int value)
  {
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      data.distanceStatus = value;
      xSemaphoreGive(dataMutex);
    }
  }

  SensorData getData()
  {
    SensorData copy;
    if (xSemaphoreTake(dataMutex, portMAX_DELAY) == pdTRUE)
    {
      copy = data;
      xSemaphoreGive(dataMutex);
    }
    return copy;
  }
};

// ---------------- Base Sensor Class ----------------
class SensorBase
{
public:
  SensorManager *manager;

  SensorBase(SensorManager *mgr) : manager(mgr) {}

  virtual void run() = 0;
};

// ---------------- Rain Sensor ----------------
class RainSensor : public SensorBase
{
  uint8_t pin;

public:
  RainSensor(uint8_t p, SensorManager *mgr) : SensorBase(mgr), pin(p)
  {
    pinMode(pin, INPUT);
  }

  void run() override
  {
    for (;;)
    {
      int value = digitalRead(pin) == LOW ? 1 : 0;
      manager->updateRain(value);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
};

// ---------------- Light Sensor ----------------
class LightSensor : public SensorBase
{
  uint8_t pin;

public:
  LightSensor(uint8_t p, SensorManager *mgr) : SensorBase(mgr), pin(p) {}

  void run() override
  {
    for (;;)
    {
      int level = analogRead(pin);
      int value = (level > 50) ? 1 : 0;
      manager->updateLight(value);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
};

// ---------------- Distance Sensor ----------------
class DistanceSensor : public SensorBase
{
  uint8_t trig, echo;

public:
  DistanceSensor(uint8_t t, uint8_t e, SensorManager *mgr)
      : SensorBase(mgr), trig(t), echo(e)
  {
    pinMode(trig, OUTPUT);
    pinMode(echo, INPUT);
  }

  void run() override
  {
    for (;;)
    {
      digitalWrite(trig, LOW);
      delayMicroseconds(2);
      digitalWrite(trig, HIGH);
      delayMicroseconds(10);
      digitalWrite(trig, LOW);
      long duration = pulseIn(echo, HIGH);
      long dist = (duration * 0.0343) / 2;
      int value = (dist < 30) ? 1 : 0;
      manager->updateDistance(value);
      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
};

// ---------------- CAN Transmitter ----------------
class CanTransmitter
{
  MCP2515 &can;
  SensorManager *manager;
  struct can_frame msg;

public:
  CanTransmitter(MCP2515 &c, SensorManager *mgr) : can(c), manager(mgr)
  {
    msg.can_id = 0x100;
    msg.can_dlc = 5;
  }

  void run()
  {
    for (;;)
    {
      SensorData data = manager->getData();

      msg.data[0] = data.rain;
      msg.data[1] = data.light;
      msg.data[2] = data.distanceStatus;

      can.sendMessage(&msg);

      if (xSemaphoreTake(manager->serialMutex, pdMS_TO_TICKS(100)) == pdTRUE)
      {
        Serial.print("Rain: ");
        Serial.print(data.rain == 1 ? "Rain" : "Dry");
        Serial.print(" | Light: ");
        Serial.print(data.light == 1 ? "Dark" : "Bright");
        Serial.print(" | Distance: ");
        Serial.println(data.distanceStatus == 1 ? "Dangerous" : "Safe");
        xSemaphoreGive(manager->serialMutex);
      }

      vTaskDelay(pdMS_TO_TICKS(500));
    }
  }
};

// ---------------- Tasks Wrappers ----------------
SensorManager manager;
MCP2515 mcp2515(SPI_CS_PIN);

RainSensor rain(RAIN_SENSOR_PIN, &manager);
LightSensor light(LDR_PIN, &manager);
DistanceSensor distance(TRIG_PIN, ECHO_PIN, &manager);
CanTransmitter canTx(mcp2515, &manager);

void RainTaskWrapper(void *pvParameters) { rain.run(); }
void LightTaskWrapper(void *pvParameters) { light.run(); }
void DistanceTaskWrapper(void *pvParameters) { distance.run(); }
void CanSendTaskWrapper(void *pvParameters) { canTx.run(); }

// ---------------- Setup ----------------
void setup()
{
  Serial.begin(115200);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_125KBPS);
  mcp2515.setNormalMode();

  if (manager.dataMutex == NULL || manager.serialMutex == NULL)
  {
    Serial.println("Failed to create mutexes");
    while (1)
      ;
  }

  xTaskCreate(RainTaskWrapper, "Rain", 128, NULL, 1, NULL);
  xTaskCreate(LightTaskWrapper, "Light", 128, NULL, 1, NULL);
  xTaskCreate(DistanceTaskWrapper, "Distance", 128, NULL, 1, NULL);
  xTaskCreate(CanSendTaskWrapper, "CAN", 256, NULL, 2, NULL);

  vTaskStartScheduler();
}

void loop()
{
  // Unused in FreeRTOS
}
