#include <Arduino.h>
#include <Servo.h>
#include <SPI.h>
#include <mcp2515.h>
#include <FreeRTOS.h>
#include <task.h>
#include <queue.h>
#include <semphr.h>

#define SERVO_PIN PA1
#define LED1_PIN PB0
#define LED2_PIN PB1
#define WARNING_LED_PIN PB10
#define BUZZER_PIN PA0
#define SPI_CS_PIN PA4

// Struct dùng cho queue
struct SensorData {
  int rainStatus;
  int lightStatus;
  int distanceStatus;
};

// ==================== CAN Receiver ====================
class CANReceiver {
private:
  MCP2515 mcp;
  QueueHandle_t queue;
  SemaphoreHandle_t notifySemaphore;

public:
  CANReceiver(uint8_t csPin, QueueHandle_t q, SemaphoreHandle_t sema)
    : mcp(csPin), queue(q), notifySemaphore(sema) {}

  void begin() {
    mcp.reset();
    mcp.setBitrate(CAN_125KBPS);
    mcp.setNormalMode();
  }

  void task() {
    struct can_frame canMsg;
    while (true) {
      if (mcp.readMessage(&canMsg) == MCP2515::ERROR_OK && canMsg.can_id == 0x100) {
        SensorData data;
        data.rainStatus = canMsg.data[0];
        data.lightStatus = canMsg.data[1];
        data.distanceStatus = canMsg.data[2];
        xQueueSend(queue, &data, portMAX_DELAY);
        xSemaphoreGive(notifySemaphore);
      }
      vTaskDelay(pdMS_TO_TICKS(200));
    }
  }

  static void taskWrapper(void* param) {
    static_cast<CANReceiver*>(param)->task();
  }
};

// ==================== Servo Controller ====================
class ServoController {
private:
  Servo servo;
  SemaphoreHandle_t notifySemaphore;
  QueueHandle_t queue;
  SemaphoreHandle_t mutex;

public:
  ServoController(uint8_t pin, QueueHandle_t q, SemaphoreHandle_t sema, SemaphoreHandle_t mtx)
    : queue(q), notifySemaphore(sema), mutex(mtx) {
    servo.attach(pin);
    servo.write(0);
  }

  void task() {
    SensorData data;
    while (true) {
      if (xSemaphoreTake(notifySemaphore, portMAX_DELAY) == pdTRUE &&
          xQueuePeek(queue, &data, 0) == pdTRUE) {
        if (data.rainStatus == 1) {
          if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            for (int pos = 0; pos <= 180; pos += 2) {
              servo.write(pos);
              vTaskDelay(pdMS_TO_TICKS(10));
            }
            for (int pos = 180; pos >= 0; pos -= 2) {
              servo.write(pos);
              vTaskDelay(pdMS_TO_TICKS(10));
            }
            xSemaphoreGive(mutex);
          }
        } else {
          if (xSemaphoreTake(mutex, portMAX_DELAY) == pdTRUE) {
            servo.write(0);
            xSemaphoreGive(mutex);
          }
        }
      }
    }
  }

  static void taskWrapper(void* param) {
    static_cast<ServoController*>(param)->task();
  }
};

// ==================== Alert Handler ====================
class AlertHandler {
private:
  QueueHandle_t queue;
  SemaphoreHandle_t notifySemaphore;

public:
  AlertHandler(QueueHandle_t q, SemaphoreHandle_t sema)
    : queue(q), notifySemaphore(sema) {
    pinMode(LED1_PIN, OUTPUT);
    pinMode(LED2_PIN, OUTPUT);
    pinMode(WARNING_LED_PIN, OUTPUT);
    pinMode(BUZZER_PIN, OUTPUT);
  }

  void task() {
    SensorData data;
    while (true) {
      if (xSemaphoreTake(notifySemaphore, portMAX_DELAY) == pdTRUE &&
          xQueueReceive(queue, &data, 0) == pdTRUE) {

        if (data.lightStatus == 1) {
          digitalWrite(LED1_PIN, HIGH);
          digitalWrite(LED2_PIN, HIGH);
          Serial.println("Light: Dark");
        } else {
          digitalWrite(LED1_PIN, LOW);
          digitalWrite(LED2_PIN, LOW);
          Serial.println("Light: Bright");
        }

        if (data.distanceStatus == 1) {
          digitalWrite(BUZZER_PIN, HIGH);
          digitalWrite(WARNING_LED_PIN, HIGH);
          vTaskDelay(pdMS_TO_TICKS(300));
          digitalWrite(WARNING_LED_PIN, LOW);
          Serial.println("Distance: Dangerous");
        } else {
          digitalWrite(BUZZER_PIN, LOW);
          Serial.println("Distance: Safe");
        }
      }
    }
  }

  static void taskWrapper(void* param) {
    static_cast<AlertHandler*>(param)->task();
  }
};

// ==================== Global Objects ====================
QueueHandle_t sensorQueue;
SemaphoreHandle_t notifySemaphore;
SemaphoreHandle_t servoMutex;

CANReceiver* canReceiver;
ServoController* servoCtrl;
AlertHandler* alertHandler;

void setup() {
  Serial.begin(115200);

  // Tạo tài nguyên RTOS
  sensorQueue = xQueueCreate(5, sizeof(SensorData));
  notifySemaphore = xSemaphoreCreateBinary();
  servoMutex = xSemaphoreCreateMutex();

  // Khởi tạo đối tượng
  canReceiver = new CANReceiver(SPI_CS_PIN, sensorQueue, notifySemaphore);
  servoCtrl = new ServoController(SERVO_PIN, sensorQueue, notifySemaphore, servoMutex);
  alertHandler = new AlertHandler(sensorQueue, notifySemaphore);

  canReceiver->begin();

  // Tạo task
  xTaskCreate(CANReceiver::taskWrapper, "CAN", 256, canReceiver, 2, NULL);
  xTaskCreate(ServoController::taskWrapper, "Servo", 256, servoCtrl, 2, NULL);
  xTaskCreate(AlertHandler::taskWrapper, "Alert", 256, alertHandler, 2, NULL);

  Serial.println("System Ready - OOP + FreeRTOS");
  vTaskStartScheduler();
}

void loop() {
  // Không dùng loop trong FreeRTOS
}
