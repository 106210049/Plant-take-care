#include <SPI.h>
#include <mcp2515.h>
#include <DHT.h>

// Khai báo Rain Sensor
#define RAIN_SENSOR_PIN PA0 // Chân đọc tín hiệu từ cảm biến mưa (digital)
int rainStatus = 0;         // Biến trạng thái mưa (0: không mưa, 1: mưa)

// Khai báo DHT11
#define DHTPIN PA1        // Chân kết nối với cảm biến DHT11
#define DHTTYPE DHT11     // Loại cảm biến DHT11
DHT dht(DHTPIN, DHTTYPE); // Khai báo đối tượng DHT

// Khai báo LDR
#define LDR_PIN PB0 // Chân analog cho cảm biến ánh sáng
int lightLevel = 0; // Biến lưu trạng thái ánh sáng (0: trời sáng, 1: trời tối)

// Khai báo HC-SR04
#define TRIG_PIN PB15    // Chân phát sóng (TRIG)
#define ECHO_PIN PB14    // Chân nhận sóng (ECHO)
long duration, distance; // Thời gian sóng siêu âm và khoảng cách đo được
int distanceStatus;      // Biến lưu trạng còi xe dựa vào khoảng cách(0: không bíp còi, 1: bíp còi)

// Khai báo MCP2515
#define SPI_CS_PIN PA4       // Chân CS của MCP2515 nối với PA4
MCP2515 mcp2515(SPI_CS_PIN); // Khởi tạo đối tượng MCP2515
struct can_frame canMsg;     // Thông điệp CAN

void setup()
{
  // Khởi tạo Serial Monitor
  Serial.begin(115200);

  // Cấu hình Rain Sensor
  pinMode(RAIN_SENSOR_PIN, INPUT); // Chân cảm biến mưa (PA0) làm đầu vào

  // Khởi tạo DHT11
  dht.begin();

  // Cấu hình HC-SR04
  pinMode(TRIG_PIN, OUTPUT); // Chân phát sóng TRIG
  pinMode(ECHO_PIN, INPUT);  // Chân nhận sóng ECHO

  // Cấu hình thông điệp CAN
  canMsg.can_id = 0x100; // ID thông điệp CAN
  canMsg.can_dlc = 5;    // 1 byte dữ liệu (trạng thái mưa) + 4 byte dữ liệu (2 byte nhiệt độ + 2 byte độ ẩm)

  // Khởi tạo MCP2515
  mcp2515.reset();                 // Reset module MCP2515
  mcp2515.setBitrate(CAN_125KBPS); // Đặt tốc độ CAN là 125Kbps
  mcp2515.setNormalMode();         // Đặt chế độ bình thường của MCP2515

  Serial.println("Sender: Waiting to send messages");

  delay(1000);
}

void loop()
{
  // Đọc trạng thái cảm biến mưa (HIGH = không mưa, LOW = mưa)
  rainStatus = digitalRead(RAIN_SENSOR_PIN) == LOW ? 1 : 0; // Nếu LOW, có mưa
  // Gửi thông điệp CAN cho Rain Sensor
  canMsg.data[0] = rainStatus;

  // Đọc dữ liệu từ cảm biến DHT11
  float humidity = dht.readHumidity();
  float temperature = dht.readTemperature();
  // Chuyển đổi giá trị float thành int trước khi gửi qua CAN
  int tempInt = (int)temperature + 26;  // Làm tròn nhiệt độ thành int
  int humidityInt = (int)humidity + 22; // Làm tròn độ ẩm thành int
  // Gửi thông điệp CAN cho DHT11 (nhiệt độ và độ ẩm)
  canMsg.data[1] = tempInt;     // Nhiệt độ
  canMsg.data[2] = humidityInt; // Độ ẩm

  // Đọc mức ánh sáng từ cảm biến LDR (giá trị 0-1023)
  lightLevel = analogRead(LDR_PIN);
  int lightStatus = (lightLevel > 50) ? 1 : 0; // Nếu ánh sáng yếu, bật đèn pha
  // Gửi thông điệp CAN cho LDR
  canMsg.data[3] = lightStatus;

  // Đọc khoảng cách từ cảm biến HC-SR04
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);
  duration = pulseIn(ECHO_PIN, HIGH);
  distance = (duration * 0.0343) / 2;           // Tính khoảng cách (cm)
  int distanceStatus = (distance < 30) ? 1 : 0; // Khoảng cách an toàn là 30cm
  // Gửi thông điệp CAN cho HC-SR04
  canMsg.data[4] = distanceStatus;

  // Gửi thông điệp CAN
  mcp2515.sendMessage(&canMsg);

  // In dữ liệu ra Serial Monitor
  Serial.print("Sent Rain Status: ");
  Serial.println(rainStatus == 1 ? "Rain" : "Dry");

  Serial.print("Sent Temperature: ");
  Serial.print(tempInt);
  Serial.print("°C, Humidity: ");
  Serial.print(humidityInt);
  Serial.println("%");

  Serial.print("Sent Light Status: ");
  Serial.println(lightStatus == 1 ? "Dark" : "Bright");

  Serial.print("Sent Distance Status: ");
  Serial.println(distanceStatus == 1 ? "Dangerous" : "Safe");

  delay(500);
}
