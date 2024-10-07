#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHTesp.h"

volatile int state = 1;                      // Biến state sẽ xoay vòng giữa 1, 2, 3
volatile unsigned long lastDebounceTime = 0; // Thời gian cuối cùng nút được nhấn
const unsigned long debounceDelay = 250;     // Thời gian trễ khử rung 50ms

class Sensor
{
private:
  int DHT_PIN;
  int button_pin;
  int soil_sensor;
  DHTesp dhtSensor;
  int pinVol;
  int pinAmp;
  float volt, current, power, ampHours;
  unsigned long previousMillis;
  float totalCharge;
  float elapsedTime;
  String temp, hum;

public:
  friend class Load;
  friend class OLED_DISPLAY;
  void Set_Sensor_Pin();
  void Read_Sensor();
  // void Read_Button();
  static void IRAM_ATTR handleInterrupt()
  {
    unsigned long currentTime = millis(); // Thời gian hiện tại
    // Kiểm tra nếu thời gian từ lần nhấn trước vượt quá khoảng thời gian khử rung
    if ((currentTime - lastDebounceTime) > debounceDelay)
    {
      state++; // Tăng giá trị state
      if (state > 3)
      {
        state = 1; // Xoay vòng giá trị state giữa 1, 2, 3
      }
      lastDebounceTime = currentTime; // Cập nhật thời gian nhấn nút cuối cùng
    }
  }
};

class Load
{
private:
  int Buzzer;
  int water_pump;

public:
  void Set_Load_Pin();
  void Turn_On_Buzzer();
  void Turn_On_Pump();
};
#define SCREEN_WIDTH 128 // OLED width,  in pixels
#define SCREEN_HEIGHT 64 // OLED height, in pixels
#define ADDRESS 0x3C
Adafruit_SSD1306 oled(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
class OLED_DISPLAY
{
public:
  void Set_up_oled();
  void display_oled();
};
Sensor sensor;
Load load;
OLED_DISPLAY oled_1306;
void Sensor::Set_Sensor_Pin()
{
  DHT_PIN = 10;
  button_pin = 6;
  pinVol = 2;
  pinAmp = 3;
  pinMode(button_pin, INPUT_PULLUP); // Cấu hình nút nhấn với pullup
  pinMode(pinVol, INPUT);
  pinMode(pinAmp, INPUT);
  attachInterrupt(digitalPinToInterrupt(button_pin), handleInterrupt, FALLING);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
}
void OLED_DISPLAY::Set_up_oled()
{
  // initialize OLED display with I2C address 0x3C
  if (!oled.begin(SSD1306_SWITCHCAPVCC, ADDRESS))
  {
    Serial.println(F("failed to start SSD1306 OLED"));
    while (1)
      ;
  }
  delay(2000);              // wait two seconds for initializing
  oled.clearDisplay();      // clear display
  oled.setTextSize(1);      // set text size
  oled.setTextColor(WHITE); // set text color
  oled.setCursor(0, 2);     // set position to display (x,y)
}
void OLED_DISPLAY::display_oled()
{
  oled.clearDisplay();

  switch (state)
  {
  case 1:
    oled.setCursor(0, 2);
    if (sensor.temp != "NaN" && sensor.hum != "NaN")
    {
      oled.println("Temperature: " + sensor.temp + "oC"); // set text
      oled.println("Humidity: " + sensor.hum + "%");
    }
    else
    {
      oled.println("Temp/Humid Error"); // Hiển thị thông báo nếu không đọc được cảm biến
    }
    break;
  case 2:
    oled.setCursor(0, 0);
    oled.println("Soil: "); // Chưa có logic, thêm sau
    break;
  case 3:
    oled.setCursor(0, 0);
    oled.print("Volt: ");
    oled.print(sensor.volt);
    oled.print(" V");

    oled.setCursor(0, 16);
    oled.print("Current: ");
    oled.print(sensor.current);
    oled.print(" A");

    oled.setCursor(0, 32);
    oled.print("Power: ");
    oled.print(sensor.power);
    oled.print(" W");

    oled.setCursor(0, 48);
    oled.print("Energy: ");
    oled.print(sensor.ampHours);
    oled.print(" Wh");
    break;
  default:
    break;
  }

  oled.display(); // display on OLED
}
void Load::Set_Load_Pin()
{
  Buzzer = 7;
  water_pump = 1;
  pinMode(Buzzer, OUTPUT);
  pinMode(water_pump, OUTPUT);
}
void Sensor::Read_Sensor()
{
  TempAndHumidity data = sensor.dhtSensor.getTempAndHumidity();

  // Kiểm tra nếu dữ liệu trả về hợp lệ
  if (!isnan(data.temperature) && !isnan(data.humidity))
  {
    sensor.temp = String(data.temperature, 2);
    sensor.hum = String(data.humidity, 1);
  }
  else
  {
    sensor.temp = "NaN"; // Giá trị báo lỗi
    sensor.hum = "NaN";
  }

  volt = 0;
  current = 0;
  int a = 0, c = 0;
  for (int i = 0; i < 200; i++)
  {
    c = analogRead(pinAmp); // Đọc giá trị dòng
    a = analogRead(pinVol); // Đọc giá trị điện áp
    sensor.volt += a;
    sensor.current += c;
    delay(1);
  }

  // Điều chỉnh giá trị dòng điện và điện áp dựa trên cảm biến
  sensor.volt = (sensor.volt / 200) * (3.3 / 1023.0) * 11;                  // Điều chỉnh cho nguồn 3.3V
  sensor.current = ((sensor.current / 200) - 512) * (3.3 / 1023.0) / 0.066; // Điều chỉnh cho nguồn 3.3V

  if (sensor.current < 0)
    sensor.current = 0; // Bỏ qua giá trị âm (dòng rất nhỏ)

  // Tính công suất
  sensor.power = sensor.volt * sensor.current; // Công suất (Watt)

  // Tính thời gian trôi qua
  unsigned long currentMillis = millis();
  elapsedTime = (currentMillis - previousMillis) / 1000.0; // Thời gian tính bằng giây
  previousMillis = currentMillis;

  // Tính toán tổng năng lượng tiêu thụ (Watt-hour)
  totalCharge += current * elapsedTime; // Tích lũy dòng
  ampHours = totalCharge / 3600.0;      // Đổi sang Amp-giờ
}
void Load::Turn_On_Buzzer()
{
  digitalWrite(Buzzer, HIGH);
  delay(500);
  digitalWrite(Buzzer, LOW);
}
void setup()
{
  Serial.begin(9600);
  sensor.Set_Sensor_Pin();
  load.Set_Load_Pin();
  oled_1306.Set_up_oled();
}
void loop()
{
  sensor.Read_Sensor();
  oled_1306.display_oled();
  delay(1000); // Thêm thời gian chờ giữa các lần cập nhật
}
