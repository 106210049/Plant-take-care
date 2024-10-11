#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"

volatile int state = 1;                      // Biến state sẽ xoay vòng giữa 1, 2, 3
volatile bool state_em = 0;                      // Biến state sẽ xoay vòng giữa 1, 2, 3
volatile unsigned long lastDebounceTime = 0; // Thời gian cuối cùng nút được nhấn
const unsigned long debounceDelay = 250;     // Thời gian trễ khử rung
bool buzzerTriggered = false;                // Biến cờ để kiểm tra khi nào buzzer được kích hoạt

double hum_tmp = 0, temp_tmp = 0;            // Biến lưu trữ giá trị cũ của nhiệt độ và độ ẩm
double volt_tmp = 0, current_tmp = 0;        // Biến lưu trữ giá trị cũ của điện áp và dòng điện
int moisture_tmp=0;
int previousState = 0;                       // Trạng thái trước đó của hiển thị

class Sensor
{
  private:
    int DHT_PIN;
    int button_pin,button_pin_em;
    int soil_sensor_pin;
    int moisturePercent;   // Phần trăm độ ẩm đất
    String status;         // Trạng thái độ ẩm đất (WET, OK, DRY)
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
    friend class LCD_DISPLAY;
    void Set_Sensor_Pin();
    void Read_Sensor();
    static void IRAM_ATTR handleInterrupt()
    {
      unsigned long currentTime = millis(); // Thời gian hiện tại
      if ((currentTime - lastDebounceTime) > debounceDelay)
      {
        state++;  // Tăng giá trị state
        if (state > 3) {
          state = 1;  // Xoay vòng giá trị state giữa 1, 2, 3
        }

        buzzerTriggered = true; // Kích hoạt buzzer khi nút được nhấn
        lastDebounceTime = currentTime; // Cập nhật thời gian nhấn nút cuối cùng
      }
    }

    static void IRAM_ATTR handleEM()
    {
      unsigned long currentTime = millis(); // Thời gian hiện tại
      if ((currentTime - lastDebounceTime) > debounceDelay)
      {
        state_em=!state_em;

        // buzzerTriggered = true; // Kích hoạt buzzer khi nút được nhấn
        lastDebounceTime = currentTime; // Cập nhật thời gian nhấn nút cuối cùng
      }
    }
};

class Load
{
  private:
    int Buzzer;
    int motor_left, motor_right,pwm_pin;

  public:
    void Set_Load_Pin();
    void Turn_On_Buzzer();
    void Control_Motor();
};

#define I2C_ADDR 0x27 // Address for I2C LCD
LiquidCrystal_I2C lcd(I2C_ADDR, 16, 2); // Define 16x2 LCD

class LCD_DISPLAY
{
  public:
    void Set_up_lcd();
    void display_lcd();
};

Sensor sensor;
Load load;
LCD_DISPLAY lcd_1602;

void Sensor::Set_Sensor_Pin()
{
  DHT_PIN = 10;
  button_pin = 6;
  pinVol = 2;
  pinAmp = 3;
  // button_pin_em=
  soil_sensor_pin = 4;
  pinMode(button_pin, INPUT_PULLUP); // Cấu hình nút nhấn với pullup
  pinMode(pinVol, INPUT);
  pinMode(pinAmp, INPUT);
  pinMode(button_pin_em, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(button_pin), handleInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(button_pin_em), handleEM, FALLING);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(soil_sensor_pin, INPUT);
}

void LCD_DISPLAY::Set_up_lcd()
{
  lcd.init();         // Initialize LCD
  lcd.backlight();    // Turn on the backlight
  lcd.clear();        // Clear any previous data
}

void LCD_DISPLAY::display_lcd()
{
  if (state != previousState) {
    lcd.clear();  // Chỉ gọi lcd.clear() khi state thay đổi
    previousState = state;  // Cập nhật trạng thái hiện tại
  }

  switch (state)
  {
    case 1:
      if (
      abs(sensor.temp.toDouble() - temp_tmp) >= 0.1 || 
      abs(sensor.hum.toDouble() - hum_tmp) >= 0.1
      )
      {
        lcd.clear(); // Xóa LCD khi có thay đổi nhiệt độ hoặc độ ẩm
        temp_tmp = sensor.temp.toDouble();
        hum_tmp = sensor.hum.toDouble();
      }
      if (sensor.temp != "NaN" && sensor.hum != "NaN")
      {
        lcd.setCursor(0, 0); // Set cursor to top-left
        lcd.print("Temp: " + sensor.temp);
        lcd.setCursor(11, 0);
        lcd.print("*C");
        lcd.setCursor(0, 1); // Set cursor to the second line
        lcd.print("Hum: ");
        lcd.setCursor(6, 1);
        lcd.print(sensor.hum);
        lcd.setCursor(11, 1);
        lcd.print("%");
      }
      else
      {
        lcd.setCursor(0, 0);
        lcd.print("Temp/Humid Error"); // Error message if sensor fails
      }
      break;

    case 2:
      if (abs(sensor.moisturePercent - moisture_tmp) >= 1) // Kiểm tra thay đổi độ ẩm đất
      {
        lcd.clear(); // Xóa LCD khi có thay đổi độ ẩm đất
        moisture_tmp = sensor.moisturePercent;
      }
      lcd.setCursor(0, 0);
      lcd.print("Soil: ");
      lcd.setCursor(6, 0);
      lcd.print(sensor.moisturePercent);
      lcd.setCursor(8, 0);
      lcd.print("%");
      lcd.setCursor(0, 1);
      lcd.print("Status: ");
      lcd.setCursor(8, 1);
      lcd.print(sensor.status);
      break;

    case 3:
      if (
      abs(sensor.volt - volt_tmp) >= 0.1 || 
      abs(sensor.current - current_tmp) >= 0.1
      )
      {
        lcd.clear(); // Xóa LCD khi có thay đổi điện áp hoặc dòng điện
        volt_tmp = sensor.volt;
        current_tmp = sensor.current;
      }
      lcd.setCursor(0, 0);
      lcd.print(String(sensor.volt));
      lcd.setCursor(6, 0);
      lcd.print("V");
      lcd.setCursor(9, 0);
      lcd.print(String(sensor.power));
      lcd.setCursor(14, 0);
      lcd.print("W");
      lcd.setCursor(0, 1);
      lcd.print(String(sensor.current));
      lcd.setCursor(6, 1);
      lcd.print("A");
      lcd.setCursor(9, 1);
      lcd.print(String(sensor.ampHours));
      lcd.setCursor(14, 1);
      lcd.print("Wh");
      break;

    default:
      break;
  }
}


void Load::Set_Load_Pin()
{
  Buzzer = 7;
  pwm_pin=0;
  motor_left=1;
  motor_right=10;
  pinMode(Buzzer, OUTPUT);
  pinMode(motor_left, OUTPUT);
  pinMode(motor_right, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  digitalWrite(motor_left, HIGH);
  digitalWrite(motor_right, LOW);

}
void Load::Control_Motor(){
// Map giá trị độ ẩm sang giá trị PWM (tỉ lệ nghịch)
      int pwmValue = map(sensor.moisturePercent, 0, 100, 255, 0);
      
      // Đảm bảo giá trị PWM nằm trong khoảng hợp lệ
      pwmValue = constrain(pwmValue, 0, 255);
      
      analogWrite(pwm_pin, pwmValue);  // Gửi giá trị PWM tới chân PWM
}

void Sensor::Read_Sensor()
{
  TempAndHumidity data = sensor.dhtSensor.getTempAndHumidity();

  if (!isnan(data.temperature) && !isnan(data.humidity))
  {
    sensor.temp = String(data.temperature, 1);
    sensor.hum = String(data.humidity, 1);
  }
  else
  {
    sensor.temp = "NaN";
    sensor.hum = "NaN";
  }

  volt = 0;
  current = 0;
  int a = 0, c = 0;
  for (int i = 0; i < 200; i++)
  {
    c = analogRead(pinAmp); // Read current
    a = analogRead(pinVol); // Read voltage
    sensor.volt += a;
    sensor.current += c;
    delay(1);
  }

  sensor.volt = (sensor.volt / 200) * (3.3 / 1023.0) * 11;                  // Voltage adjustment for 3.3V source
  sensor.current = ((sensor.current / 200) - 512) * (3.3 / 1023.0) / 0.066; // Current adjustment for 3.3V source

  if (sensor.current < 0)
    sensor.current = 0; // Ignore negative values

  sensor.power = sensor.volt * sensor.current; // Power (Watts)

  unsigned long currentMillis = millis();
  elapsedTime = (currentMillis - previousMillis) / 1000.0; // Time in seconds
  previousMillis = currentMillis;

  totalCharge += current * elapsedTime; // Accumulate current over time
  ampHours = totalCharge / 3600.0;      // Convert to Amp-hours

  int SensorValue = analogRead(soil_sensor_pin); // Placeholder for soil sensor logic
  sensor.moisturePercent = map(SensorValue, 0, 4095, 0, 100);  // Tính phần trăm độ ẩm
  if (moisturePercent > 70) {
    sensor.status = "WET";
  } else if (moisturePercent < 30) {
    sensor.status = "DRY";
  } else {
    sensor.status = "OK";
  }
}

void Load::Turn_On_Buzzer()
{
  // digitalWrite(Buzzer, HIGH);   // Bật buzzer
  // delay(500);                   // Để buzzer kêu trong 500ms
  // digitalWrite(Buzzer, LOW);    // Tắt buzzer
  tone(Buzzer, 3000, 100);
}

void setup()
{
  Serial.begin(9600);
  sensor.Set_Sensor_Pin();
  load.Set_Load_Pin();
  lcd_1602.Set_up_lcd();
}

void loop()
{
  if(state_em){
    lcd.clear();
    lcd.setCursor(0,0);
    lcd.print("Emergency");
  }else{
  if (buzzerTriggered) {
    load.Turn_On_Buzzer();    // Kích hoạt buzzer nếu đã nhấn nút
    buzzerTriggered = false;  // Reset biến cờ để tránh buzzer kêu nhiều lần
  }

  sensor.Read_Sensor();
  lcd_1602.display_lcd();
  load.Control_Motor();
  delay(1000); // Delay between updates
  }
}
