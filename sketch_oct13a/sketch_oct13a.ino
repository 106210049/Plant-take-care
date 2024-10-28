#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include "DHTesp.h"

volatile int state = 1;                      // Biến state sẽ xoay vòng giữa 1, 2, 3
// volatile bool state_em = 0;                      // Biến state sẽ xoay vòng giữa 1, 2, 3
volatile unsigned long lastDebounceTime = 0; // Thời gian cuối cùng nút được nhấn
const unsigned long debounceDelay = 250;     // Thời gian trễ khử rung
bool buzzerTriggered = false;                // Biến cờ để kiểm tra khi nào buzzer được kích hoạt

double hum_tmp = 0, temp_tmp = 0;            // Biến lưu trữ giá trị cũ của nhiệt độ và độ ẩm
double volt_tmp = 0, current_tmp = 0;        // Biến lưu trữ giá trị cũ của điện áp và dòng điện
int moisture_tmp = 0;
int target_value_tmp = 0;
int previousState = 0;                       // Trạng thái trước đó của hiển thị

volatile bool mode_tmp=false;
unsigned long lastBlinkTime;
volatile long lcdBlinking=false;
//volatile long buzzerStartTime;
volatile bool emergencyMode = false;         // Emergency mode status

//volatile em_mode=false;

volatile unsigned long buttonPressTime = 0;   // Time when button press starts
volatile bool longPressDetected = false;     // Flag to check if long press is detected
void IRAM_ATTR handleInterrupt() {
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

  void IRAM_ATTR EMhandleInterrupt() {
      unsigned long currentTime = millis(); // Thời gian hiện tại
      if ((currentTime - lastDebounceTime) > debounceDelay)
      {
        
        emergencyMode=!emergencyMode;
//        buzzerTriggered = true; // Kích hoạt buzzer khi nút được nhấn
        lastDebounceTime = currentTime; // Cập nhật thời gian nhấn nút cuối cùng
      }
  }
class Sensor // Quản lý các thông tin về cảm biến ( chân cảm biến, giá trị cảm biến, hàm đọc cảm biến)
{
  private:
    int DHT_PIN; // chân cảm biến nhiệt độ
    int button_pin; // chân nút nhấn chuyển trạng thái hiển thị
    int em_button_pin; // chân nút nhấn trạng thái emergency
    int target_pin; // chân set giá trị độ ẩm
    int soil_sensor_pin; // châm cảm biến độ ẩm đất
    int pinVol; // chân đo điện áp
    int pinAmp; // chân đo dòng
    
    int target_value; // giá trị độ ẩm set
    int moisturePercent;   // Phần trăm độ ẩm đất
    String status;         // Trạng thái độ ẩm đất (WET, OK, DRY)
    DHTesp dhtSensor;
    
    float volt, current, power, wattHours;
    unsigned long previousMillis;
    float totalEnergy;
    float elapsedTime;
    String temp, hum;

  public:
    friend class Load;
    friend class LCD_DISPLAY;
    void Set_Sensor_Pin();
    void Read_Sensor();
    
};

class Load // Class quản lý các tải, hiển thị( motor, LCD 16x02)
{
  private:
    int Buzzer;
    int motor_left, motor_right, pwm_pin;

  public:
    void Set_Load_Pin();
    void Turn_On_Buzzer();
    void Control_Motor();
    void Stop_Motor();
    
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

void LCD_DISPLAY::Set_up_lcd()
{
  Wire.begin(21,22);
  lcd.init();         // Initialize LCD
  lcd.backlight();    // Turn on the backlight
  lcd.clear();        // Clear any previous data
  
}
void LCD_DISPLAY::display_lcd(){
  
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
      if (abs(sensor.moisturePercent - moisture_tmp) >= 1|| abs(sensor.target_value-target_value_tmp)>=1) // Kiểm tra thay đổi độ ẩm đất
      {
        lcd.clear(); // Xóa LCD khi có thay đổi độ ẩm đất
        moisture_tmp = sensor.moisturePercent;
        target_value_tmp=sensor.target_value;
      }
      lcd.setCursor(0, 0);
      lcd.print("Soil: ");
      lcd.setCursor(6, 0);
      lcd.print(sensor.moisturePercent);
      lcd.setCursor(9, 0);
      lcd.print("%");
      lcd.setCursor(11, 0);

      lcd.print("- "+sensor.status);
      lcd.setCursor(0,1);
      lcd.print("Target: ");
      lcd.setCursor(8, 1);
      lcd.print(sensor.target_value);
      lcd.setCursor(11, 1);
      lcd.print("%");
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
      lcd.setCursor(15, 0);
      lcd.print("W");
      lcd.setCursor(0, 1);
      lcd.print(String(sensor.current));
      lcd.setCursor(6, 1);
      lcd.print("mA");
      lcd.setCursor(9, 1);
      lcd.print(String(sensor.wattHours));
      lcd.setCursor(14, 1);
      lcd.print("Wh");
      break;
      default:
      break;
  }
}

void Sensor::Set_Sensor_Pin() // set chân cho các cảm biến
{
  DHT_PIN = 18;
  button_pin = 27;
  pinVol = 32;
  pinAmp = 35;
  target_pin= 25;
  soil_sensor_pin = 4;
  em_button_pin = 19;
  pinMode(button_pin, INPUT_PULLUP); // Cấu hình nút nhấn với pullup
  attachInterrupt(digitalPinToInterrupt(button_pin), handleInterrupt, FALLING);
  pinMode(em_button_pin, INPUT_PULLUP); // Cấu hình nút nhấn với pullup
  attachInterrupt(digitalPinToInterrupt(em_button_pin), EMhandleInterrupt, FALLING);
  pinMode(pinVol, INPUT);
  pinMode(pinAmp, INPUT);
  pinMode(target_pin,INPUT);
  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);
  pinMode(soil_sensor_pin, INPUT);
}
void Sensor::Read_Sensor() // đọc giá trị cảm biến
{
  TempAndHumidity data = sensor.dhtSensor.getTempAndHumidity();
  sensor.temp = String(data.temperature, 1);
  sensor.hum = String(data.humidity, 1);
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
  int SensorValue = analogRead(soil_sensor_pin); // Placeholder for soil sensor logic
  sensor.moisturePercent = map(SensorValue, 0, 4095, 100, 0);  // Tính phần trăm độ ẩm
  if (moisturePercent > 70) {
    sensor.status = "WET";
  } else if (moisturePercent < 30) {
    sensor.status = "DRY";
  } else {
    sensor.status = "OK";
  }
  int setvalue=analogRead(sensor.target_pin);
  sensor.target_value=map(setvalue, 0, 4095, 0,100);
  
//  Serial.print("Set Value");
//  Serial.println(setvalue);
//  Serial.print("Target value");
//  Serial.println(sensor.target_value);
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

  // Convert readings to voltage and current values for a 5V system
sensor.volt = (sensor.volt / 200.0) * (5.0 / 1023.0) * 11; // 11 is the voltage divider ratio
sensor.current = ((sensor.current / 200.0) - 512) * (5.0 / 1023.0) / 0.066; // 0.066 is the current sensor sensitivity

if (sensor.current < 0)
    sensor.current = 0; // Ignore negative values

sensor.power = sensor.volt/1000 * sensor.current; // Power in Watts

unsigned long currentMillis = millis();
elapsedTime = (currentMillis - previousMillis) / 1000.0; // Time in seconds
previousMillis = currentMillis;

// Accumulate power over time for watt-hours
totalEnergy += sensor.power * elapsedTime; // Accumulate power over time
wattHours = totalEnergy / 3600.0; // Convert to Watt-hours (Wh)
  
}


void Load::Set_Load_Pin() // set chân tải 
{
  Buzzer = 2;
  pwm_pin = 13;
  motor_left = 12;
  motor_right = 14;
  pinMode(Buzzer, OUTPUT);
  pinMode(motor_left, OUTPUT);
  pinMode(motor_right, OUTPUT);
  pinMode(pwm_pin, OUTPUT);
  

}

void Load::Turn_On_Buzzer()
{
  tone(Buzzer, 3000, 300);
}
void Load::Stop_Motor(){
    digitalWrite(motor_left,LOW);
    digitalWrite(motor_right, LOW);
  }
void Load::Control_Motor() {
  
    digitalWrite(motor_left, HIGH);
    digitalWrite(motor_right, LOW);
  // Map giá trị độ ẩm sang giá trị PWM (tỉ lệ nghịch)
    int pwmValue = map(sensor.moisturePercent, 0, sensor.target_value, 255, 0);

  // Đảm bảo giá trị PWM nằm trong khoảng hợp lệ
    pwmValue = constrain(pwmValue, 0, 255);

    analogWrite(pwm_pin, pwmValue);  // Gửi giá trị PWM tới chân PWM
  
  
}

void setup()
{
  Serial.begin(115200);
  sensor.Set_Sensor_Pin();
  load.Set_Load_Pin();
  lcd_1602.Set_up_lcd();
}
void loop(){
  unsigned long currentTime = millis(); // Get current time
  
  // Check if the mode has changed to update the display and reset variables
  if (mode_tmp != emergencyMode) {
    lcd.clear();
    mode_tmp = emergencyMode;
  }

  // Emergency Mode Logic
  if (emergencyMode) {
    load.Stop_Motor();
    lcd.setCursor(1, 0);
    lcd.print("Emergency mode");
    
    // Blinking and continuous buzzer sound every 500ms
    if (currentTime - lastBlinkTime >= 500) {
      lcdBlinking = !lcdBlinking;
      
      if (lcdBlinking) {
        lcd.backlight();
        load.Turn_On_Buzzer(); // Continuous beeping every 500ms
      } else {
        lcd.noBacklight();
      }
      
      lastBlinkTime = currentTime;
    }
  } 
  
  // Normal Mode Logic
  else {
    lcd.backlight(); // Ensure backlight is on in normal mode

    // Brief beep when changing display state
    if (buzzerTriggered) {
      load.Turn_On_Buzzer();  // Sound buzzer briefly
      buzzerTriggered = false; // Reset flag to prevent continuous beeping
    }

    // Read sensor data and update display
    sensor.Read_Sensor();
    lcd_1602.display_lcd();
    load.Control_Motor();
    delay(10);  // Delay between updates
  }

}
