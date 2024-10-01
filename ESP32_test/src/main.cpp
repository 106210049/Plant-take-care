#include <Arduino.h>
#include <math.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "DHTesp.h"
class Sensor
{
private:
  int DHT_PIN;
  int soil_sensor;
  DHTesp dhtSensor;
  String temp, hum;

public:
  friend class Load;
  friend class OLED_DISPLAY;
  void Set_Sensor_Pin();
  void Read_Sensor();
};

class Load
{
private:
  // int Lamp_Pin;
  // int Motor_Pin_Left;
  // int Motor_Pin_Right;
  int water_pump;

public:
  void
  Set_Load_Pin();
  // void Turn_On_Lamp();
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
  DHT_PIN = 0;
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
  oled.setCursor(0, 2);

  oled.println("Temperature: " + sensor.temp + "oC"); // set text
  oled.println("Humidity: " + sensor.hum + "%");
  oled.display(); // display on OLED
}

void Sensor::Read_Sensor()
{
  TempAndHumidity data = sensor.dhtSensor.getTempAndHumidity();
  sensor.temp = String(data.temperature, 2);
  sensor.hum = String(data.humidity, 1);
}
void setup()
{
  Serial.begin(9600);
  sensor.Set_Sensor_Pin();
  oled_1306.Set_up_oled();
}
void loop()
{
  sensor.Read_Sensor();
  oled_1306.display_oled();
}