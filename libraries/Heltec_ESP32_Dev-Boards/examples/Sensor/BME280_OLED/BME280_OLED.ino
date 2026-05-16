#include <Wire.h>          // I2C communication
#include <Adafruit_BME280.h>  // BME280 sensor library
#include "HT_SSD1306Wire.h" // OLED display

// I2C instance for BME280
TwoWire Wire2(1);

// Sensor and display objects
Adafruit_BME280 bme;  // Changed to BME280 object
static SSD1306Wire display(0x3c, 500000, SDA_OLED, SCL_OLED, GEOMETRY_128_64, RST_OLED);

void setup() {
  Serial.begin(115200);     // Start serial for debugging
  delay(500);              // Stabilization delay
  
  Wire2.begin(41,42);      // Initialize I2C on pins 41(SDA),42(SCL)
  
  // Try both common I2C addresses for BME280
  if(!bme.begin(0x76, &Wire2)) {  // Try address 0x76 first
    Serial.println("Trying address 0x76...");
    if(!bme.begin(0x77, &Wire2)) {  // Try address 0x77 if 0x76 fails
      Serial.println("BME280 sensor not found! Check connections.");
      while(1);              // Halt if failed
    }
  }
  
  Serial.println("BME280 sensor initialized successfully!");
  delay(500);
  
  // Configure BME280 settings - 使用正确的常量名称
  // 注意：Adafruit BME280库使用不同的常量命名
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,     // Temperature
                  Adafruit_BME280::SAMPLING_X16,    // Pressure
                  Adafruit_BME280::SAMPLING_X1,     // Humidity
                  Adafruit_BME280::FILTER_X16,
                  Adafruit_BME280::STANDBY_MS_500);  // 修正这里
  
  // OLED setup
  display.init();
  display.clear();
  display.display();
  display.setContrast(255);
  display.setFont(ArialMT_Plain_10);
  display.setTextAlignment(TEXT_ALIGN_LEFT);
  display.drawString(0, 0, "BME280 Sensor Test");
  display.display();
}

void loop() {
  // Read sensor data
  float temp = bme.readTemperature();          // Celsius
  float pressure = bme.readPressure() / 100.0; // hPa
  float humidity = bme.readHumidity();         // Relative humidity %
  
  // Serial output
  Serial.print("Temperature: ");
  Serial.print(temp, 1);  // 1 decimal place
  Serial.print(" C, Pressure: ");
  Serial.print(pressure, 1);  // 1 decimal place
  Serial.print(" hPa, Humidity: ");
  Serial.print(humidity, 1);  // 1 decimal place
  Serial.println(" %");
  
  // OLED output - updated to show all three measurements
  display.clear();
  display.drawString(0, 0, "BME280 Sensor");
  display.drawString(0, 15, "Temp: " + String(temp, 1) + " C");
  display.drawString(0, 30, "Press: " + String(pressure, 1) + " hPa");
  display.drawString(0, 45, "Humid: " + String(humidity, 1) + " %");
  display.display();
  
  delay(3000); // Update every 3 seconds
}