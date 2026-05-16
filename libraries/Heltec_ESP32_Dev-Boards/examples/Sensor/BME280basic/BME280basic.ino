#include <Wire.h>
#include <Adafruit_BME280.h>

// Pin definitions
#define BME_SDA 41      // BME280 SDA pin
#define BME_SCL 42      // BME280 SCL pin
#define VEXT_PIN 36     // External power control pin

// Create second I2C instance
TwoWire bmeWire(1);

// Create sensor object
Adafruit_BME280 bme;

void setup() {
  Serial.begin(115200);
  delay(1000);
  
  Serial.println("BME280 Basic Sensor Test");
  Serial.println("========================");
  
  // Enable external power (for Heltec V3)
  pinMode(VEXT_PIN, OUTPUT);
  digitalWrite(VEXT_PIN, HIGH);
  delay(100);  // Wait for power stabilization
  
  // Initialize I2C communication
  bmeWire.begin(BME_SDA, BME_SCL);
  
  // Try to initialize BME280 sensor
  Serial.println("Initializing BME280...");
  
  // Try address 0x76
  if (bme.begin(0x76, &bmeWire)) {
    Serial.println("BME280 found at address 0x76");
  }
  // Try address 0x77
  else if (bme.begin(0x77, &bmeWire)) {
    Serial.println("BME280 found at address 0x77");
  }
  else {
    Serial.println("Could not find BME280 sensor!");
    Serial.println("Please check wiring and I2C address.");
    Serial.println("Address 0x76 or 0x77 should work.");
    while (1);  // Stop execution
  }
  
  // Configure sensor parameters
  bme.setSampling(
    Adafruit_BME280::MODE_NORMAL,     // Normal mode
    Adafruit_BME280::SAMPLING_X2,     // Temperature sampling
    Adafruit_BME280::SAMPLING_X16,    // Pressure sampling
    Adafruit_BME280::SAMPLING_X1,     // Humidity sampling
    Adafruit_BME280::FILTER_OFF,      // Filter
    Adafruit_BME280::STANDBY_MS_1000  // Standby time
  );
  
  Serial.println("BME280 initialized successfully!");
  Serial.println("Reading sensor data...");
  Serial.println();
}

void loop() {
  // Read sensor data
  float temperature = bme.readTemperature();    // Celsius
  float pressure = bme.readPressure() / 100.0F; // Convert Pa to hPa
  float humidity = bme.readHumidity();          // Percentage
  
  // Check if data is valid
  if (isnan(temperature) || isnan(pressure) || isnan(humidity)) {
    Serial.println("Error: Failed to read from sensor!");
    delay(2000);
    return;
  }
  
  // Calculate altitude (using sea level standard pressure 1013.25 hPa)
  float altitude = bme.readAltitude(1013.25);
  
  // Serial output format
  Serial.println("================================");
  Serial.print("Temperature: ");
  Serial.print(temperature);
  Serial.println(" °C");
  
  Serial.print("Pressure:    ");
  Serial.print(pressure);
  Serial.println(" hPa");
  
  Serial.print("Humidity:    ");
  Serial.print(humidity);
  Serial.println(" %");
  
  Serial.print("Altitude:    ");
  Serial.print(altitude);
  Serial.println(" m");
  
  Serial.println("================================");
  Serial.println();
  
  // Wait 2 seconds
  delay(2000);
}