  // Initialize SD Card with diagnostics
  Serial.print(F("Init SD (CS Pin "));
  Serial.print(SD_CS);
  Serial.print(F(")... "));
  
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  
  // Try multiple initialization methods
  bool sdInit = false;
  
  // Method 1: Standard init
  if (SD.begin(SD_CS)) {
    sdInit = true;
  } 
  // Method 2: Try with SPI speed configuration
  else {
    delay(100);
    pinMode(10, OUTPUT); // Disable other SPI devices if any
    digitalWrite(10, HIGH);
    if (SD.begin(SD_CS, 11, 12, 13)) { // MOSI, MISO, SCK
      sdInit = true;
    }
  }
  
  if (!sdInit) {
    Serial.println(F("FAILED"));
    Serial.println(F("Troubleshooting:"));
    Serial.println(F("- SD card inserted correctly?"));
    Serial.println(F("- Card formatted as FAT16/FAT32?"));
    Serial.println(F("- Try different SD card (< 32GB)"));
    Serial.println(F("- Wiring: CS->8, MOSI->11, MISO->12, SCK->13"));
    Serial.println(F("- Check 5V and GND connections"));
    Serial.println(F("- Add 10K pull-up on MISO if needed"));
  } else {
    Serial.println(F("OK"));
    sdReady = true;
    
    // Try to create/open file
    if (!SD.exists("FL15.CSV")) {
      File f = SD.open("FL15.CSV", FILE_WRITE);
      if (f) {
        f.println(F("Timestamp,Temperature(C),Humidity(%),Pressure(hPa),Altitude(m)"));
        f.close();
        Serial.println(F("Created FL15.CSV"));
      } else {
        Serial.println(F("Cannot create file - card may be write-protected"));
        sdReady = false;
      }
    } else {
      Serial.println(F("FL15.CSV exists - appending data"));
      // Test write access
      File f = SD.open("FL15.CSV", FILE_WRITE);
      if (f) {
        f.close();
        Serial.println(F("Write access: OK"));
      } else {
        Serial.println(F("Write access: FAILED - check write protection"));
        sdReady = false;
      }
    }
  /*
 * Environmental Data Logger
 * Sensors: DHT12 (Temperature & Humidity), BMP280 (Pressure & Altitude)
 * Storage: SD Card Module
 * Features: Live Serial Monitor Display + SD Card Logging
 * 
 * Board: Arduino Uno/Nano
 * 
 * Required Libraries:
 * - DHT sensor library by Adafruit
 * - Adafruit BMP280 Library
 * - SD (built-in)
 * - SPI (built-in)
 */

#include <Wire.h>
#include <DHT.h>
#include <Adafruit_BMP280.h>
#include <SD.h>
#include <SPI.h>

// Pin Definitions
#define DHTPIN 2              // DHT12 data pin
#define SD_CS 8               // SD Card Chip Select

// Configuration
#define LOG_INTERVAL 10000    // Log every 10 seconds
#define DISPLAY_INTERVAL 2000 // Display every 2 seconds

// Initialize sensors
DHT dht(DHTPIN, DHT12);
Adafruit_BMP280 bmp;

// Variables
float temperature = 0;
float humidity = 0;
float pressure = 0;
float altitude = 0;
unsigned long lastLog = 0;
unsigned long lastDisp = 0;
uint16_t dataCount = 0;
bool sdReady = false;

void setup() {
  Serial.begin(9600);
  delay(2000);
  
  Serial.println(F("\n================================"));
  Serial.println(F("  Environmental Data Logger"));
  Serial.println(F("  DHT12 + BMP280 + SD Card"));
  Serial.println(F("================================\n"));
  
  // Initialize SD Card with diagnostics
  Serial.print(F("Init SD (CS Pin "));
  Serial.print(SD_CS);
  Serial.print(F(")... "));
  
  pinMode(SD_CS, OUTPUT);
  digitalWrite(SD_CS, HIGH);
  
  if (!SD.begin(SD_CS)) {
    Serial.println(F("FAILED"));
    Serial.println(F("Check:"));
    Serial.println(F("- SD card inserted?"));
    Serial.println(F("- Wiring: CS->8, MOSI->11, MISO->12, SCK->13"));
    Serial.println(F("- 5V and GND connected?"));
  } else {
    Serial.println(F("OK"));
    sdReady = true;
    
    if (!SD.exists("FL15.CSV")) {
      File f = SD.open("FL15.CSV", FILE_WRITE);
      if (f) {
        f.println(F("Timestamp,Temperature(C),Humidity(%),Pressure(hPa),Altitude(m)"));
        f.close();
        Serial.println(F("Created FL15.CSV"));
      } else {
        Serial.println(F("Cannot create file"));
        sdReady = false;
      }
    } else {
      Serial.println(F("FL15.CSV exists"));
    }
  }
  
  // Initialize DHT12 with diagnostics
  Serial.print(F("Init DHT12 (Pin "));
  Serial.print(DHTPIN);
  Serial.print(F(")... "));
  dht.begin();
  delay(2000); // DHT needs time to stabilize
  
  // Test read
  float testH = dht.readHumidity();
  float testT = dht.readTemperature();
  
  if (isnan(testH) || isnan(testT)) {
    Serial.println(F("FAILED"));
    Serial.println(F("Check:"));
    Serial.println(F("- DHT12 wiring: VCC->5V, GND->GND, DATA->Pin2"));
    Serial.println(F("- Is it DHT12 (not DHT11 or DHT22)?"));
    Serial.println(F("- Try adding 10K pull-up resistor"));
  } else {
    Serial.println(F("OK"));
    Serial.print(F("  Test: "));
    Serial.print(testT);
    Serial.print(F("C, "));
    Serial.print(testH);
    Serial.println(F("%"));
  }
  
  // Initialize BMP280
  Serial.print(F("Init BMP280 (I2C)... "));
  if (!bmp.begin(0x76)) {
    Serial.print(F("0x76 failed, trying 0x77... "));
    if (!bmp.begin(0x77)) {
      Serial.println(F("FAILED"));
      Serial.println(F("Check I2C wiring: SDA->A4, SCL->A5"));
      while(1);
    }
  }
  Serial.println(F("OK"));
  
  bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                  Adafruit_BMP280::SAMPLING_X2,
                  Adafruit_BMP280::SAMPLING_X16,
                  Adafruit_BMP280::FILTER_X16,
                  Adafruit_BMP280::STANDBY_MS_500);
  
  Serial.println(F("\nSystem Ready!\n"));
  delay(1000);
}

void loop() {
  unsigned long now = millis();
  
  // Display live data
  if (now - lastDisp >= DISPLAY_INTERVAL) {
    readSensors();
    displayData();
    lastDisp = now;
  }
  
  // Log to SD
  if (now - lastLog >= LOG_INTERVAL) {
    if (sdReady) {
      logData();
    }
    lastLog = now;
  }
}

void readSensors() {
  humidity = dht.readHumidity();
  temperature = dht.readTemperature();
  pressure = bmp.readPressure() / 100.0;
  altitude = bmp.readAltitude(1013.25);
  
  if (isnan(humidity) || isnan(temperature)) {
    humidity = 0;
    temperature = 0;
  }
}

void displayData() {
  Serial.println(F("\n========== LIVE DATA =========="));
  
  Serial.print(F("Temperature:  "));
  Serial.print(temperature, 1);
  Serial.println(F(" C"));
  
  Serial.print(F("Humidity:     "));
  Serial.print(humidity, 1);
  Serial.println(F(" %"));
  
  Serial.print(F("Pressure:     "));
  Serial.print(pressure, 2);
  Serial.println(F(" hPa"));
  
  Serial.print(F("Altitude:     "));
  Serial.print(altitude, 1);
  Serial.println(F(" m"));
  
  Serial.println(F("-------------------------------"));
  Serial.print(F("SD Card:      "));
  Serial.println(sdReady ? F("OK") : F("ERROR"));
  
  Serial.print(F("Records:      "));
  Serial.println(dataCount);
  
  Serial.print(F("Uptime:       "));
  unsigned long sec = millis() / 1000;
  Serial.print(sec / 60);
  Serial.print(F(" min "));
  Serial.print(sec % 60);
  Serial.println(F(" sec"));
  
  Serial.println(F("===============================\n"));
}

void logData() {
  File f = SD.open("FL15.CSV", FILE_WRITE);
  
  if (f) {
    unsigned long timestamp = millis() / 1000;
    
    f.print(timestamp);
    f.print(',');
    f.print(temperature, 2);
    f.print(',');
    f.print(humidity, 2);
    f.print(',');
    f.print(pressure, 2);
    f.print(',');
    f.println(altitude, 2);
    
    f.close();
    dataCount++;
    
    Serial.print(F("[LOG] Record #"));
    Serial.print(dataCount);
    Serial.println(F(" saved"));
  } else {
    Serial.println(F("[ERROR] Cannot write to SD"));
    sdReady = false;
  }
}