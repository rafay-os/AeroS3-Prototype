#include <Wire.h>
#include <SPI.h>
#include <SD.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#include <TinyGPSPlus.h>
#include <HardwareSerial.h>

// Pin Definitions according to your Schematic
const int SD_CS_PIN = 4;    // D3 / GPIO4 -> SD_CS

// Sensors & GPS Objects
Adafruit_MPU6050 mpu;
Adafruit_BMP280 bmp; // I2C
TinyGPSPlus gps;
HardwareSerial GPS_Serial(1); // Using UART1 on ESP32-S3

// Timing variables for 10Hz logging (100ms interval)
unsigned long lastLogTime = 0;
const unsigned long LOG_INTERVAL = 100; 

File dataFile;
bool sdInitialized = false;

void setup() {
  Serial.begin(115200);
  while(!Serial && millis() < 3000); // Wait for serial monitor if connected

  // 1. Initialize I2C Bus (D4 / GPIO5 = SDA, D5 / GPIO6 = SCL)
  Wire.begin(5, 6);

  // 2. Initialize GPS UART (D6 / GPIO43 = TX, D7 / GPIO44 = RX -> connected to GPS RX/TX)
  GPS_Serial.begin(9600, SERIAL_8N1, 44, 43); // RX pin 44, TX pin 43

  Serial.println(F("MINI-FC V1.0 Flight Computer Initializing..."));

  // 3. Initialize MPU6050
  if (!mpu.begin()) {
    Serial.println(F("WARNING: MPU6050 not found! Check wiring/I2C address."));
  } else {
    Serial.println(F("MPU6050 Initialized."));
    mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
    mpu.setGyroRange(MPU6050_RANGE_500_DEG);
    mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  }

  // 4. Initialize BMP280 (Address 0x76)
  if (!bmp.begin(0x76)) {
    Serial.println(F("WARNING: BMP280 not found! Check I2C address."));
  } else {
    Serial.println(F("BMP280 Initialized."));
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,  // Temp oversampling
                    Adafruit_BMP280::SAMPLING_X16, // Pressure oversampling
                    Adafruit_BMP280::FILTER_X16,   // Filtering
                    Adafruit_BMP280::STANDBY_MS_50);
  }

  // 5. Initialize MicroSD Card
  pinMode(SD_CS_PIN, OUTPUT);
  if (!SD.begin(SD_CS_PIN)) {
    Serial.println(F("ERROR: MicroSD card initialization failed!"));
    sdInitialized = false;
  } else {
    Serial.println(F("MicroSD Card Initialized."));
    sdInitialized = true;

    // Create CSV Header if file doesn't exist
    dataFile = SD.open("/flight_log.csv", FILE_WRITE);
    if (dataFile) {
      if (dataFile.size() == 0) {
        dataFile.println("Millis,AccelX,AccelY,AccelZ,GyroX,GyroY,GyroZ,Pressure,Temp,EstAltitude,GPS_Lat,GPS_Lng,GPS_Alt,Satellites");
      }
      dataFile.close();
    } else {
      Serial.println(F("ERROR: Could not open flight_log.csv for writing"));
    }
  }

  Serial.println(F("System Setup Complete. Starting 10Hz Flight Logging..."));
}

void loop() {
  // Continuously feed GPS data parser from hardware serial
  while (GPS_Serial.available() > 0) {
    gps.encode(GPS_Serial.read());
  }

  // Handle 10Hz logging interval using millis() (non-blocking)
  unsigned long currentMillis = millis();
  if (currentMillis - lastLogTime >= LOG_INTERVAL) {
    lastLogTime = currentMillis;

    // Read Sensors
    sensors_event_t a, g, tempMpu;
    mpu.getEvent(&a, &g, &tempMpu);

    float pressure = bmp.readPressure(); // Pascals
    float temperature = bmp.readTemperature(); // Celsius
    float estimatedAltitude = bmp.readAltitude(1013.25); // Standard sea-level pressure hPa

    // Read GPS Data
    double lat = gps.location.isValid() ? gps.location.lat() : 0.0;
    double lng = gps.location.isValid() ? gps.location.lng() : 0.0;
    double gpsAlt = gps.altitude.isValid() ? gps.altitude.meters() : 0.0;
    int sats = gps.satellites.isValid() ? gps.satellites.value() : 0;

    // Format Data String (CSV)
    String logData = String(currentMillis) + "," +
                     String(a.acceleration.x) + "," +
                     String(a.acceleration.y) + "," +
                     String(a.acceleration.z) + "," +
                     String(g.gyro.x) + "," +
                     String(g.gyro.y) + "," +
                     String(g.gyro.z) + "," +
                     String(pressure) + "," +
                     String(temperature) + "," +
                     String(estimatedAltitude) + "," +
                     String(lat, 6) + "," +
                     String(lng, 6) + "," +
                     String(gpsAlt) + "," +
                     String(sats);

    // Print to Serial Monitor for debugging
    Serial.println(logData);

    // Save to MicroSD Card safely
    if (sdInitialized) {
      dataFile = SD.open("/flight_log.csv", FILE_WRITE);
      if (dataFile) {
        dataFile.println(logData);
        dataFile.close();
      } else {
        Serial.println(F("LOGGING ERROR: Failed to open file on SD"));
      }
    }
  }
}
