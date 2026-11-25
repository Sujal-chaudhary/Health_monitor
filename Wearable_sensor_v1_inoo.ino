uint16_t latestIRValue = 0;

#include <Wire.h>

// Pulse sensor
#include "MAX30100_PulseOximeter.h"

// Temperature sensor
#include <OneWire.h>
#include <DallasTemperature.h>

// Motion sensor
#include <AlashMPU.h>

// ---------------- PIN CONFIGURATION ----------------

#define SDA_PIN 21
#define SCL_PIN 22
#define TEMP_PIN 4

// ---------------- SENSOR OBJECTS ----------------

PulseOximeter pox;

OneWire oneWire(TEMP_PIN);
DallasTemperature tempSensor(&oneWire);

AlashMPU mpu;

// ---------------- TIMING ----------------

unsigned long lastReport = 0;
unsigned long lastTempRequest = 0;

unsigned long lastBeatDetected = 0;

const unsigned long REPORT_INTERVAL = 3000;
const unsigned long TEMP_INTERVAL = 1000;

//Seperate heart rate sampling timer
const unsigned long HR_SAMPLE_INTERVAL = 250;  // Sample HR every 250 ms
const unsigned long HEARTBEAT_TIMEOUT = 5000;   // Mark HR invalid after 5 sec

unsigned long lastHrSample = 0;


float latestRawHeartRate = NAN;
float latestFilteredHeartRate = NAN;


// Heart-rate smoothing
const int HR_WINDOW = 5;

float hrSamples[HR_WINDOW];
int hrIndex = 0;
int hrCount = 0;

// ---------------- TEMPERATURE STATE ----------------

bool temperaturePending = false;
float temperatureC = NAN;

// ---------------- HEARTBEAT FILTER ----------------

float getFilteredHeartRate(float newReading) {

  // Reject invalid or non-finite readings
  if (!isfinite(newReading) || newReading < 40.0 || newReading > 180.0) {
    return NAN;
  }

  hrSamples[hrIndex] = newReading;

  hrIndex = (hrIndex + 1) % HR_WINDOW;

  if (hrCount < HR_WINDOW) {
    hrCount++;
  }

  float sorted[HR_WINDOW];

  for (int i = 0; i < hrCount; i++) {
    sorted[i] = hrSamples[i];
  }

  // Sort valid samples
  for (int i = 0; i < hrCount - 1; i++) {
    for (int j = i + 1; j < hrCount; j++) {
      if (sorted[i] > sorted[j]) {
        float temp = sorted[i];
        sorted[i] = sorted[j];
        sorted[j] = temp;
      }
    }
  }

  // Median of available samples
  return sorted[hrCount / 2];
}

void onBeatDetected() {
    lastBeatDetected = millis();
}

// ---------------- SETUP ----------------

void setup() {
  Serial.begin(115200);
  delay(1500);

  Serial.println();
  Serial.println("================================");
  Serial.println("     WEARABLE SENSOR SYSTEM");
  Serial.println("================================");

  // Start I2C bus
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  // Initialize DS18B20
  Serial.println("Initializing DS18B20...");

  tempSensor.begin();

  tempSensor.setResolution(9);
  tempSensor.setWaitForConversion(false);

  tempSensor.requestTemperatures();
  temperaturePending = true;
  lastTempRequest = millis();

  Serial.println("DS18B20 initialized.");

  // Initialize motion sensor
  Serial.println("Initializing motion sensor...");

  if (!mpu.begin()) {
    Serial.println("Motion sensor initialization FAILED");
    while (true) {
      delay(1000);
    }
  }

  Serial.println("Motion sensor initialized.");

  // Initialize MAX30100
  Serial.println("Initializing MAX30100...");

  if (!pox.begin()) {
    Serial.println("MAX30100 initialization FAILED");
    while (true) {
      delay(1000);
    }
  }


   //Whenever you detect a heartbeat, call my function so I can record its time.
   pox.setOnBeatDetectedCallback(onBeatDetected);

  Serial.println("MAX30100 initialized.");

  Serial.println("--------------------------------");
  Serial.println("All sensors initialized!");
  Serial.println("--------------------------------");
}

// ---------------- MAIN LOOP ----------------

void loop() {

  // Keep pulse sensor processing continuously
  pox.update();

  latestIRValue = pox.getLastIRValue();

  unsigned long currentTime = millis();

  if (currentTime - lastHrSample >= HR_SAMPLE_INTERVAL) {

    lastHrSample = currentTime;

    float rawHR = pox.getHeartRate();

    // Accept only plausible heart-rate estimates
    if (isfinite(rawHR) && rawHR >= 40.0 && rawHR <= 180.0) {

      float filteredHR = getFilteredHeartRate(rawHR);

     if (isfinite(filteredHR)) {
         latestRawHeartRate = rawHR;
        latestFilteredHeartRate = filteredHR;
        }
    }
  }


  // Request temperature periodically
  if (!temperaturePending && currentTime - lastTempRequest >= TEMP_INTERVAL) {

    tempSensor.requestTemperatures();

    temperaturePending = true;
    lastTempRequest = currentTime;
  }

  // Read temperature after conversion time
  if (temperaturePending && currentTime - lastTempRequest >= 120) {

    temperatureC = tempSensor.getTempCByIndex(0);
    temperaturePending = false;
  }

  // Print sensor data every 3 second
  if (currentTime - lastReport >= REPORT_INTERVAL) {

    // Pulse sensor
   bool heartRateFresh =
    lastBeatDetected != 0 &&
    currentTime - lastBeatDetected <= HEARTBEAT_TIMEOUT;
    float spo2 = pox.getSpO2();

    // Motion sensor
    float accelX = mpu.accelX();
    float accelY = mpu.accelY();
    float accelZ = mpu.accelZ();

    float gyroX = mpu.gyroX();
    float gyroY = mpu.gyroY();
    float gyroZ = mpu.gyroZ();


    // Output one JSON record per reporting interval

    Serial.print("{");

    Serial.print("\"timeMs\":");
    Serial.print(currentTime);

    Serial.print(",\"heartRate\":");

    if (heartRateFresh)
      Serial.print(latestFilteredHeartRate, 2);
    else
      Serial.print("null");

    Serial.print(",\"rawHeartRate\":");

    if (heartRateFresh)
      Serial.print(latestRawHeartRate, 2);
    else
      Serial.print("null");


      Serial.print(",\"spo2\":");

       if (heartRateFresh &&
            isfinite(spo2) &&
            spo2 > 0 &&
            spo2 <= 100) {
          Serial.print(spo2, 2);
        } else {
             Serial.print("null");
        }

    Serial.print(",\"temperatureC\":");
    if (isfinite(temperatureC) && temperatureC != DEVICE_DISCONNECTED_C) {
      Serial.print(temperatureC, 2);
    } else {
      Serial.print("null");
    }

    Serial.print(",\"irRaw\":");
    Serial.print(latestIRValue);

    Serial.print(",\"acceleration\":{");
    Serial.print("\"x\":");
    Serial.print(accelX, 2);
    Serial.print(",\"y\":");
    Serial.print(accelY, 2);
    Serial.print(",\"z\":");
    Serial.print(accelZ, 2);
    Serial.print("}");

    Serial.print(",\"gyroscope\":{");
    Serial.print("\"x\":");
    Serial.print(gyroX, 2);
    Serial.print(",\"y\":");
    Serial.print(gyroY, 2);
    Serial.print(",\"z\":");
    Serial.print(gyroZ, 2);
    Serial.print("}");

    Serial.println("}");

    lastReport = currentTime;
  }
}
