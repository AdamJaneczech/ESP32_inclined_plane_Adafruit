#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VL53L0X.h>

#define SENSOR1_ADDRESS 0x29
#define SENSOR2_ADDRESS 0x30

Adafruit_VL53L0X sensor1;
Adafruit_VL53L0X sensor2;

void setup() {
  Serial.begin(9600);
  Wire.begin();

  // Initialize sensors with different I2C addresses
  sensor1.begin(SENSOR1_ADDRESS);
  sensor2.begin(SENSOR2_ADDRESS);

  // Optional: Set measurement timing budget (in microseconds)
  //sensor1.setMeasurementTimingBudget(20000);
  //sensor2.setMeasurementTimingBudget(20000);
}

void loop() {
  // Read distance from sensor 1
  VL53L0X_RangingMeasurementData_t measurement1;
  sensor1.rangingTest(&measurement1, false);
  Serial.print("Sensor 1: ");
  Serial.print(measurement1.RangeMilliMeter);
  Serial.println(" mm");

  // Read distance from sensor 2
  VL53L0X_RangingMeasurementData_t measurement2;
  sensor2.rangingTest(&measurement2, false);
  Serial.print("Sensor 2: ");
  Serial.print(measurement2.RangeMilliMeter);
  Serial.println(" mm");

  delay(1000);
}
