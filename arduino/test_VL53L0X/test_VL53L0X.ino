/*
  test_VL53L0X SU 7/6/2023
*/

#include <VL53L0X.h>

VL53L0X sensor;

unsigned long check_time = millis();

void setup() {
  Serial.begin(9600);
  Wire.begin();

  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  //sensor.setMeasurementTimingBudget(500000);

  // Start continuous back-to-back mode (take readings as
  // fast as possible).  To use continuous timed mode
  // instead, provide a desired inter-measurement period in
  // ms (e.g. sensor.startContinuous(100)).
  sensor.startContinuous();
}

void loop() {
  // When used with ROS the sensor will be sampled at 5Hz
  if ((millis()-check_time) > 500) {
    check_time += 500;
    Serial.print(sensor.readRangeContinuousMillimeters());
    if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
    Serial.println();
  }
}
