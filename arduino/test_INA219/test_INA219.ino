/*
  test_INA219 SU 6/4/2023
*/

#include <Wire.h>
#include <Adafruit_INA219.h>

float battery_voltage = 0;
float battery_current_mA = 0;
float battery_power_mW = 0;

Adafruit_INA219 power_monitor;

void setup() {
  Wire.begin();
  power_monitor.begin();
}

void loop() {
  battery_voltage = power_monitor.getBusVoltage_V() + (power_monitor.getShuntVoltage_mV() / 1000);
  battery_current_mA = power_monitor.getCurrent_mA();
  battery_power_mW = battery_voltage * battery_current_mA;
  Serial.print("battery voltage = ");
  Serial.println(battery_voltage);
  Serial.print("battery current mA = ");
  Serial.println(battery_current_mA);
  Serial.print("battery power mW = ");
  Serial.println(battery_power_mW);
  Serial.println();
  delay(2000);
}
