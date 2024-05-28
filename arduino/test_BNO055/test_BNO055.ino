/*
  test_BNO055 SU 6/4/2023
*/

#include <Adafruit_BNO055.h>
#include <EEPROM.h>

int eeAddress = 0;
long bnoID;
bool foundCalib = false;
adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;
uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
imu::Quaternion quat;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> magnet;

Adafruit_BNO055 imu_sensor = Adafruit_BNO055(55);  

void setup() {
  Wire.begin();
  imu_sensor.begin(); 
  EEPROM.get(eeAddress, bnoID); // Check for calibration data in Teensy's EEPROM
  imu_sensor.getSensor(&sensor);  // Get the BNO sensor ID and compare to that stored in the EEPROM (check if same sensor)
  if (bnoID == sensor.sensor_id) {
    // Serial.println("\nFound Calibration for this sensor in EEPROM.");
    eeAddress += sizeof(long);
    EEPROM.get(eeAddress, calibrationData);
    // Serial.println("\n\nRestoring Calibration data to the BNO055...");
    imu_sensor.setSensorOffsets(calibrationData);
    // Serial.println("\n\nCalibration data loaded into BNO055");
    foundCalib = true;
  }
  delay(100);  // Example sketch used a delay here
  imu_sensor.setExtCrystalUse(true);
}

void loop() {
  // Test the IMU
  quat = imu_sensor.getQuat();
  gyro = imu_sensor.getVector(Adafruit_BNO055::VECTOR_GYROSCOPE);
  accel = imu_sensor.getVector(Adafruit_BNO055::VECTOR_ACCELEROMETER);
  magnet = imu_sensor.getVector(Adafruit_BNO055::VECTOR_MAGNETOMETER);
  imu_sensor.getCalibration(&system_cal, &gyro_cal, &accel_cal, &mag_cal);
  Serial.println("Calibration:");
  Serial.print("System = ");
  Serial.print(system_cal);
  Serial.print(" Gyro = ");
  Serial.print(gyro_cal);
  Serial.print(" Accel = ");
  Serial.print(accel_cal);
  Serial.print(" Mag = ");
  Serial.println(mag_cal);
  Serial.println("Quaternion:");
  Serial.print("w = ");
  Serial.print(quat.w());
  Serial.print(" x = ");
  Serial.print(quat.x());
  Serial.print(" y = ");
  Serial.print(quat.y());
  Serial.print(" z = ");
  Serial.println(quat.z());
  Serial.println("Gyroscope [radian/second]:");
  Serial.print("x = ");
  Serial.print(gyro.x());
  Serial.print(" y = ");
  Serial.print(gyro.y());
  Serial.print(" z = ");
  Serial.println(gyro.z());
  Serial.println("Acceleration [m/s^2]:");
  Serial.print("x = ");
  Serial.print(accel.x());
  Serial.print(" y = ");
  Serial.print(accel.y());
  Serial.print(" z = ");
  Serial.println(accel.z());
  Serial.println("Magnetic Field [uT]:");
  Serial.print("x = ");
  Serial.print(magnet.x());
  Serial.print(" y = ");
  Serial.print(magnet.y());
  Serial.print(" z = ");
  Serial.println(magnet.z());
  Serial.println();
  delay(2000);
}
