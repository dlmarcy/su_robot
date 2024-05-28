/*
  cse400 test all hardware SU 8/21/2023
  Syracuse University Robotics Programming Course
*/

#include <VL53L0X.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>
#include <PID_v2.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <AccelStepper.h>

// GPIO pin definitions
int Mot_L_EN = 4;
int Mot_L_PH = 5;
int Mot_R_EN = 3;
int Mot_R_PH = 2;
int Encode_L1 = 17;
int Encode_L2 = 16;
int Encode_R1 = 0;
int Encode_R2 = 1;
int Step1_Or = 20;
int Step1_Ye = 6;
int Step1_Pi = 12;
int Step1_Bl = 11;
int Step2_Or = 7;
int Step2_Ye = 8;
int Step2_Pi = 9;
int Step2_Bl = 10;
int LED_GRN1 = 14; // SOC LEDs
int LED_RED1 = 15;
int LED_GRN2 = 22; // Current LEDs
int LED_RED2 = 23;
int Teensy_LED = 13;
int Push_But = 21;

// Power monitor variables
float battery_voltage = 0;
float battery_current_mA = 0;
float battery_power_mW = 0;

// BNO055 variables
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

// Motor variables
long encoder_l;
long encoder_r;
long previous_l = 0;
long previous_r = 0;
int test_Speed = 200;
int test_Duration = 1000;

// Arm variables
unsigned long check_time;
int test_speed = 100;
unsigned int test_duration = 3000;

// objects
Adafruit_INA219 power_monitor;
AccelStepper control_shoulder(AccelStepper::FULL4WIRE, Step1_Or, Step1_Pi, Step1_Ye, Step1_Bl);
AccelStepper control_elbow(AccelStepper::FULL4WIRE, Step2_Or, Step2_Pi, Step2_Ye, Step2_Bl);
Adafruit_BNO055 imu_sensor = Adafruit_BNO055(55);  
VL53L0X sensor;
Encoder encoder_left(Encode_L1, Encode_L2); 
Encoder encoder_right(Encode_R2, Encode_R1); 

void setup() {
  Wire.begin();
  
  // Set up LEDs and Pushbutton
  analogWrite(LED_RED1, 0); // red SOC LED
  analogWrite(LED_GRN1, 0); // green SOC LED
  analogWrite(LED_RED2, 0); // red current LED
  analogWrite(LED_GRN2, 0); // green current LED
  pinMode(Teensy_LED, OUTPUT); // Teensy LED
  digitalWrite(Teensy_LED, LOW); 
  pinMode(Push_But, INPUT_PULLDOWN); // Pushbutton
  
  // Set up power monitor
  power_monitor.begin();
  
  // Set up BNO055
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
  
  // Set up range sensor
  sensor.setTimeout(15);
  if (!sensor.init()) {
    Serial.println("Failed to detect and initialize sensor!");
    while (1) {}
  }
  sensor.setMeasurementTimingBudget(200000);
  sensor.startContinuous();

  // Set up motors
  analogWrite(Mot_L_EN, 0); // left speed pin
  pinMode(Mot_L_PH, OUTPUT);  // left direction pin
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_R_EN, 0); // right speed pin
  pinMode(Mot_R_PH, OUTPUT);  //right direction pin
  digitalWrite(Mot_R_PH, LOW);  // right forward
  encoder_left.write(0);
  encoder_right.write(0);

  // Set up Arm
  control_shoulder.disableOutputs();
  control_elbow.disableOutputs();
  control_shoulder.setMaxSpeed(400);
  control_shoulder.setAcceleration(10);
  control_elbow.setMaxSpeed(400);
  control_elbow.setAcceleration(10);
}

void loop() {
  // Test the LEDs
  // Test the SOC LED
  Serial.println("Watch the LEDs");
  Serial.println();
  for(int i=0; i<=255; i++) {
    analogWrite(LED_RED1, i); // red SOC LED
    analogWrite(LED_GRN1, 255-i); // green SOC LED
    delay(10);
  }
  analogWrite(LED_RED1, 0); // red SOC LED
  analogWrite(LED_GRN1, 0); // green SOC LED
  
  // Test the current LED
  for(int i=0; i<=255; i++) {
    analogWrite(LED_GRN2, i); // green current LED
    delay(5);
  }
  for(int i=0; i<=255; i++) {
    analogWrite(LED_GRN2, 255-i); // green current LED
    delay(5);
  }
  for(int i=0; i<=255; i++) {
    analogWrite(LED_RED2, i); // red current LED
    delay(5);
  }
  for(int i=0; i<=255; i++) {
    analogWrite(LED_RED2, 255-i); // red current LED
    delay(5);
  }
  analogWrite(LED_RED2, 0); // red current LED
  analogWrite(LED_GRN2, 0); // green current LED
 
  // Test the Teensy_LED
  digitalWrite(Teensy_LED, HIGH); 
  delay(1000);
  digitalWrite(Teensy_LED, LOW); 

  // Test the pushbutton
  Serial.println("Press the pushbutton");
  for(int i=0; i<=20; i++) {
    int pb = digitalRead(Push_But);
    Serial.print(pb);
    delay(250);
  }
  Serial.println();
  Serial.println();
  delay(1000);

  // Test the power monitor
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
	
  // Test the BNO055
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
  
  // Test the range sensor
  check_time = millis()
  Serial.print("Range sensor reading ");
  for (int i=0; i<10; i++) {
    if ((millis()-check_time) > 200) {
      check_time += 200;
      Serial.print(sensor.readRangeContinuousMillimeters());
      if (sensor.timeoutOccurred()) { Serial.print(" TIMEOUT"); }
      Serial.println();
    }
  }

  // Test the motors
  // Move the left wheel forward for 2 seconds
  Serial.println("Left wheel forward encoder values:");
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_L_EN, test_Speed); // left speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_l = encoder_left.read();
    Serial.println(encoder_l);
  }
  analogWrite(Mot_L_EN, 0); // left speed pin
  Serial.println();
  delay(1000);
  // Move the left wheel backward for 2 seconds
  Serial.println("Left wheel backward encoder values:");
  digitalWrite(Mot_L_PH, LOW);  // left backward
  analogWrite(Mot_L_EN, test_Speed); // left speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_l = encoder_left.read();
    Serial.println(encoder_l);
  }
  analogWrite(Mot_L_EN, 0); // left speed pin
  Serial.println();
  delay(1000);
  // Move the right wheel forward for 2 seconds
  Serial.println("Right wheel forward encoder values:");
  digitalWrite(Mot_R_PH, LOW);  // right forward
  analogWrite(Mot_R_EN, test_Speed); // right speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_r = encoder_right.read();
    Serial.println(encoder_r);
  }
  analogWrite(Mot_R_EN, 0); // right speed pin
  Serial.println();
  delay(1000);
  // Move the right wheel backward for 2 seconds
  Serial.println("Right wheel backward encoder values:");
  digitalWrite(Mot_R_PH, HIGH);  // right backward
  analogWrite(Mot_R_EN, test_Speed); // right speed pin
  for(int i=0; i<10; i++) {
    delay(test_Duration);
    encoder_r = encoder_right.read();
    Serial.println(encoder_r);
  }
  analogWrite(Mot_R_EN, 0); // right speed pin
  Serial.println();
  delay(1000);

  // Test the Arm
  check_time = millis()
  Serial.println("Moving shoulder forward then backward:");
  control_shoulder.setSpeed(test_speed);
  check_time = millis() + test_duration;
  while(millis() <= check_time) {
    control_shoulder.runSpeed();
  }
  control_shoulder.setSpeed(-test_speed);
  check_time = millis() + test_duration + test_duration;
  while(millis() <= check_time) {
    control_shoulder.runSpeed();
  }
  control_shoulder.setSpeed(test_speed);
  check_time = millis() + test_duration;
  while(millis() <= check_time) {
    control_shoulder.runSpeed();
  } 
  Serial.println("Moving elbow forward then backward:");
  control_elbow.setSpeed(test_speed);
  check_time = millis() + test_duration;
  while(millis() <= check_time) {
    control_elbow.runSpeed();
  }
  control_elbow.setSpeed(-test_speed);
  check_time = millis() + test_duration + test_duration;
  while(millis() <= check_time) {
    control_elbow.runSpeed();
  }
  control_elbow.setSpeed(test_speed);
  check_time = millis() + test_duration;
  while(millis() <= check_time) {
    control_elbow.runSpeed();
  }
  Serial.println();
}
