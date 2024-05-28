/*
  test_Arm SU 7/6/2023
*/

#include <AccelStepper.h>

int Step1_Or = 20;
int Step1_Ye = 6;
int Step1_Pi = 12;
int Step1_Bl = 11;
int Step2_Or = 7;
int Step2_Ye = 8;
int Step2_Pi = 9;
int Step2_Bl = 10;

unsigned long check_time;
int test_speed = 100;
unsigned int test_duration = 3000;

AccelStepper control_shoulder(AccelStepper::FULL4WIRE, Step1_Or, Step1_Pi, Step1_Ye, Step1_Bl);
AccelStepper control_elbow(AccelStepper::FULL4WIRE, Step2_Or, Step2_Pi, Step2_Ye, Step2_Bl);

void setup() {
  control_shoulder.disableOutputs();
  control_elbow.disableOutputs();
  control_shoulder.setMaxSpeed(400);
  control_shoulder.setAcceleration(10);
  control_elbow.setMaxSpeed(400);
  control_elbow.setAcceleration(10);
}

void loop() {
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
