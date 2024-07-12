/*
  cse400_arduino_F24 SU 7/12/2024
  Syracuse University Robotics Programming Course
*/

#include "MicroROSArduino.h"
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

const int LEFT = 1;
const int RIGHT = 2;

// create a micro ros object
MicroROSArduino micro_ros;

// define the encoder variables and objects
uint8_t joint_state_id;
long encoder_l = 0;
long encoder_r = 0;
long previous_l = 0;
long previous_r = 0;
Encoder encoder_left(Encode_L1, Encode_L2); 
Encoder encoder_right(Encode_R2, Encode_R1); 

// define controller variables and objects
// default sample time for the PID controller is 100ms
// can be changed with object.SetSampleTime(int NewSampleTime);
// change setpoint with object.Setpoint(double v);
double input_l = 0.0;
double output_l = 0.0;
double input_r = 0.0;
double output_r = 0.0;
double Kp = 5.0, Ki = 100.0, Kd = 0.0;
PID_v2 control_left(Kp, Ki, Kd, PID::Direct);
PID_v2 control_right(Kp, Ki, Kd, PID::Direct);
AccelStepper control_shoulder(AccelStepper::FULL4WIRE, Step2_Or, Step2_Pi, Step2_Ye, Step2_Bl);
AccelStepper control_elbow(AccelStepper::FULL4WIRE, Step1_Or, Step1_Pi, Step1_Ye, Step1_Bl);

// define scaling variables
// The documentation says the motor encoders count 48 edges per revolution
// The gear ratio is 99:1, 99*48 = 4752
// The wheels are 80 mm diameter
const float scale_counts_pos = 99*48/(2*3.14159); // encoder counts per radian
const float scale_radians_pos = 1.0/scale_counts_pos;  // radians per encoder count
// The loop for measuring velocity counts for 1/10 second
const float scale_counts_vel = scale_counts_pos*0.1;
const float scale_radians_vel = 1.0/scale_counts_vel;
const float scale_steps_arm = 1019.0/3.14159; // steps per radian
const float scale_radians_arm = 1.0/scale_steps_arm; // radians per step

// define battery variables and objects
uint8_t battery_id;
Adafruit_INA219 power_monitor;

// define range variables and objects
uint8_t range_id;
VL53L0X range_sensor;

// define imu variables and objects
uint8_t imu_id;
int eeAddress = 0;
long bnoID;
adafruit_bno055_offsets_t calibrationData;
sensor_t sensor;
uint8_t system_cal, gyro_cal, accel_cal, mag_cal = 0;
imu::Quaternion quat;
imu::Vector<3> gyro;
imu::Vector<3> accel;
imu::Vector<3> magnet;
Adafruit_BNO055 imu_sensor = Adafruit_BNO055(55);  

void battery_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.battery_msg[battery_id].voltage = power_monitor.getBusVoltage_V() + (power_monitor.getShuntVoltage_mV()/1000.0);
  micro_ros.battery_msg[battery_id].current = power_monitor.getCurrent_mA()/-1000.0;
  micro_ros.publishBattery(battery_id);
}

void range_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.range_msg.range = range_sensor.readRangeContinuousMillimeters()*0.001;
  if (micro_ros.range_msg.range > 1.0) { range_msg.range = 1.0; }
  micro_ros.publishRange(range_id);
}

void imu_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.imu_msg.orientation.w = 0.707;
  micro_ros.imu_msg.orientation.z = 0.707;
  micro_ros.publishImu();
}

void joint_state_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  previous_l = encoder_l;
  encoder_l = encoder_left.read();
  previous_r = encoder_r;
  encoder_r = encoder_right.read();
  input_l = scale_radians_vel*(encoder_l - previous_l); // radians per second
  input_r = scale_radians_vel*(encoder_r - previous_r); // radians per second
  micro_ros.joint_state_msg.position.data[0] = scale_radians_pos*encoder_l;
  micro_ros.joint_state_msg.velocity.data[0] = input_l;
  micro_ros.joint_state_msg.position.data[1] = scale_radians_pos*encoder_r;
  micro_ros.joint_state_msg.velocity.data[1] = input_r;
  micro_ros.publishJointState(joint_state_id);
}

void commander_cb(const void * msgin)
{  
  const sensor_msgs__msg__JointState * cmd_msg = (const sensor_msgs__msg__JointState *)msgin;
  control_left.Setpoint(cmd_msg->velocity.data[0]);
  control_right.Setpoint(cmd_msg->velocity.data[1]);
  shoulder_setpoint = cmd_msg->velocity.data[2];
  elbow_setpoint = cmd_msg->velocity.data[3];
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
}

void set_PWM(int motor, int value) {
  if (motor == LEFT) {
    if (value < 0) {
      digitalWrite(Mot_L_PH, HIGH);  // left backward
      analogWrite(Mot_L_EN, -value);
    } else {
      digitalWrite(Mot_L_PH, LOW);  // left forward
      analogWrite(Mot_L_EN, value);
    }
  } else {
    if (value < 0) {
      digitalWrite(Mot_R_PH, HIGH);  // right backward
      analogWrite(Mot_R_EN, -value);
    } else {
      digitalWrite(Mot_R_PH, LOW);  // right forward
      analogWrite(Mot_R_EN, value);
    }
  }
}

void setup() {
  //micro_ros.beginBatteryBroadcaster(&battery_timer_cb);
  //micro_ros.beginRangeBroadcaster(&range_timer_cb);
  micro_ros.beginImuBroadcaster(&imu_timer_cb);
  micro_ros.beginJointStateBroadcaster(&joint_state_timer_cb);
  micro_ros.beginJointStateCommander(&commander_cb);

  // setup wheels
  analogWrite(Mot_L_EN, 0); // left speed pin
  pinMode(Mot_L_PH, OUTPUT);  // left direction pin
  digitalWrite(Mot_L_PH, HIGH);  // left forward
  analogWrite(Mot_R_EN, 0); // right speed pin
  pinMode(Mot_R_PH, OUTPUT);  //right direction pin
  digitalWrite(Mot_R_PH, LOW);  // right forward
  encoder_left.write(0);
  encoder_right.write(0);
  // Clamps the output to a specific range. 0-255 by default
  control_left.SetOutputLimits(-255, 255);
  control_right.SetOutputLimits(-255, 255);
  // object.Start(input, current output, setpoint);
  control_left.Start(0, 0, 0);
  control_right.Start(0, 0, 0);
}

void loop() {
  // keep ROS callbacks running
  micro_ros.spin();

  // keep wheels spinning
  output_l = control_left.Run(input_l);
  set_PWM(LEFT, output_l);
  output_r = control_right.Run(input_r);
  set_PWM(RIGHT, output_r);
}
