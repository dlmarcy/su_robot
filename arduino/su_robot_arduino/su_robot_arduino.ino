#include "MicroROSArduino.h"
#include <VL53L0X.h>
#include <Adafruit_INA219.h>
#include <Adafruit_BNO055.h>
#include <EEPROM.h>
#include <PID_v2.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>
#include <AccelStepper.h>

// define pin numbers
int Mot_L_EN = 4;
int Mot_L_PH = 5;
int Mot_R_EN = 3;
int Mot_R_PH = 2;
int Encode_L1 = 12;
int Encode_L2 = 6;
int Encode_R1 = 0;
int Encode_R2 = 1;

int left = 1;
int right = 2;

// create a micro ros object
MicroROSArduino micro_ros;

// define the encoder variables and objects
long encoder_l = 0;
long encoder_r = 0;
long previous_l = 0;
long previous_r = 0;
Encoder encoder_left(Encode_L1, Encode_L2); 
Encoder encoder_right(Encode_R2, Encode_R1); 

// default sample time for the PID controller is 100ms
// can be changed with object.SetSampleTime(int NewSampleTime);
// change setpoint with object.Setpoint(double v);
double Kp = 5.0, Ki = 100.0, Kd = 0.0;
PID_v2 control_left(Kp, Ki, Kd, PID::Direct);
PID_v2 control_right(Kp, Ki, Kd, PID::Direct);

// controller variables
double input_l = 0.0;
double output_l = 0.0;
double setpoint_l = 0.0;
double input_r = 0.0;
double output_r = 0.0;
double setpoint_r = 0.0;

// The documentation says the motor encoders count 48 edges per revolution
// The gear ratio is 99:1, 99*48 = 4752
// The wheels are 80 mm diameter
float scale_counts_pos = 99*48/(2*3.14159); // encoder counts per radian
float scale_radians_pos = 1.0/scale_counts_pos;  // radians per encoder count
// The loop for measuring velocity counts for 1/10 second
float scale_counts_vel = scale_counts_pos*0.1;
float scale_radians_vel = 1.0/scale_counts_vel;

double shoulder_setpoint = 0;
double elbow_setpoint = 0;

void battery_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.battery_msg.voltage = 10;
  micro_ros.publishBattery();
}

void range_timer_cb(rcl_timer_t * timer, int64_t last_call_time) {
  micro_ros.range_msg.range = 0.5;
  micro_ros.publishRange();
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
  micro_ros.publishJointState();
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
  if (motor == left) {
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
  set_PWM(left, output_l);
  output_r = control_right.Run(input_r);
  set_PWM(right, output_r);
}
