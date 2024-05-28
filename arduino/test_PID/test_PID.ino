/*
  test_PID SU 6/3/2023
*/

#include <PID_v2.h>
#define ENCODER_OPTIMIZE_INTERRUPTS
#include <Encoder.h>

int Mot_L_EN = 4;
int Mot_L_PH = 5;
int Mot_R_EN = 3;
int Mot_R_PH = 2;
int Encode_L1 = 17;
int Encode_L2 = 16;
int Encode_R1 = 0;
int Encode_R2 = 1;

int left = 1;
int right = 2;

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
float scale_angle_pos = 99*48/(2*3.14159); // encoder counts per radian
float scale_steps_pos = 1.0/scale_angle_pos;  // radians per encoder count
// The loop for measuring velocity counts for 1/10 second
float scale_angle_vel = scale_angle_pos*0.1;
float scale_steps_vel = 1.0/scale_angle_vel;

// timing variables
long check_time = millis();
unsigned int check_duration = 25;
unsigned int check_count = 0;
unsigned int check_seconds = 0;

void set_PWM(int motor, int value) {
  if (motor == left) {
    if (value < 0) {
      digitalWrite(Mot_L_PH, LOW);  // left backward
      analogWrite(Mot_L_EN, -value);
    } else {
      digitalWrite(Mot_L_PH, HIGH);  // left forward
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
  if (millis()-check_time > check_duration) {
    check_time += check_duration;
    check_count++;
    check_count = check_count % 40;
    if (((check_count + 3) % 4) == 0) {
      previous_l = encoder_l;
      encoder_l = encoder_left.read();
      previous_r = encoder_r;
      encoder_r = encoder_right.read();
      input_l = scale_steps_vel*(encoder_l - previous_l); // radians per second
      input_r = scale_steps_vel*(encoder_r - previous_r); // radians per second
    } else if (check_count == 0) {
      check_seconds++;
      check_seconds = check_seconds % 3;
      if (check_seconds == 0) {
        setpoint_l += 1.0471975512;
        if (setpoint_l > 3.5) {
          setpoint_l = -3.14159265359;
        }
        setpoint_r -= 1.0471975512;
        if (setpoint_r < -3.5) {
          setpoint_r = 3.14159265359;
        }
        control_left.Setpoint(setpoint_l);
        control_right.Setpoint(setpoint_r);
      }
    }
  }
  output_l = control_left.Run(input_l);
  set_PWM(left, output_l);
  output_r = control_right.Run(input_r);
  set_PWM(right, output_r);
}
