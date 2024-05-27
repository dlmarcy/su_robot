#include "MicroROSArduino.h"

MicroROSArduino micro_ros;

double left_setpoint = 10;
double right_setpoint = -20;
double shoulder_setpoint = 72;
double elbow_setpoint = 160;

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
  micro_ros.joint_state_msg.position.data[0]++;
  micro_ros.joint_state_msg.velocity.data[0]++;
  micro_ros.joint_state_msg.effort.data[0]++;
  micro_ros.publishJointState();
}

void commander_cb(const void * msgin)
{  
  const sensor_msgs__msg__JointState * cmd_msg = (const sensor_msgs__msg__JointState *)msgin;
  left_setpoint = cmd_msg->velocity.data[0];
  right_setpoint = cmd_msg->velocity.data[1];
  shoulder_setpoint = cmd_msg->velocity.data[2];
  elbow_setpoint = cmd_msg->velocity.data[3];
  digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));  
}

void setup() {
  micro_ros.beginBatteryBroadcaster(&battery_timer_cb);
  micro_ros.beginRangeBroadcaster(&range_timer_cb);
  micro_ros.beginImuBroadcaster(&imu_timer_cb);
  micro_ros.beginJointStateBroadcaster(&joint_state_timer_cb);
  micro_ros.beginJointStateCommander(&commander_cb);
}

void loop() {
  // keep ROS callbacks running
  micro_ros.spin();
}
