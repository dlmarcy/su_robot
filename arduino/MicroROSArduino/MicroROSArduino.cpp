#include "Arduino.h"
#include "MicroROSArduino.h"

MicroROSArduino::MicroROSArduino()
{
  //create transport
  set_microros_transports();
  //create allocator
  allocator = rcl_get_default_allocator();
  //create init_options
  if (rclc_support_init(&support, 0, NULL, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  // create node
  if (rclc_node_init_default(&node, "micro_ros_arduino_node", "", &support) != RCL_RET_OK) {
    errorLoop();
  }
  battery = false;
  range = false;
  imu = false;
  joint_state = false;
  command = false;
}

void MicroROSArduino::errorLoop()
{
  while(1);
}

void MicroROSArduino::spin()
{
  if (battery) {rclc_executor_spin_some(&battery_executor, RCL_MS_TO_NS(20));}
  if (range) {rclc_executor_spin_some(&range_executor, RCL_MS_TO_NS(20));}
  if (imu) {rclc_executor_spin_some(&imu_executor, RCL_MS_TO_NS(20));}
  if (joint_state) {rclc_executor_spin_some(&joint_state_executor, RCL_MS_TO_NS(20));}
  if (command) {rclc_executor_spin_some(&command_executor, RCL_MS_TO_NS(20));}
}

void MicroROSArduino::beginBatteryBroadcaster(void (*battery_function)(rcl_timer_t*, int64_t))
{
  // create battery publisher
  if (rclc_publisher_init_default(
    &battery_broadcaster,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, BatteryState),
    "arduino/battery") != RCL_RET_OK) {
    errorLoop();
  }
  // create battery timer,
  if (rclc_timer_init_default(
    &battery_timer,
    &support,
    RCL_MS_TO_NS(1000),
    battery_function) != RCL_RET_OK) {
    errorLoop();
  }
  // create executor
  if (rclc_executor_init(&battery_executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  if (rclc_executor_add_timer(&battery_executor, &battery_timer) != RCL_RET_OK) {
    errorLoop();
  }
  // initialize battery msg data
  battery_msg.voltage = 12;
  battery = true;
}

void MicroROSArduino::publishBattery()
{
    if (rcl_publish(&battery_broadcaster, &battery_msg, NULL) != RCL_RET_OK) {}
}

void MicroROSArduino::beginRangeBroadcaster(void (*range_function)(rcl_timer_t*, int64_t))
{
  // create range publisher
  if (rclc_publisher_init_default(
    &range_broadcaster,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Range),
    "arduino/range") != RCL_RET_OK) {
    errorLoop();
  }
  // create range timer,
  if (rclc_timer_init_default(
    &range_timer,
    &support,
    RCL_MS_TO_NS(1000),
    range_function) != RCL_RET_OK) {
    errorLoop();
  }
  // create executor
  if (rclc_executor_init(&range_executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  if (rclc_executor_add_timer(&range_executor, &range_timer) != RCL_RET_OK) {
    errorLoop();
  }
  // initialize battery msg data
  range_msg.range = 0.9;
  range = true;
}

void MicroROSArduino::publishRange()
{
    if (rcl_publish(&range_broadcaster, &range_msg, NULL) != RCL_RET_OK) {}
}

void MicroROSArduino::beginImuBroadcaster(void (*imu_function)(rcl_timer_t*, int64_t))
{
  // create imu publisher
  if (rclc_publisher_init_default(
    &imu_broadcaster,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, Imu),
    "arduino/imu") != RCL_RET_OK) {
    errorLoop();
  }
  // create imu timer,
  if (rclc_timer_init_default(
    &imu_timer,
    &support,
    RCL_MS_TO_NS(1000),
    imu_function) != RCL_RET_OK) {
    errorLoop();
  }
  // create executor
  if (rclc_executor_init(&imu_executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  if (rclc_executor_add_timer(&imu_executor, &imu_timer) != RCL_RET_OK) {
    errorLoop();
  }
  // initialize imu msg data
  imu_msg.angular_velocity.z = 0.1;
  imu = true;
}

void MicroROSArduino::publishImu()
{
    if (rcl_publish(&imu_broadcaster, &imu_msg, NULL) != RCL_RET_OK) {}
}

void MicroROSArduino::beginJointStateBroadcaster(void (*joint_state_function)(rcl_timer_t*, int64_t))
{
  // create joint state publisher
  if (rclc_publisher_init_default(
    &joint_state_broadcaster,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "arduino/joint_states") != RCL_RET_OK) {
    errorLoop();
  }
  // create joint timer,
  if (rclc_timer_init_default(
    &joint_state_timer,
    &support,
    RCL_MS_TO_NS(1000),
    joint_state_function) != RCL_RET_OK) {
    errorLoop();
  }
  // create executor
  if (rclc_executor_init(&joint_state_executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  if (rclc_executor_add_timer(&joint_state_executor, &joint_state_timer) != RCL_RET_OK) {
    errorLoop();
  }
  // initialize joint state msg data
  #define ARRAY_LEN 4
  String joint1 = "motor_left_shaft";
  String joint2 = "motor_right_shaft";
  String joint3 = "motor_shoulder_shaft";
  String joint4 = "motor_elbow_shaft";
  rosidl_runtime_c__String__Sequence__init(&joint_state_msg.name, ARRAY_LEN);
  rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[0], joint1.c_str(), joint1.length());
  rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[1], joint2.c_str(), joint2.length());
  rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[2], joint3.c_str(), joint3.length());
  rosidl_runtime_c__String__assignn(&joint_state_msg.name.data[3], joint4.c_str(), joint4.length());
  joint_state_msg.position.data = (double *) malloc(ARRAY_LEN * sizeof(double));
  joint_state_msg.position.size= ARRAY_LEN;
  joint_state_msg.position.capacity = ARRAY_LEN;
  joint_state_msg.position.data[0] = 1;
  joint_state_msg.position.data[1] = 2;
  joint_state_msg.position.data[2] = 3;
  joint_state_msg.position.data[3] = 4;
  joint_state_msg.velocity.data = (double *) malloc(ARRAY_LEN * sizeof(double));
  joint_state_msg.velocity.size = ARRAY_LEN;
  joint_state_msg.velocity.capacity = ARRAY_LEN;
  joint_state_msg.velocity.data[0] = 10;
  joint_state_msg.velocity.data[1] = 20;
  joint_state_msg.velocity.data[2] = 30;
  joint_state_msg.velocity.data[3] = 40;
  joint_state_msg.effort.data = (double *) malloc(ARRAY_LEN * sizeof(double));
  joint_state_msg.effort.size = ARRAY_LEN;
  joint_state_msg.effort.capacity = ARRAY_LEN;
  joint_state_msg.effort.data[0] = 100;
  joint_state_msg.effort.data[1] = 200;
  joint_state_msg.effort.data[2] = 300;
  joint_state_msg.effort.data[3] = 400;
  joint_state = true;
}

void MicroROSArduino::publishJointState()
{
    if (rcl_publish(&joint_state_broadcaster, &joint_state_msg, NULL) != RCL_RET_OK) {}
}

void MicroROSArduino::beginJointStateCommander(void (*command_function)(const void*))
{
  // create joint state commander
  if (rclc_subscription_init_default(
    &joint_state_commander,
    &node,
    ROSIDL_GET_MSG_TYPE_SUPPORT(sensor_msgs, msg, JointState),
    "arduino/commands") != RCL_RET_OK) {
    errorLoop();
  }
  // create executor
  if (rclc_executor_init(&command_executor, &support.context, 1, &allocator) != RCL_RET_OK) {
    errorLoop();
  }
  if (rclc_executor_add_subscription(&command_executor, &joint_state_commander, &command_msg, command_function, ON_NEW_DATA) != RCL_RET_OK) {
    errorLoop();
  }
  // initialize js cmd data
  #define NUM_JOINTS 4
  String joint1 = "motor_left_shaft";
  String joint2 = "motor_right_shaft";
  String joint3 = "motor_shoulder_shaft";
  String joint4 = "motor_elbow_shaft";
  rosidl_runtime_c__String__Sequence__init(&command_msg.name, NUM_JOINTS);
  rosidl_runtime_c__String__assignn(&command_msg.name.data[0], joint1.c_str(), joint1.length());
  rosidl_runtime_c__String__assignn(&command_msg.name.data[1], joint2.c_str(), joint2.length());
  rosidl_runtime_c__String__assignn(&command_msg.name.data[2], joint3.c_str(), joint3.length());
  rosidl_runtime_c__String__assignn(&command_msg.name.data[3], joint4.c_str(), joint4.length());
  command_msg.position.data = (double *) malloc(NUM_JOINTS * sizeof(double));
  command_msg.position.size= NUM_JOINTS;
  command_msg.position.capacity = NUM_JOINTS;
  command_msg.velocity.data = (double *) malloc(NUM_JOINTS * sizeof(double));
  command_msg.velocity.size = NUM_JOINTS;
  command_msg.velocity.capacity = NUM_JOINTS;
  command_msg.effort.data = (double *) malloc(NUM_JOINTS * sizeof(double));
  command_msg.effort.size = NUM_JOINTS;
  command_msg.effort.capacity = NUM_JOINTS;
  command = true;
}


