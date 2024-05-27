#ifndef MICROROSARDUINO_H
#define MICROROSARDUINO_H

#include "Arduino.h"
#include <micro_ros_arduino.h>
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <sensor_msgs/msg/battery_state.h>
#include <sensor_msgs/msg/range.h>
#include <sensor_msgs/msg/imu.h>
#include <sensor_msgs/msg/joint_state.h>
#include <rosidl_runtime_c/string_functions.h>

class MicroROSArduino
{
  public:
    MicroROSArduino();
    void errorLoop();
    void spin();
    // battery broadcaster
    void beginBatteryBroadcaster(void (*battery_function)(rcl_timer_t*, int64_t));
    void publishBattery();
    sensor_msgs__msg__BatteryState battery_msg;
    // range broadcaster
    void beginRangeBroadcaster(void (*range_function)(rcl_timer_t*, int64_t));
    void publishRange();
    sensor_msgs__msg__Range range_msg;
    // imu broadcaster
    void beginImuBroadcaster(void (*imu_function)(rcl_timer_t*, int64_t));
    void publishImu();
    sensor_msgs__msg__Imu imu_msg;
    // joint state broadcaster
    void beginJointStateBroadcaster(void (*joint_state_function)(rcl_timer_t*, int64_t));
    void publishJointState();
    sensor_msgs__msg__JointState joint_state_msg;
    // joint state commander
    void beginJointStateCommander(void (*command_function)(const void*));
    sensor_msgs__msg__JointState command_msg;
  private:
    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    // battery broadcaster
    rclc_executor_t battery_executor;
    rcl_timer_t battery_timer;
    rcl_publisher_t battery_broadcaster;
    bool battery;
    // range broadcaster
    rclc_executor_t range_executor;
    rcl_timer_t range_timer;
    rcl_publisher_t range_broadcaster;
    bool range;
    // imu broadcaster
    rclc_executor_t imu_executor;
    rcl_timer_t imu_timer;
    rcl_publisher_t imu_broadcaster;
    bool imu;
    // joint state broadcaster
    rclc_executor_t joint_state_executor;
    rcl_timer_t joint_state_timer;
    rcl_publisher_t joint_state_broadcaster;
    bool joint_state;
    // joint state commander
    rclc_executor_t command_executor;
    rcl_subscription_t joint_state_commander;
    bool command;
};

#endif
