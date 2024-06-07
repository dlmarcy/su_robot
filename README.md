# SU robot

ros-iron

## Micro Ros
### Arduino side

Download the latest micro-ROS/micro_ros_arduino zip file  
v2.0.7-iron  
extract it into the Arduino libraries folder  

### Linux side

Make a microros_ws  
Follow the teensy tutorial to clone the micro_ros_setup package  
colcon build the package  
run the scripts to create the micro_ros_agent  
then build the micro_ros-agent  
then you can source the workspace to get the micro_ros_agent package in ROS  

## Arduino

Teensy 4.0

| Pin Name      | Type          | SU 1 - 8      | Number 9      |
| ------------- | ------------- | ------------- | ------------- |
| Mot_L_EN      | Analog PWM    | 4             | 4             |
| Mot_L_PH      | Digital Out   | 5             | 5             |
| Mot_R_EN      | Analog PWM    | 3             | 3             |
| Mot_R_PH      | Digital Out   | 2             | 2             |
| Encode_L1     | Digital In    | 17            | 12            |
| Encode_L2     | Digital In    | 16            | 6             |
| Encode_R1     | Digital In    | 0             | 0             |
| Encode_R2     | Digital In    | 1             | 1             |
| Step1_Or      | Digital Out   | 20            | 11            |
| Step1_Ye      | Digital Out   | 6             | 10            |
| Step1_Pi      | Digital Out   | 12            | 9             |
| Step1_Bl      | Digital Out   | 11            | 8             |
| Step2_Or      | Digital Out   | 7             | 7             |
| Step2_Ye      | Digital Out   | 8             | 16            |
| Step2_Pi      | Digital Out   | 9             | 17            |
| Step2_Bl      | Digital Out   | 10            | 20            |
| LED_GRN1      | Analog PWM    | 14            | 14            |
| LED_RED1      | Analog PWM    | 15            | 15            |
| LED_GRN2      | Analog PWM    | 22            | 23            |
| LED_RED2      | Analog PWM    | 23            | 22            |
| Teensy_LED    | Digital Out   | 13            | 13            |
| Push_But      | Digital In PD | 21            | 21            |

## ROS
###ROS controls

sudo apt install ros-iron-ros2-control  
sudo apt install ros-iron-ros2-controllers  
sudo apt install ros-iron-topic-based-ros2-control  






