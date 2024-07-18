import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
import math
import random

class Teensy_Sim(Node):

	def __init__(self):
		super().__init__('teensy_sim')

		# robot state variables
		self.left_pos = 0.0
		self.left_pos_k1 = 0.0
		self.left_pos_k2 = 0.0
		self.left_vel = 0.0
		self.left_vel_k1 = 0.0
		self.left_vel_k2 = 0.0
		self.right_pos = 0.0
		self.right_pos_k1 = 0.0
		self.right_pos_k2 = 0.0
		self.right_vel = 0.0
		self.right_vel_k1 = 0.0
		self.right_vel_k2 = 0.0
		self.x = 0.0
		self.y = 0.0
		self.prev_v = 0.0
		self.v = 0.0
		self.a = 0.0
		self.theta = 0.0
		self.prev_w = 0.0
		self.w = 0.0
		self.shoulder_pos = 0.0
		self.shoulder_vel = 0.0
		self.elbow_pos = 0.0
		self.elbow_vel = 0.0

		# motor model and velocity setpoints
		self.a0 = 0
		self.a1 = 0.04651454
		self.a2 = -0.01953466
		self.a3 = -0.01519043
		self.b0 = 1
		self.b1 = -1.58135638
		self.b2 = 0.64295142
		self.b3 = -0.0498056
		self.setpoint_left = 0.0
		self.setpoint_left_k1 = 0.0
		self.setpoint_left_k2 = 0.0
		self.setpoint_left_k3 = 0.0
		self.setpoint_right = 0.0
		self.setpoint_right_k1 = 0.0
		self.setpoint_right_k2 = 0.0
		self.setpoint_right_k3 = 0.0
		self.setpoint_shoulder = 0.0
		self.setpoint_elbow = 0.0

		# create timing variables for robot model timer
		self.TIMER_PERIOD = 10e-3
		self.TIMER_RATE = 1.0/self.TIMER_PERIOD
		self.robot_model_timer = self.create_timer(self.TIMER_PERIOD, self.robot_model_timer_cb)

		# robot constants
		self.ARM_ACCEL = 2.0 # rad/s/s
		self.TIRE_DIA = 0.08 # meter
		self.TIRE_SEP = 0.25 # meter
		self.BASE_SENSOR_OFFSET = 0.155
		self.TIRE_SCALE_SEP = self.TIRE_DIA/(2*self.TIRE_SEP)
		self.TIRE_SCALE_DIA = self.TIRE_DIA/4
		self.ARM_DELTA_V = self.ARM_ACCEL * self.TIMER_PERIOD
		
		# range constants
		self.MAX_DISTANCE = 1.0 # meters
		#self.SEGMENTS = self.get_parameter('teensy/segments').get_parameter_value().double_array_value
		self.SEGMENTS = []

		# set up power monitor
		self.battery_broadcaster = self.create_publisher(BatteryState, 'arduino/battery', 10)
		self.battery_timer = self.create_timer(1.0/1.0, self.battery_timer_cb)
		self.battery_msg = BatteryState()

		# set up range sensor
		self.range_broadcaster = self.create_publisher(Range, 'arduino/range', 10)
		self.range_timer = self.create_timer(1.0/5.0, self.range_timer_cb)
		self.range_msg = Range()

		# set up IMU
		self.imu_broadcaster = self.create_publisher(Imu, 'arduino/imu', 10)
		self.imu_timer = self.create_timer(1.0/20.0, self.imu_timer_cb)
		self.imu_msg = Imu()

		# set up encoders and broadcaster
		self.joint_state_brodcaster = self.create_publisher(JointState, 'arduino/joint_states', 10)
		self.joint_state_timer = self.create_timer(1.0/10.0, self.joint_state_timer_cb)
		self.joint_state_msg = JointState()
		self.joint_state_msg.name = ['motor_left_shaft', 'motor_right_shaft', 'motor_shoulder_shaft', 'motor_elbow_shaft']
		self.joint_state_msg.position = [0.0, 0.0, 0.0, 0.0]
		self.joint_state_msg.velocity = [0.0, 0.0, 0.0, 0.0]
		
		# set up PID controller, motors, and commander
		self.joint_state_commander = self.create_subscription(JointState, 'arduino/commands', self.commander_cb, 10)
		self.joint_state_commander  # prevent unused variable warning
		self.command_msg = JointState()
		
	def battery_timer_cb(self):
		self.battery_msg.voltage = 12.0
		self.battery_msg.current = -0.7
		self.battery_msg.header.stamp = self.get_clock().now().to_msg()
		self.battery_broadcaster.publish(self.battery_msg)
		
	def range_timer_cb(self):
		d = self.detect_range()
		self.range_msg.range = d
		self.range_msg.header.stamp = self.get_clock().now().to_msg()
		self.range_broadcaster.publish(self.range_msg)
		
	def imu_timer_cb(self):
		self.imu_msg.orientation.w = math.cos(0.5 * self.theta)
		self.imu_msg.orientation.z = math.sin(0.5 * self.theta)
		self.imu_msg.angular_velocity.z = self.w * 57.295779513 # teensy BNO055 is using degrees/s
		if self.w == 0.0:
			self.imu_msg.linear_acceleration.x = self.a
			self.imu_msg.linear_acceleration.y = 0.0
		else:
			R = self.v/self.w
			self.imu_msg.linear_acceleration.x = self.a + R * (self.w - self.prev_w) * self.TIMER_RATE
			if self.w < 0.0:
				self.imu_msg.linear_acceleration.y = -R * self.w * self.w
			else:
				self.imu_msg.linear_acceleration.y = R * self.w * self.w
		self.imu_msg.header.stamp = self.get_clock().now().to_msg()
		self.imu_broadcaster.publish(self.imu_msg)

	def joint_state_timer_cb(self):
		self.ticks = (self.ticks + 1) % 100
		self.joint_state_msg.position[0] = self.left_pos
		self.joint_state_msg.velocity[0] = self.left_vel
		self.joint_state_msg.position[1] = self.right_pos
		self.joint_state_msg.velocity[1] = self.right_vel
		self.joint_state_msg.position[2] = self.shoulder_pos
		self.joint_state_msg.velocity[2] = self.shoulder_vel
		self.joint_state_msg.position[3] = self.elbow_pos
		self.joint_state_msg.velocity[3] = self.elbow_vel
		self.joint_state_msg.header.stamp = self.get_clock().now().to_msg()
		self.joint_state_broadcaster.publish(self.joint_state_msg)

	def commander_cb(self, cmd):
		self.setpoint_left = cmd.velocity[0]
		self.setpoint_right = cmd.velocity[1]
		self.setpoint_shoulder = cmd.velocity[2]
		self.setpoint_elbow = cmd.velocity[3]

	def robot_model_timer_cb(self):
		self.left_vel_k3 = self.left_vel_k2
		self.left_vel_k2 = self.left_vel_k1
		self.left_vel_k1 = self.left_vel
		self.right_vel_k3 = self.right_vel_k2
		self.right_vel_k2 = self.right_vel_k1
		self.right_vel_k1 = self.right_vel
		self.setpoint_left_k3 = self.setpoint_left_k2
		self.setpoint_left_k2 = self.setpoint_left_k1
		self.setpoint_left_k1 = self.setpoint_left
		self.setpoint_right_k3 = self.setpoint_right_k2
		self.setpoint_right_k2 = self.setpoint_right_k1
		self.setpoint_right_k1 = self.setpoint_right
		self.left_vel = -self.b1*self.left_vel_k1 - self.b2*self.left_vel_k2 - self.b3*self.left_vel_k3
		self.left_vel += (self.a1*self.setpoint_left_k1 + self.a2*self.setpoint_left_k2+ self.a3*self.setpoint_left_k3)
		self.left_pos += (self.left_vel * self.TIMER_PERIOD)
		self.right_vel = -self.b1*self.right_vel_k1 - self.b2*self.right_vel_k2 - self.b3*self.right_vel_k3
		self.right_vel += (self.a1*self.setpoint_right_k1 + self.a2*self.setpoint_right_k2+ self.a3*self.setpoint_right_k3)
		self.right_pos += (self.right_vel * self.TIMER_PERIOD)
		self.shoulder_vel = self.accelerate_stepper(self.setpoint_shoulder, self.shoulder_vel, self.ARM_DELTA_V)
		self.shoulder_pos += (self.shoulder_vel * self.TIMER_PERIOD)
		self.elbow_vel = self.accelerate_stepper(self.setpoint_elbow, self.elbow_vel, self.ARM_DELTA_V)
		self.elbow_pos += (self.elbow_vel * self.TIMER_PERIOD)
		self.prev_w = self.w
		slip_l = 1.0/random.paretovariate(100)
		slip_r = 1.0/random.paretovariate(150)
		self.w = (slip_r*self.right_vel - slip_l*self.left_vel) * self.TIRE_SCALE_SEP
		self.prev_v = self.v
		self.v = (slip_r*self.right_vel + slip_l*self.left_vel) * self.TIRE_SCALE_DIA
		self.a = (self.v - self.prev_v) * self.TIMER_RATE
		self.x += (self.v * self.TIMER_PERIOD * math.cos(self.theta))
		self.y += (self.v * self.TIMER_PERIOD * math.sin(self.theta))
		self.theta += (self.w * self.TIMER_PERIOD)
	
	def accelerate_stepper(self, setpoint, current_vel, delta_v):
		if setpoint > current_vel:
			vel = current_vel + delta_v
			if vel > setpoint:
				vel = setpoint
		else:
			vel = current_vel - delta_v
			if vel < setpoint:
				vel = setpoint
		return vel

	def detect_range(self):
		distance = self.MAX_DISTANCE
		if self.SEGMENTS == []:
			return distance
		x = self.x + self.BASE_SENSOR_OFFSET * math.cos(self.theta)
		y = self.y + self.BASE_SENSOR_OFFSET * math.sin(self.theta)
		for segment in self.SEGMENTS:
			a1 = self.MAX_DISTANCE * math.cos(self.theta)
			b1 = segment[0] - segment[2]
			a2 = self.MAX_DISTANCE * math.sin(self.theta)
			b2 = segment[1] - segment[3]
			if a1*b2!=a2*b1:
				c1 = segment[0] - x
				c2 = segment[1] - y
				s0 = (c1*b2-c2*b1)/(a1*b2-a2*b1)
				if 0 <= s0 <= 1:
					t0 = (a1*c2-a2*c1)/(a1*b2-a2*b1)
					if 0 <= t0 <= 1:
						d = s0*math.sqrt(a1*a1+a2*a2)
						if d < distance:
							distance = d
		return distance

def main(args=None):
	rclpy.init(args=args)

	simulator = Teensy_Sim()

	print('Starting Teensy simulation node')

	rclpy.spin(simulator)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	simulator.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()

