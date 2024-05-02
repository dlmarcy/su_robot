import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from sensor_msgs.msg import Imu
from sensor_msgs.msg import Range
from sensor_msgs.msg import BatteryState
import math

class Teensy_Sim(Node):

	def __init__(self):
		super().__init__('teensy_sim')
		
		# create publishers and message variables
		self.pub_motors = self.create_publisher(JointState, 'teensy/motors', 10)
		self.pub_imu = self.create_publisher(Imu, 'teensy/imu', 10)
		self.pub_range = self.create_publisher(Range, 'teensy/range', 10)
		self.pub_battery = self.create_publisher(BatteryState, 'teensy/battery', 10)
		self.motor_msg = JointState()
		self.motor_msg.name = ['motor_left_shaft', 'motor_right_shaft', 'motor_shoulder_shaft', 'motor_elbow_shaft']
		self.motor_msg.position = [0.0, 0.0, 0.0, 0.0]
		self.motor_msg.velocity = [0.0, 0.0, 0.0, 0.0]
		self.imu_msg = Imu()
		self.range_msg = Range()
		self.battery_msg = BatteryState()
		
		# create subscribers and message variables
		self.sub_commands = self.create_subscription(JointState, 'teensy/commands', self.cmd_callback, 10)
		self.sub_commands  # prevent unused variable warning
		self.command_msg = JointState()
		
		# create timing variables for a tick timer
		self.TIMER_TICK = 10 # milliseconds
		self.TIMER_PERIOD = self.TIMER_TICK * 1e-3
		self.TIMER_RATE = 1.0/self.TIMER_PERIOD
		self.tick_timer = self.create_timer(self.TIMER_PERIOD, self.tick_callback)
		self.ticks = 0

		# robot constants
		self.ARM_ACCEL = 2.0 # rad/s/s
		self.TIRE_DIA = 0.08 # meter
		self.TIRE_SEP = 0.25 # meter
		self.BASE_SENSOR_OFFSET = 0.155
		self.TIRE_SCALE_SEP = self.TIRE_DIA/(2*self.TIRE_SEP)
		self.TIRE_SCALE_DIA = self.TIRE_DIA/4
		self.ARM_DELTA_V = self.ARM_ACCEL * self.TIMER_PERIOD
		
		# range constants
		self.MAX_DISTANCE = 1.2 # meters
		#self.SEGMENTS = self.get_parameter('teensy/segments').get_parameter_value().double_array_value
		self.SEGMENTS = []

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
		self.a1 = 0.06839991
		self.a0 = -0.05605661
		self.b1 = -1.54671584
		self.b0 = 0.55905915
		self.control_left = 0.0
		self.control_left_k1 = 0.0
		self.control_left_k2 = 0.0
		self.control_right = 0.0
		self.control_right_k1 = 0.0
		self.control_right_k2 = 0.0
		self.control_shoulder = 0.0
		self.control_elbow = 0.0
		
	def tick_callback(self):
		self.ticks = (self.ticks + 1) % 100
		self.update_robot_state()
		if self.ticks % 5 == 0:
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
			self.pub_imu.publish(self.imu_msg)
		if (self.ticks + 7) % 10 == 0:
			self.motor_msg.position[0] = self.left_pos
			self.motor_msg.velocity[0] = self.left_vel
			self.motor_msg.position[1] = self.right_pos
			self.motor_msg.velocity[1] = self.right_vel
			self.motor_msg.position[2] = self.shoulder_pos
			self.motor_msg.velocity[2] = self.shoulder_vel
			self.motor_msg.position[3] = self.elbow_pos
			self.motor_msg.velocity[3] = self.elbow_vel
			self.motor_msg.header.stamp = self.get_clock().now().to_msg()
			self.pub_motors.publish(self.motor_msg)
		if (self.ticks + 2) % 10 == 0:
			d = self.detect_range()
			self.range_msg.range = d
			self.pub_range.publish(self.range_msg)
		if self.ticks == 9:
			self.battery_msg.voltage = 12.0
			self.battery_msg.current = -0.7
			self.pub_battery.publish(self.battery_msg)

	def cmd_callback(self, cmd):
		self.control_left = cmd.velocity[0]
		self.control_right = cmd.velocity[1]
		self.control_shoulder = cmd.velocity[2]
		self.control_elbow = cmd.velocity[3]

	def update_robot_state(self):
		self.left_vel_k2 = self.left_vel_k1
		self.left_vel_k1 = self.left_vel
		self.right_vel_k2 = self.right_vel_k1
		self.right_vel_k1 = self.right_vel
		self.control_left_k2 = self.control_left_k1
		self.control_left_k1 = self.control_left
		self.control_right_k2 = self.control_right_k1
		self.control_right_k1 = self.control_right
		self.left_vel = -self.b1*self.left_vel_k1 - self.b0*self.left_vel_k2
		self.left_vel += (self.a1*self.control_left_k1 + self.a0*self.control_left_k2)
		self.left_pos += (self.left_vel * self.TIMER_PERIOD)
		self.right_vel = -self.b1*self.right_vel_k1 - self.b0*self.right_vel_k2
		self.right_vel += (self.a1*self.control_right_k1 + self.a0*self.control_right_k2)
		self.right_pos += (self.right_vel * self.TIMER_PERIOD)
		self.shoulder_vel = self.accelerate_motor(self.control_shoulder, self.shoulder_vel, self.ARM_DELTA_V)
		self.shoulder_pos += (self.shoulder_vel * self.TIMER_PERIOD)
		self.elbow_vel = self.accelerate_motor(self.control_elbow, self.elbow_vel, self.ARM_DELTA_V)
		self.elbow_pos += (self.elbow_vel * self.TIMER_PERIOD)
		self.prev_w = self.w
		self.w = (self.right_vel - self.left_vel) * self.TIRE_SCALE_SEP
		self.prev_v = self.v
		self.v = (self.right_vel + self.left_vel) * self.TIRE_SCALE_DIA
		self.a = (self.v - self.prev_v) * self.TIMER_RATE
		self.x += (self.v * math.cos(self.theta) * self.TIMER_PERIOD)
		self.y += (self.v * math.sin(self.theta) * self.TIMER_PERIOD)
		self.theta += (self.w * self.TIMER_PERIOD)
	
	def accelerate_motor(self, setpoint, current_vel, delta_v):
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

