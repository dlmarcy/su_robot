import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState

class Arduino_Control(Node):

	def __init__(self):
		super().__init__('arduino_control')
		
		# create publishers and message variables
		self.pub_motors = self.create_publisher(JointState, 'arduino/commands', 10)
		self.motor_msg = JointState()
		self.motor_msg.name = ['gear_left_shaft', 'gear_right_shaft']
		self.motor_msg.position = [0.0, 0.0]
		self.motor_msg.velocity = [0.0, 0.0]
		
		# create timing variables for a tick timer
		self.TIMER_TICK = 200 # milliseconds
		self.TIMER_PERIOD = self.TIMER_TICK * 1e-3
		self.TIMER_RATE = 1.0/self.TIMER_PERIOD
		self.tick_timer = self.create_timer(self.TIMER_PERIOD, self.tick_callback)
		self.ticks = 0

	def tick_callback(self):
		self.ticks = (self.ticks + 1) % 100
		if self.ticks == 0:
			self.motor_msg.velocity = [0.0, 0.0] # radians per second
			self.motor_msg.header.stamp = self.get_clock().now().to_msg()
			self.pub_motors.publish(self.motor_msg)
		elif self.ticks == 25:
			self.motor_msg.velocity = [4.0, 0.0] # radians per second
			self.motor_msg.header.stamp = self.get_clock().now().to_msg()
			self.pub_motors.publish(self.motor_msg)
		elif self.ticks == 50:
			self.motor_msg.velocity = [0.0, -2.0] # radians per second
			self.motor_msg.header.stamp = self.get_clock().now().to_msg()
			self.pub_motors.publish(self.motor_msg)
		elif self.ticks == 75:
			self.motor_msg.velocity = [-6.0, 3.0] # radians per second
			self.motor_msg.header.stamp = self.get_clock().now().to_msg()
			self.pub_motors.publish(self.motor_msg)

def main(args=None):
	rclpy.init(args=args)

	control_node = Arduino_Control()

	rclpy.spin(control_node)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	control_node.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()

