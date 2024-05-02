import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class DriveSquare(Node):

	def __init__(self):
		super().__init__('drive_square')
		
		# create cmd_vel publisher variables
		self.pub_cmd_vel = self.create_publisher(Twist, 'diff_drive_controller/cmd_vel_unstamped', 10)
		self.cmd_vel_msg = Twist()
		
		# create timer variables
		self.timer = self.create_timer(0.1, self.timer_callback)
		self.count = 0
		
	def timer_callback(self):
		if self.count == 0:
			self.cmd_vel_msg.linear.x = 0.1
			self.cmd_vel_msg.angular.z = 0.0
		elif self.count == 100:
			self.cmd_vel_msg.linear.x = 0.0
			self.cmd_vel_msg.angular.z = 0.15707963268
		self.pub_cmd_vel.publish(self.cmd_vel_msg)
		self.count = (self.count + 1) % 200

def main(args=None):
	rclpy.init(args=args)

	drive = DriveSquare()

	print('Starting drive node')

	rclpy.spin(drive)

	# Destroy the node explicitly
	# (optional - otherwise it will be done automatically
	# when the garbage collector destroys the node object)
	simulator.destroy_node()
	rclpy.shutdown()

if __name__ == '__main__':
    main()
