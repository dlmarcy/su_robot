import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	apriltag_params = os.path.join(get_package_share_directory('robot_camera'), 'params', 'apriltag_params.yaml')

	return LaunchDescription([
		Node(
			package="apriltag_ros",
			executable="apriltag_node",
			arguments=["--ros-args", "-r", "image_rect:=/camera/image_rect", "-r", "camera_info:=/camera/camera_info"],
			parameters=[apriltag_params],
		)]
	)

