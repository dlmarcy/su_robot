import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	usb_cam_launch = os.path.join(get_package_share_directory('robot_camera'), 'launch', 'usb_cam_launch.py')
	image_proc_launch = os.path.join(get_package_share_directory('robot_camera'), 'launch', 'image_proc_launch.py')
	apriltag_launch = os.path.join(get_package_share_directory('robot_camera'), 'launch', 'april_tag_launch.py')
	
	usb_cam_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(usb_cam_launch))
	image_proc_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(image_proc_launch))
	apriltag_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(apriltag_launch))

	return LaunchDescription([
		usb_cam_include,
		image_proc_include,
		apriltag_include,
	])
