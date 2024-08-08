from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
	camera_params = os.path.join(get_package_share_directory('robot_camera'), 'params', 'camera_params.yaml')

	return LaunchDescription([
		launch_ros.actions.Node(
		package='usb_cam',
		executable='usb_cam_node_exe',
		name='usb_cam_node',
		namespace='camera',
		output='screen',
		parameters=[camera_params],
	),
])

#import os
#from ament_index_python.packages import get_package_share_directory
#from launch import LaunchDescription
#from launch.actions import IncludeLaunchDescription
#from launch.launch_description_sources import PythonLaunchDescriptionSource
#from launch_ros.actions import Node
#
#def generate_launch_description():
#	usb_cam_launch = os.path.join(get_package_share_directory('usb_cam'), 'launch', 'camera.launch.py')
#	
#	usb_cam_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(usb_cam_launch))
#
#	return LaunchDescription([
#		usb_cam_include,
#	])
