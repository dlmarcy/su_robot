import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
	control_launch = os.path.join(get_package_share_directory('robot_control'), 'launch', 'real_control_launch.py')
	ekf_launch = os.path.join(get_package_share_directory('robot_base'), 'launch', 'ekf_launch.py')
	map_launch = os.path.join(get_package_share_directory('robot_base'), 'launch', 'map_server_launch.py')
	nav_launch = os.path.join(get_package_share_directory('robot_base'), 'launch', 'navigation_launch.py')
	rviz2_file = os.path.join(get_package_share_directory('robot_base'), 'rviz', 'su_robot.rviz')
	
	control_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch))
	ekf_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(ekf_launch))
	map_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(map_launch))
	nav_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(nav_launch))

	return LaunchDescription([
		control_include,
		ekf_include,
		map_include,
		nav_include,
#		Node(
#			package='rviz2',
#			executable='rviz2',
#			name = 'rviz2',
#			arguments=['-d', [rviz2_file]]
#		),
#		Node(
#			package='rqt_gui',
#			executable='rqt_gui',
#			name = 'rqt_gui',
#		)
	])

