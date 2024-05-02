import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
	control_launch = os.path.join(get_package_share_directory('robot_control'), 'launch', 'control_sim_launch.py')
	ekf_launch = os.path.join(get_package_share_directory('robot_base'), 'launch', 'ekf_launch.py')
	map_launch = os.path.join(get_package_share_directory('robot_base'), 'launch', 'map_server_launch.py')
	
	control_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(control_launch))
	ekf_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(ekf_launch))
	map_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(map_launch))

	return LaunchDescription([
		control_include,
		ekf_include,
		map_include
	])
