import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	urdf_path = os.path.join(get_package_share_directory('robot_control'), 'urdf', 'robot.urdf')
	urdf_file = open(urdf_path).read()

	return LaunchDescription([
		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name = 'robot_state_publisher',
			output={'stdout': 'screen', 'stderr': 'screen'},
			parameters=[{'robot_description': urdf_file}]
		),
		Node(
			package='joint_state_publisher_gui',
			executable='joint_state_publisher_gui',
			name = 'joint_state_publisher_gui',
			output={'stdout': 'screen', 'stderr': 'screen'},
			remappings=[('/joint_states', '/arduino/commands')]
		)]
	)

