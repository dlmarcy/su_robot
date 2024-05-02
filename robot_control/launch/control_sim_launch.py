import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	urdf_path = os.path.join(get_package_share_directory('robot_control'), 'urdf', 'robot.urdf')
	urdf_file = open(urdf_path).read()
	control_params = os.path.join(get_package_share_directory('robot_control'), 'params', 'control_params.yaml')

	return LaunchDescription([
		Node(
			package = 'micro_sim',
			executable = 'teensy_sim',
			name = 'teensy_sim',
			output={'stdout': 'screen', 'stderr': 'screen'}
		),
		Node(
			package='robot_state_publisher',
			executable='robot_state_publisher',
			name = 'robot_state_publisher',
			output={'stdout': 'screen', 'stderr': 'screen'},
			parameters=[{'robot_description': urdf_file}]
		),
		Node(
			package="controller_manager",
			executable="ros2_control_node",
			output={'stdout': 'screen', 'stderr': 'screen'},
			#parameters=[{'robot_description': urdf_file}, control_params]
			parameters=[control_params],
			remappings=[('/controller_manager/robot_description', '/robot_description')]
		),
		Node(
			package="controller_manager",
			executable="spawner",
			arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
		),
		Node(
			package="controller_manager",
			executable="spawner",
			arguments=["diff_drive_controller", "--controller-manager", "/controller_manager"],
		)]
	)

