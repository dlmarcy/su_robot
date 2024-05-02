#!/usr/bin/env python3

# Copyright (c) 2020 Samsung Research Russia
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
	# Parameters
	lifecycle_nodes = ['map_server']
	use_sim_time = False
	autostart = True
	map_file = os.path.join(get_package_share_directory('robot_base'), 'map', 'map_config.yaml')

	# Nodes launching commands
	start_map_server_cmd = launch_ros.actions.Node(
		package='nav2_map_server',
		executable='map_server',
		output='screen',
		emulate_tty=True,  # https://github.com/ros2/launch/issues/188
		parameters=[{'yaml_filename': map_file}])

	start_lifecycle_manager_cmd = launch_ros.actions.Node(
		package='nav2_lifecycle_manager',
		executable='lifecycle_manager',
		name='lifecycle_manager',
		output='screen',
		emulate_tty=True,  # https://github.com/ros2/launch/issues/188
		parameters=[{'use_sim_time': use_sim_time},
					{'autostart': autostart},
					{'node_names': lifecycle_nodes}])

	ld = LaunchDescription()

	ld.add_action(start_map_server_cmd)
	ld.add_action(start_lifecycle_manager_cmd)

	return ld
