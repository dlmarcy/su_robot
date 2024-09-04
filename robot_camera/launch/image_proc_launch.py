# Copyright 2018 Open Source Robotics Foundation, Inc.
# Copyright 2019 Samsung Research America
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

from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
import launch_ros.actions
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.substitutions import TextSubstitution
import os
import yaml
from launch.substitutions import EnvironmentVariable
import pathlib
import launch.actions
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
	image_proc_launch = os.path.join(get_package_share_directory('image_proc'), 'launch', 'image_proc.launch.py')

	image_proc_namespace = DeclareLaunchArgument('camera_ns', default_value=TextSubstitution(text='camera'))
	
	#image_proc_include = IncludeLaunchDescription(PythonLaunchDescriptionSource(image_proc_launch))
	
	image_proc_include = GroupAction(
		actions=[
			# push-ros-namespace to set namespace of included nodes
			PushRosNamespace(LaunchConfiguration('camera_ns')),
			IncludeLaunchDescription(PythonLaunchDescriptionSource(image_proc_launch))
		]
	)

	return LaunchDescription([
		image_proc_namespace,
		image_proc_include
])
