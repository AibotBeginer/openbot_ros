"""
  Copyright 2024 The OpenRobotic Beginner Authors
  Copyright 2022 Wyca Robotics (for the ros2 conversion)

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')

     # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='navigation',
        description=('Top-level namespace. The value will be used to replace the '
                     '<robot_namespace> keyword on the rviz config file.'))

    declare_use_namespace_cmd = DeclareLaunchArgument(
        'use_namespace',
        default_value='false',
        description='Whether to apply a namespace to the navigation stack')


    start_openbot_cmd = Node(
        package = 'openbot_ros',
        executable = 'openbot_node',
        parameters = [{'use_sim_time': True}],
        arguments = [
            '-configuration_directory', FindPackageShare('openbot_ros').find('openbot_ros') + '/configuration_files',
            '-configuration_basename', 'openbot.lua'],
        # remappings = [
        #     ('scan', 'horizontal_laser_2d')],
        output = 'screen')

    start_rviz_cmd = Node(
        package = 'rviz2',
        executable = 'rviz2',
        on_exit = Shutdown(),
        arguments = ['-d', FindPackageShare('openbot_ros').find('openbot_ros') + '/rviz/openbot_ros.rviz'],
        parameters = [{'use_sim_time': True}],
    )

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)

    # Add any conditioned actions
    # ld.add_action(start_rviz_cmd)
    ld.add_action(start_openbot_cmd)

    return ld