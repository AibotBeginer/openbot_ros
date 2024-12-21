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

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch import LaunchService
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

def generate_launch_description():
    # Get the launch directory
    bringup_dir = get_package_share_directory('rosbridge_server')
    launch_dir = os.path.join(bringup_dir, 'launch')

    # Create the launch configuration variables
    namespace = LaunchConfiguration('namespace')
    use_namespace = LaunchConfiguration('use_namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

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

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true')


    start_openbot_ros_bridge_cmd = Node(
        package = 'rosbridge_server',
        executable = 'openbot_node',
        parameters = [{'use_sim_time': True}],
        output = 'screen')
    
    include_launch_description = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(launch_dir, 'rosbridge_websocket_launch.xml')),
        launch_arguments={
            'namespace': 'your_namespace',
            'use_sim_time': 'true',
        }.items()
    )

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
    ld.add_action(declare_use_namespace_cmd)

    # Add any conditioned actions
    # ld.add_action(start_rviz_cmd)

    launch_service = LaunchService()
    ld.add_action(start_openbot_ros_bridge_cmd)
    launch_service.include_launch_description(include_launch_description)
    launch_service.run()

    return ld