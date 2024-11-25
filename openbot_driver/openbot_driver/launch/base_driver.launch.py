#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config = os.path.join(
        get_package_share_directory('openbot_driver'),
        'param',
        'chassis_param.yaml'
        )
    return LaunchDescription([
        Node(
            package='openbot_driver',
            executable='openbot_bringup',
            name='openbot_bringup_node',
            parameters=[config]
        )
    ])
