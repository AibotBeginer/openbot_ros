import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='surfel_fusion',
            executable='surfel_map_node',
            name='surfel_map_node',
            output='screen',
            parameters=[
                {'cam_width': 640},
                {'cam_height': 480},
                {'cam_fx': 223.19148254},
                {'cam_cx': 159.8085022},
                {'cam_fy': 279.78665161},
                {'cam_cy': 109.63733673},
                {'save_name': ''},
                {'fuse_far_distance': 3.0},
                {'fuse_near_distance': 0.5},
                {'drift_free_poses': 300},
            ]
        )
    ])