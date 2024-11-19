import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    namespace = LaunchConfiguration('namespace')
    use_sim_time = LaunchConfiguration('use_sim_time')

    params_file = os.path.join(
        get_package_share_directory('jdbot_navigation'),
        'params',
        'behaviour_param.yaml')
    
    return LaunchDescription([

        DeclareLaunchArgument(
            'namespace', default_value='',
            description='Top-level namespace'),

        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='octomap_server',
            executable='octomap_server_node',
            parameters=[{
                'resolution': 0.05,
                'sensor_model.max_range': 3.0,
                'point_cloud_min_z': -0.5,
                'point_cloud_max_z': 1.5,
                'filter_ground_plane': True,
                'frame_id': '/map'
            }],
            remappings=[
                ('/cloud_in', '/camera/depth/color/points')
            ],
            output='screen'
        )

    ])
