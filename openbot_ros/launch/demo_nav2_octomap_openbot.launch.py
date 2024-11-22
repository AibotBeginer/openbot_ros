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
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import Shutdown
import os

import os
import tempfile

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnShutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParameter, SetRemap
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import HasNodeParams, RewrittenYaml

from launch_ros.actions import Node
from launch.actions import TimerAction, ExecuteProcess


def generate_launch_description():
    # Get the launch directory
    nav2_bringup_dir = get_package_share_directory("nav2_bringup")
    nav2_launch_dir = os.path.join(nav2_bringup_dir, "launch")
    openbot_ros_dir = FindPackageShare("openbot_ros").find("openbot_ros")
    sim_dir = get_package_share_directory("openbot_fake_robot")

    # Create the launch configuration variables
    namespace = LaunchConfiguration("namespace")
    use_sim_time = LaunchConfiguration("use_sim_time")
    params_file = LaunchConfiguration("params_file")
    autostart = LaunchConfiguration("autostart")
    use_respawn = LaunchConfiguration("use_respawn")
    
    remappings = [("/tf", "tf"), ("/tf_static", "tf_static")]

    # Declare the launch arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        "namespace", default_value="", description="Top-level namespace"
    )

    declare_use_namespace_cmd = DeclareLaunchArgument(
        "use_namespace",
        default_value="false",
        description="Whether to apply a namespace to the navigation stack",
    )

    declare_use_sim_time_cmd = DeclareLaunchArgument(
        "use_sim_time",
        default_value="true",
        description="Use simulation (Gazebo) clock if true",
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        "params_file",
        default_value=os.path.join(sim_dir, "param", "nav2_params.yaml"),
        description="Full path to the ROS2 parameters file to use for all launched nodes",
    )

    declare_autostart_cmd = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack",
    )

    declare_use_composition_cmd = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup",
    )

    declare_use_respawn_cmd = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )


    start_navigation_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_launch_dir, "navigation_launch.py")
        ),
        launch_arguments={
            "namespace": namespace,
            "use_sim_time": use_sim_time,
            "autostart": "True",
            "params_file": params_file,
            "use_composition": "False",
            "use_respawn": use_respawn,
            "log_level": "info"
        }.items()
    )
    
    # In ROS 2, when launching a node, arguments should be passed as strings
    start_static_map_odom_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["-0.5", "0.5", "0", "0", "0", "0", "map", "odom"],
    )
    
    ## robot
    
    start_fake_turtlebot_waffle_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_dir,
                "launch",
                "turtlebot3_fake_node.launch.py",
            )
        )
    )
    
    start_fake_turtlebot_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        on_exit=Shutdown(),
        arguments=[
            "-d",
            os.path.join(openbot_ros_dir, "rviz/nav2_octomap_openbot.rviz"),
        ],
        parameters=[{"use_sim_time": True}]
    )
    
    
    start_diablo_simulator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                sim_dir,
                "launch",
                "diablo_ctrl.launch.py",
            )
        )
    )
    
    start_diablo_rviz_cmd = Node(
        package="rviz2",
        executable="rviz2",
        on_exit=Shutdown(),
        arguments=[
            "-d",
            os.path.join(openbot_ros_dir, "rviz/nav2_octomap_diablo.rviz"),
        ],
        parameters=[{"use_sim_time": True}]
    )
    
    ## maps
    start_mockamap_cmd = Node(
        package="openbot_ros",
        executable="mockamap_generator_node",
        parameters=[
            {"use_sim_time": True},
            {"seed": 511},
            {"resolution": 0.1},
            {"x_length": 15},
            {"y_length": 15},
            {"z_length": 1},
            {"type": 1},
            # {"type": 3},
            {"complexity": 0.03},
            {"fill": 0.3},
            {"fractal": 1},
            {"attenuation": 0.1},
            {"width_min": 0.6},
            {"width_max": 1.5},
            {"obstacle_number": 30},
            {"road_width": 0.5},
            {"add_wall_x": 0},
            {"add_wall_y": 1},
            {"maze_type": 1},
            {"num_nodes": 40},
            {"connectivity": 0.8},
            {"node_rad": 1},
            {"road_rad": 10},
        ],
        output="screen",
    )
    
    start_mockamap_to_occupancy_grid_converter_cmd = Node(
        package="openbot_ros",
        executable="pointcloud_to_occupancy_grid_node",
        parameters=[
        ],
        output="screen",
    )
    
    
    start_mockamap_to_occupancy_grid_converter_cmd = Node(
            package='openbot_octomap_server',
            executable='octomap_server_node',
            parameters=[{
                'resolution': 0.05,
                'sensor_model.max_range': 3.0,
                'frame_id': 'map',
                'base_frame_id': 'base_link',
                'use_height_map': True,
                'colored_map': False, #  You enabled both height map and RGB color registration. This is contradictory. Defaulting to height map.
                'occupancy_min_z': -5.0,
                'occupancy_max_z': 5.0,
                # 'filter_ground_plane': True, # crashes
                # 'incremental_2D_projection': True
            }],
        arguments=["--ros-args", "--log-level", "info"],
            remappings=[
                ('/cloud_in', '/camera_point_cloud'),
                ('/projected_map', '/map'),
            ],
            output='screen'
        )
    
    
    start_mockamap_camera_transformer_cmd =    Node(
            package='openbot_ros',
            executable='pointcloud_to_occupancy_grid_node',
            parameters=[{
                'base_frame_id': 'base_link',
            }],
            arguments=["--ros-args", "--log-level", "info"],
            remappings=[
                ('/cloud_in', '/global_map'),
                ('/cloud_out', '/camera_point_cloud'),
            ],
            output='screen'
        )
    
    
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

    
    # nav2 map

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )
    
    start_nav2_map_server_cmd = Node(
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        respawn=use_respawn,
        respawn_delay=2.0,
        parameters=[configured_params, {'yaml_filename': os.path.join(sim_dir, 'maps', 'warehouse.yaml')}],
        arguments=["--ros-args", "--log-level", "info"],
        remappings=remappings,
    )
    
    # https://answers.ros.org/question/398094/ros2-nav2-map_server-problems-loading-map-with-nav2_map_server/
    lifecycle_nodes = ['map_server']
    use_sim_time = True
    autostart = True

    start_lifecycle_manager_cmd = Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_map_server',
            output='screen',
            emulate_tty=True,  # https://github.com/ros2/launch/issues/188
            parameters=[{'use_sim_time': use_sim_time},
                        {'autostart': autostart},
                        {'node_names': lifecycle_nodes}])
    

    # Create the launch description and populate
    ld = LaunchDescription()

    # Declare the launch options
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_use_namespace_cmd)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(declare_autostart_cmd)
    ld.add_action(declare_use_composition_cmd)

    ld.add_action(declare_use_respawn_cmd)

    # Add the actions to launch all of the navigation nodes
    ld.add_action(start_navigation_cmd)
    ld.add_action(start_static_map_odom_transform)
    
    # rviz
    
    # maps
    # ld.add_action(start_nav2_map_server_cmd)
    # ld.add_action(start_lifecycle_manager_cmd)
    ld.add_action(start_mockamap_cmd)
    ld.add_action(start_mockamap_to_occupancy_grid_converter_cmd)
    ld.add_action(start_mockamap_camera_transformer_cmd)
    
    # global planner
    ld.add_action(start_openbot_cmd)
    
    ## robots
    # ld.add_action(start_fake_turtlebot_waffle_simulator)
    # ld.add_action(start_fake_turtlebot_rviz_cmd)
    
    ld.add_action(start_diablo_simulator)
    ld.add_action(start_diablo_rviz_cmd)


    # Add any conditioned actions
    
    # The issue is that ros2 topic pub publishes the command as a latched publisher by default when invoked without the --once flag. This means the last message sent will continue to be available on the topi
    # ld.add_action(ExecuteProcess(
    #     cmd=[
    #         'bash', '-c', '''
    #         sleep 2 && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 2.14159}}'
    #         && sleep 0.5 && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 0}} && sleep infinity
    #         '''
    #     ],
    #     output='screen'
    # ))
    # ld.add_action(ExecuteProcess(
    #     cmd=[
    #         'bash', '-c', '''
    #         sleep 2 && ros2 topic pub /cmd_vel geometry_msgs/msg/Twist '{"linear": {"x": 0.0, "y": 0.0, "z": 0.0}, "angular": {"x": 0.0, "y": 0.0, "z": 6.14159}}' && sleep infinity
    #         '''
    #     ],
    #     output='screen'
    # ))
    
  
      
    return ld
