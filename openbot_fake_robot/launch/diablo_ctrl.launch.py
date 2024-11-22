from launch_ros.actions import Node
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition, UnlessCondition
import xacro
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    share_dir = get_package_share_directory('diablo_simulation')

    xacro_file = os.path.join(share_dir, 'urdf', 'diablo_simulation.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_urdf = robot_description_config.toxml()


    gui_arg = DeclareLaunchArgument(
        name='gui',
        default_value='True'
    )

    show_gui = LaunchConfiguration('gui')

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[
            {'robot_description': robot_urdf}
        ]
    )

    joint_state_publisher_node = Node(
        # condition=UnlessCondition(show_gui),
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher'
    )

    joint_state_publisher_gui_node = Node(
        # condition=IfCondition(show_gui),
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui'
    )

    joint_synchronization_node = Node(
        package='diablo_simpose_trans',
        executable='motor_trans_node',
        name='motor_trans_node'
    )

    diablo_ctrl_node = Node(
        package='diablo_ctrl',
        executable='diablo_ctrl',
        name='diablo_ctrl'
    )
    
    start_static_footprint_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "base_footprint", "base_link"],
    )
    
    
    param_dir = LaunchConfiguration(
        'param_dir',
        default=os.path.join(
            get_package_share_directory('openbot_fake_robot'),
            'param',
             'diablo.yaml')
        )

    return LaunchDescription([

        DeclareLaunchArgument(
            'param_dir',
            default_value=param_dir,
            description='Specifying parameter direction'),
        gui_arg,
        # joint_state_publisher_gui_node,
        robot_state_publisher_node,
        # joint_state_publisher_node,
        # joint_synchronization_node,
        start_static_footprint_transform,
        Node(
            package='openbot_fake_robot',
            executable='turtlebot3_fake_node',
            parameters=[param_dir],
            output='screen'),
        diablo_ctrl_node
        # Node(
        #     package='openbot_fake_robot',
        #     executable='diablo_fake_node.py',
        #     name='motion_controller_python',  # Node name
        #     parameters=[param_dir],
        # arguments=["--ros-args", "--log-level", "debug"],
        #     output='screen'),
    ])
