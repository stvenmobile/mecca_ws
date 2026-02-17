#!/usr/bin/env python3
# teleop_rviz.launch.py
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Locate XACRO and RViz config
    pkg_share = get_package_share_directory('mecca_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'meccabot.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'meccabot.rviz')

    # Process the XACRO into a RobotDescription XML string
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    # Declare use_sim_time argument
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time if true'
    )

    return LaunchDescription([
        use_sim_time_arg,
        
        # Publish robot_description
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': LaunchConfiguration('use_sim_time')},
            ],
        ),

        # Start RViz with our custom config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            arguments=['-d', rviz_config_file],
        ),
        
        # Joint State Publisher GUI (allows manual control of joints)
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        ),
        
        # Simple wheel controller to convert twist to joint movement
        Node(
            package='mecca_launch',
            executable='simple_wheel_controller.py',
            name='simple_wheel_controller',
            output='screen',
            parameters=[{'wheel_radius': 0.05, 'wheel_separation': 0.145}],
        ),
    ])