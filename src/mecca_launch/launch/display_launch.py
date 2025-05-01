# display_launch.py
import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Locate XACRO and RViz config
    pkg_share = get_package_share_directory('mecca_description')
    xacro_file = os.path.join(pkg_share, 'urdf', 'meccabot.xacro')
    rviz_config_file = os.path.join(pkg_share, 'rviz', 'meccabot.rviz')

    # Process the XACRO into a RobotDescription XML string
    doc = xacro.process_file(xacro_file)
    robot_desc = doc.toxml()

    return LaunchDescription([
        # Publish robot_description and (optionally) simulation time
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[
                {'robot_description': robot_desc},
                {'use_sim_time': False},
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

        # Static transform publishers for each wheel
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fl_wheel_publisher',
            arguments=['0.0875', '0.1025', '-0.015', '0', '0', '0', 'chassis_link', 'front_left_wheel'],
            output='screen',
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='fr_wheel_publisher',
            arguments=['0.0875', '-0.1025', '-0.015', '0', '0', '0', 'chassis_link', 'front_right_wheel'],
            output='screen',
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rl_wheel_publisher',
            arguments=['-0.0875', '0.1025', '-0.015', '0', '0', '0', 'chassis_link', 'rear_left_wheel'],
            output='screen',
        ),
        
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='rr_wheel_publisher',
            arguments=['-0.0875', '-0.1025', '-0.015', '0', '0', '0', 'chassis_link', 'rear_right_wheel'],
            output='screen',
        ),
    ])
