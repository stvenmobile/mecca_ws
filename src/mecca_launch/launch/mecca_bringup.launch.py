# mecca_bringup.launch.py

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue
from launch.substitutions import Command, PathJoinSubstitution


def generate_launch_description():
    # Path to joy teleop config
    teleop_params = os.path.join(
        get_package_share_directory('mecca_driver_node'),
        'config',
        'joy_teleop.yaml'
    )

    # Path to the robot xacro file
    urdf_file = PathJoinSubstitution([
        get_package_share_directory('mecca_description'),
        'urdf',
        'meccabot.xacro'
    ])

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': ParameterValue(
                    Command(['xacro ', urdf_file]),
                    value_type=str
                ),
                'use_sim_time': False,
            }],
        ),

        # Motor driver node (includes encoder polling and joint_state publisher)
        Node(
            package='mecca_driver_node',
            executable='mecca_driver_node',
            name='mecca_driver_node',
            output='screen',
            remappings=[('/motor_command', '/serial_driver/input')],
        ),

        # Navigator (path-planning logic)
        Node(
            package='navigator',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
        ),

        # Joystick reader node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Teleop twist converter
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                teleop_params,
                {'use_sim_time': False},
            ],
        ),

        # ToF distance sensor (VL53L1X)
        Node(
            package='vl53l1x_sensor',
            executable='vl53l1x_node',
            name='vl53l1x_node',
            output='screen',
        ),

        # Serial transport node (to STM32)
        Node(
            package='mecca_driver_node',
            executable='simple_serial',
            name='simple_serial_node',
            output='screen',
            parameters=[{
                'port': '/dev/stm32_serial',
                'baudrate': 115200,
            }],
        ),

        # LED controller node
        Node(
            package='mecca_driver_node',
            executable='led_controller_node',
            name='led_controller_node',
            output='screen',
        ),
    ])
