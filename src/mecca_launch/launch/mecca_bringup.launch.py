# mecca_bringup.launch.py
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Path to our teleop parameters YAML
    teleop_params = os.path.join(
        get_package_share_directory('mecca_driver'),
        'config',
        'joy_teleop.yaml'
    )

    return LaunchDescription([
        # Motor driver node
        Node(
            package='mecca_driver',
            executable='mecca_driver',
            name='mecca_driver',
            output='screen',
            remappings=[('/motor_command', '/serial_driver/input')],
        ),

        # Navigator (path‐planning) node
        Node(
            package='navigator',
            executable='navigator_node',
            name='navigator_node',
            output='screen',
        ),

        # Joystick input
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
        ),

        # Teleop-to-Twist converter
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

        # VL53L1X distance sensor
        Node(
            package='vl53l1x_sensor',
            executable='vl53l1x_node',
            name='vl53l1x_node',
            output='screen',
        ),

        # Simple serial transport node
        Node(
            package='mecca_driver',
            executable='simple_serial',
            name='simple_serial_node',
            output='screen',
            parameters=[{
                'port': '/dev/stm32_serial',
                'baudrate': 115200,
            }],
        ),

        # LED controller
        Node(
            package='mecca_driver',
            executable='led_controller_node',
            name='led_controller_node',
            output='screen',
        ),
    ])
