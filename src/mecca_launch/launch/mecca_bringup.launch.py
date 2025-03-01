from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    return LaunchDescription([
        # Existing Nodes
        Node(
            package='mecca_driver_node',
            executable='mecca_driver_node',
            name='mecca_driver_node',
            output='screen',
            remappings=[
                ('/motor_command', '/serial_driver/input')],
        ),
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[
                os.path.join(
                    get_package_share_directory('mecca_driver_node'),
                    'config',
                    'joy_teleop.yaml'
                ),
                {'use_sim_time': False}   # Corrected to be a dictionary
            ]
        ),
        # New Serial Node
        Node(
            package='mecca_driver_node',
            executable='simple_serial',
            name='simple_serial_node',
            output='screen',
            parameters=[{
                'port': '/dev/stm32_serial',
                'baudrate': 115200
            }]
        )
    ])

