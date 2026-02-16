from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='mecca_launch',  # Replace with your actual package name
            executable='serial_bridge',
            name='mecca_serial_bridge',
            output='screen',
            parameters=[{
                'serial_port': '/dev/ttyUSB0',
                'baud_rate': 115200,
                'command_frequency': 20.0, # Target 20Hz for smoothness
            }],
            arguments=['--ros-args', '--log-level', 'info']
        )
    ])
