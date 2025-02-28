from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Existing Nodes
        Node(
            package='mecca_driver_node',
            executable='mecca_driver_node',
            name='mecca_driver_node',
            output='screen'
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
            parameters=[{'use_sim_time': False}]
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

