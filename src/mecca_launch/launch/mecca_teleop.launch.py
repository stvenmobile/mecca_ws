from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    mecca_driver_node_share_dir = get_package_share_directory('mecca_driver_node')
    joy_teleop_config = os.path.join(mecca_driver_node_share_dir, 'config', 'joy_teleop.yaml')

    return LaunchDescription([
        # Joy node for reading joystick inputs
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen',
            parameters=[{
                'dev': '/dev/input/js0', # Default joystick device. Adjust if needed.
                'deadzone': 0.05,        # Small deadzone to prevent drift. Adjust as needed.
                'autorepeat_rate': 20.0, # Rate at which joystick messages are repeated
            }]
        ),

        # teleop_twist_joy node for converting joystick inputs to Twist messages
        Node(
            package='teleop_twist_joy',
            executable='teleop_node', # The executable for teleop_twist_joy
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_teleop_config],
            remappings=[
                ('/cmd_vel', '/mecanum_drive_controller/cmd_vel_unstamped') # Assuming the mecanum controller expects cmd_vel_unstamped
            ]
        ),
    ])
