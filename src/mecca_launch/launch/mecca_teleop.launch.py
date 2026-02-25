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
                'device_filename': '/dev/input/js0', 
                'deadzone': 0.05,
                'autorepeat_rate': 20.0,
            }]
        ),

        # teleop_twist_joy node for converting joystick inputs to Twist messages
        Node(
            package='teleop_twist_joy',
            executable='teleop_node', 
            name='teleop_twist_joy_node',
            output='screen',
            parameters=[joy_teleop_config],
            remappings=[
                # Updated to match the stamped input expected by the mecanum controller
                ('/cmd_vel', '/mecanum_drive_controller/cmd_vel') 
            ]
        ),
    ])