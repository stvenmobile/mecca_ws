from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
import os
import xacro

from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    mecca_driver_dir = get_package_share_directory('mecca_driver_node')
    mecca_desc_dir = get_package_share_directory('mecca_description')
    mecca_nav_dir = get_package_share_directory('navigator')

    # ✅ PROCESS the xacro file properly
    xacro_file = os.path.join(mecca_desc_dir, 'urdf', 'meccabot.xacro')
    robot_description_config = xacro.process_file(xacro_file)
    robot_description = {'robot_description': robot_description_config.toxml()}

    return LaunchDescription([
        # Robot State Publisher
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            output='screen',
            parameters=[robot_description],
        ),

        # Mecca Driver Node (handles encoder and cmd_vel I/O)
        Node(
            package='mecca_driver_node',
            executable='mecca_driver_node',
            name='mecca_driver_node',
            output='screen'
        ),

        # Navigator Node
        Node(
            package='navigator',
            executable='navigator_node',
            name='navigator_node',
            output='screen'
        ),

        # Joy Node
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),

        # Teleop Twist Joy Node
        Node(
            package='teleop_twist_joy',
            executable='teleop_node',
            name='teleop_twist_joy_node',
            parameters=[os.path.join(mecca_driver_dir, 'config', 'joy_teleop.yaml')],
            output='screen'
        ),

        # Simple Serial Node
        Node(
            package='mecca_driver_node',
            executable='simple_serial',
            name='simple_serial_node',
            parameters=[{
                'port': '/dev/stm32_serial',
                'baudrate': 115200
            }],
            output='screen'
        ),

        # LED Controller Node
        Node(
            package='mecca_driver_node',
            executable='led_controller_node',
            name='led_controller_node',
            output='screen'
        ),

        # VL53L1X Sensor Node
        Node(
            package='vl53l1x_sensor',
            executable='vl53l1x_node',
            name='vl53l1x_node',
            output='screen'
        ),

        # Controller Manager
        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[
                os.path.join(mecca_driver_dir, 'config', 'mecanum_controller.yaml'),
                {'robot_description': open(os.path.join(mecca_desc_dir, 'urdf', 'meccabot.xacro')).read()}
            ],
            output='screen'
        ),

        # Load Joint State Broadcaster
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen'
        ),

        # Load Mecanum Drive Controller
        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['mecanum_drive_controller'],
            output='screen'
        )
    ])
