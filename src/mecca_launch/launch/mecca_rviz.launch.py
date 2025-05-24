#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value=os.path.join(
            get_package_share_directory('mecca_description'),
            'urdf',
            'meccabot.xacro'
        ),
        description='Path to URDF file to display'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # === PHYSICAL ROBOT NODES (from your mecca_launch.py) ===
    
    # Main driver node
    mecca_driver_node = Node(
        package='mecca_driver_node',
        executable='mecca_driver_node',
        name='mecca_driver_node',
        output='screen',
        remappings=[
            ('/motor_command', '/serial_driver/input')
        ],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Navigator node
    navigator_node = Node(
        package='navigator',
        executable='navigator_node',
        name='navigator_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Joystick node
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Teleop node
    teleop_node = Node(
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
            {'use_sim_time': use_sim_time}
        ]
    )
    
    # VL53L1X sensor node
    vl53l1x_node = Node(
        package='vl53l1x_sensor',
        executable='vl53l1x_node',
        name='vl53l1x_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # Serial communication node
    simple_serial_node = Node(
        package='mecca_driver_node',
        executable='simple_serial',
        name='simple_serial_node',
        output='screen',
        parameters=[{
            'port': '/dev/stm32_serial',
            'baudrate': 115200,
            'use_sim_time': use_sim_time
        }]
    )
    
    # LED controller node
    led_controller_node = Node(
        package='mecca_driver_node',
        executable='led_controller_node',
        name='led_controller_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # === ROBOT STATE AND VISUALIZATION NODES ===
    
    # Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }]
    )
    
    # Joint State Publisher (publishes wheel positions)
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'source_list': ['wheel_states'],  # Listen for wheel states from your driver
            'use_sim_time': use_sim_time
        }]
    )
    
    # === LIDAR NODE ===
    
    # LiDAR node (assuming you have the sllidar package configured)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'serial_port': '/dev/serial/by-id/usb-Silicon_Labs_CP2102_USB_to_UART_Bridge_Controller_*',  # Adjust as needed
            'frame_id': 'lidar_link',
            'inverted': False,
            'angle_compensate': True,
            'use_sim_time': use_sim_time
        }]
    )
    
    # === RVIZ NODE ===
    
    # RViz with navigation config
    rviz_config_file = os.path.join(
        get_package_share_directory('mecca_description'),
        'rviz',
        'robot_navigation.rviz'
    )
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        urdf_file_arg,
        
        # Physical robot nodes
        mecca_driver_node,
        navigator_node,
        joy_node,
        teleop_node,
        vl53l1x_node,
        simple_serial_node,
        led_controller_node,
        
        # Robot state nodes
        robot_state_publisher_node,
        joint_state_publisher_node,
        
        # Sensor nodes
        lidar_node,
        
        # Visualization
        rviz_node
    ])
