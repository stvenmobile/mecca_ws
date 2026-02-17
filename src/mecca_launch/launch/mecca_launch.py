#!/usr/bin/env python3
"""
Robot Launch File for Raspberry Pi
Launches all physical robot nodes and publishes robot state
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction
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
        description='Path to URDF file'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')
    
    # Robot description
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # === PHYSICAL ROBOT NODES ===
    
    # Main driver node (enhanced version with odometry)
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
    
    # Serial communication node (using stable device name)
    simple_serial_node = Node(
        package='mecca_driver_node',
        executable='simple_serial',
        name='simple_serial_node',
        output='screen',
        parameters=[{
            'port': '/dev/stm32_serial',  # Use stable symlink
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
    
    # Robot State Publisher (publishes robot_description and TF)
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )
    
    # === AUTONOMOUS NAVIGATION COMPONENTS ===
    
    # Static transform publisher (base_link -> laser frame transform)
    base_to_laser_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_to_laser_tf',
        arguments=['0', '0', '0.1', '0', '0', '0', 'base_link', 'laser'],
        parameters=[{'use_sim_time': use_sim_time}]
    )
    
    # LiDAR node (using stable device name - working!)
    lidar_node = Node(
        package='sllidar_ros2',
        executable='sllidar_node',
        name='sllidar_node',
        output='screen',
        parameters=[{
            'channel_type': 'serial',
            'serial_port': '/dev/lidar_serial',   # Use stable symlink
            'serial_baudrate': 115200,
            'frame_id': 'laser',
            'inverted': False,
            'angle_compensate': True,
            'scan_mode': 'Sensitivity',
            'use_sim_time': use_sim_time
        }],
        respawn=True,
        respawn_delay=5.0
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
        
        # Robot state publisher
        robot_state_publisher_node,
        
        # Transform publishers
        base_to_laser_tf,
        
        # Sensor nodes (with delay for LIDAR)
        TimerAction(
            period=5.0,  # 5 second delay for LIDAR
            actions=[lidar_node]
        ),
    ])
