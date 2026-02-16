#!/usr/bin/env python3
"""
Development Workstation Launch File for Elite PC
Launches RViz and visualization tools for remote robot monitoring
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.90',  # Your robot's IP
        description='IP address of the robot'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    urdf_file = LaunchConfiguration('urdf_file')
    
    # Robot description (for local RViz display)
    robot_description_content = ParameterValue(
        Command(['xacro ', urdf_file]),
        value_type=str
    )
    
    # === VISUALIZATION NODES ===
    
    # Robot State Publisher (local copy for RViz)
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
    
    # === OPTIONAL: DEVELOPMENT TOOLS ===
    
    # rqt for debugging (uncomment if needed)
    # rqt_node = Node(
    #     package='rqt_gui',
    #     executable='rqt_gui',
    #     name='rqt_gui',
    #     output='screen'
    # )
    
    # PlotJuggler for data visualization (uncomment if installed)
    # plotjuggler_node = Node(
    #     package='plotjuggler',
    #     executable='plotjuggler',
    #     name='plotjuggler',
    #     output='screen'
    # )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        urdf_file_arg,
        robot_ip_arg,
        
        # Visualization nodes
        robot_state_publisher_node,
        rviz_node,
        
        # Development tools (uncomment as needed)
        # rqt_node,
        # plotjuggler_node,
    ])
