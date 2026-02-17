#!/usr/bin/env python3
"""
SLAM + Navigation Launch for Mecca Robot
Manual exploration with joystick + autonomous navigation to goals set in RViz
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )
    
    # Get launch configurations
    use_sim_time = LaunchConfiguration('use_sim_time')
    
    # Package directories
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # SLAM Toolbox node
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'odom_frame': 'odom',
            'map_frame': 'map',
            'base_frame': 'base_link',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'debug_logging': False,
            'throttle_scans': 1,
            'transform_publish_period': 0.02,
            'map_update_interval': 5.0,
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.5,
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'enable_interactive_mode': True,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.5,
            'minimum_travel_heading': 0.5,
            'scan_buffer_size': 10,
            'do_loop_closing': True
        }]
    )
    
    # Nav2 navigation stack
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            nav2_bringup_dir, '/launch/navigation_launch.py'
        ]),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': 'true',
            'params_file': os.path.join(
                get_package_share_directory('mecca_launch'),
                'config',
                'nav2_params.yaml'
            )
        }.items()
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # SLAM for mapping
        slam_toolbox_node,
        
        # Navigation stack (with delay to let SLAM initialize)
        TimerAction(
            period=5.0,
            actions=[nav2_launch]
        ),
    ])
