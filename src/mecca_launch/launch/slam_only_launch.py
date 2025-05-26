#!/usr/bin/env python3
"""
SLAM Only Launch for Mecca Robot
Just SLAM Toolbox for mapping - optimized for performance
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
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
    
    # SLAM Toolbox node (optimized for performance with larger buffers)
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
            'throttle_scans': 5,              # Process every 5th scan
            'transform_publish_period': 0.2,  # Slower transform updates
            'map_update_interval': 10.0,      # Slower map updates
            'resolution': 0.15,               # Lower resolution for speed
            'max_laser_range': 6.0,           # Shorter range
            'minimum_time_interval': 1.0,     # Slower processing
            'transform_timeout': 5.0,         # Much longer timeout
            'tf_buffer_duration': 60.0,       # Longer TF buffer
            'enable_interactive_mode': True,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 1.0,   # Require more movement
            'minimum_travel_heading': 1.0,    # Require more rotation
            'scan_buffer_size': 10,           # Larger scan buffer
            'scan_buffer_maximum_scan_distance': 3.0,  # Limit buffer distance
            'do_loop_closing': False,         # Disable for performance
            'autostart': True
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # SLAM for mapping
        slam_toolbox_node,
    ])
