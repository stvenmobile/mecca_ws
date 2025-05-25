#!/usr/bin/env python3
"""
SLAM Only Launch for Mecca Robot
Just SLAM Toolbox for mapping - no navigation yet
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
    
    # SLAM Toolbox node (optimized for real-time performance)
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
            'transform_publish_period': 0.05,  # Faster transform updates (was 0.02)
            'map_update_interval': 2.0,        # Faster map updates (was 5.0)
            'resolution': 0.05,
            'max_laser_range': 12.0,
            'minimum_time_interval': 0.1,      # Faster processing (was 0.5)
            'transform_timeout': 0.5,          # More lenient timeout (was 0.2)
            'tf_buffer_duration': 30.0,
            'enable_interactive_mode': True,
            'use_scan_matching': True,
            'use_scan_barycenter': True,
            'minimum_travel_distance': 0.3,    # Less travel required (was 0.5)
            'minimum_travel_heading': 0.3,     # Less rotation required (was 0.5)
            'scan_buffer_size': 5,             # Smaller buffer (was 10)
            'do_loop_closing': True
        }]
    )
    
    return LaunchDescription([
        # Launch arguments
        use_sim_time_arg,
        
        # SLAM for mapping
        slam_toolbox_node,
    ])
