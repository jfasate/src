#!/usr/bin/env python3

import os
from datetime import datetime
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    High-performance scan matching system for autonomous racing with comprehensive data collection.
    Optimized for Nvidia Jetson Orin Nano with Hokuyo UST-10LX LiDAR.
    Designed to work alongside existing control systems (wall following, etc.)
    """
    
    # Launch arguments
    use_rallycar_hardware = LaunchConfiguration('use_rallycar_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    record_data = LaunchConfiguration('record_data')
    
    # Define launch arguments
    declare_use_rallycar_hardware = DeclareLaunchArgument(
        'use_rallycar_hardware',
        default_value='true',
        description='Launch rallycar hardware components'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz for real-time visualization'
    )
    
    declare_record_data = DeclareLaunchArgument(
        'record_data',
        default_value='true',
        description='Record comprehensive data for analysis'
    )
    
    # Get package directories
    rallycar_scan_matching_dir = get_package_share_directory('rallycar_scan_matching')
    
    # Create timestamp for data collection
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    
    # Include rallycar hardware
    rallycar_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rallycar"),
                "launch",
                "rallycar_hardware.launch.py",
            )
        ),
        condition=IfCondition(use_rallycar_hardware)
    )
    
    # High-performance scan matching node optimized for racing
    scan_matching_node = Node(
        package='rallycar_scan_matching',
        executable='scanmatch_node',
        name='scanmatch_node',
        output='screen',
        parameters=[
            {
                # Core algorithm parameters optimized for performance
                'publish_tf': True,
                'use_fast_correspondence': True,
                'min_scan_distance': 0.1,           # Sensitive for racing environments
                'point_cloud_visualization': True,
                
                # Performance optimizations for Jetson Orin Nano + UST-10LX (40Hz)
                'max_iterations': 8,                # Reduced for speed while maintaining accuracy
                'correspondence_threshold': 0.4,    # Maximum correspondence distance
                'downsample_factor': 2,             # Process every 2nd point for speed
                'frame_skip_max': 2,                # Skip frames if processing is too slow
                
                # UST-10LX specific parameters
                'motion_threshold': 0.003,          # Sensitive motion detection
                'range_limit': 10.0,                # UST-10LX maximum range
                'range_min': 0.06,                  # UST-10LX minimum range
                'angular_resolution': 0.0044,       # UST-10LX 0.25 degree resolution
                
                # Covariance tuning for racing/testing
                'position_covariance': 0.01,        # Position uncertainty
                'orientation_covariance': 0.02,     # Orientation uncertainty
                'velocity_covariance': 0.1,         # Velocity uncertainty
            }
        ]
        # Removed problematic nice prefix for now
    )
    
    # RViz with scan matching visualization (keep your existing config)
    rviz_config_file = os.path.join(rallycar_scan_matching_dir, 'rviz', 'scan_matching.rviz')
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        condition=IfCondition(use_rviz)
    )
    
    # Comprehensive data recording for localization analysis (record everything)
    rosbag_record = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'record', '-o', f"/home/nvidia-car9/rallycar_ws/rallycar_data_{timestamp}", '-a'
        ],
        condition=IfCondition(record_data),
        output='screen'
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_rallycar_hardware,
        declare_use_rviz,
        declare_record_data,
        
        # Core system nodes
        rallycar_hardware,
        scan_matching_node,
        
        # Visualization and data collection
        rviz_node,
        rosbag_record,
    ])
