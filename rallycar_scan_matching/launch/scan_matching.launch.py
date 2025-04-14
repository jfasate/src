#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """
    Launch file for scan matching localization with the rallycar.
    This integrates with the existing rallycar hardware drivers.
    """
    
    # Parameters that can be set from the command line
    use_rallycar_hardware = LaunchConfiguration('use_rallycar_hardware')
    use_rviz = LaunchConfiguration('use_rviz')
    
    # Define launch arguments
    declare_use_rallycar_hardware = DeclareLaunchArgument(
        'use_rallycar_hardware',
        default_value='false',
        description='Whether to launch the rallycar hardware components'
    )
    
    declare_use_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Whether to launch RViz for visualization'
    )
    
    # Get package directories
    rallycar_scan_matching_dir = get_package_share_directory('rallycar_scan_matching')
    rallycar_dir = get_package_share_directory('rallycar')
    
    # Include the rallycar hardware launch if requested
    rallycar_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("rallycar"),
                "launch",
                "rallycar_hardware.launch.py",
            )
        )
    )
    
    # Configure the scan matching node
    scan_matching_node = Node(
        package='rallycar_scan_matching',
        executable='scanmatch_node',
        name='scanmatch_node',
        output='screen',
        parameters=[
            {
                'publish_tf': True,
                'use_fast_correspondence': True,
                'min_scan_distance': 0.2,
                'point_cloud_visualization': True
            }
        ]
    )
    
    # Configure RViz for visualization
    rviz_config_file = os.path.join(rallycar_scan_matching_dir, 'rviz', 'scan_matching.rviz')
    # Create a default config if it doesn't exist yet
    if not os.path.exists(rviz_config_file):
        rviz_config_file = ''
        
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_file] if rviz_config_file else [],
        condition=IfCondition(use_rviz)
    )
    
    return LaunchDescription([
        # Launch arguments
        declare_use_rallycar_hardware,
        declare_use_rviz,
        
        # Nodes and includes
        rallycar_hardware,
        scan_matching_node,
        rviz_node
    ])
