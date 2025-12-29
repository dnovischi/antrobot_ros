#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
Launch file for explore_lite with map converter

This launch file starts both:
1. Map converter node (antrobot_ros) - converts probabilistic maps to discrete maps
2. Explore lite node (m-explore-ros2) - performs frontier-based exploration

When launched standalone: enable_explore_lite defaults to true
When launched from pipeline: controlled by pipeline parameters
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    enable_explore_lite_arg = DeclareLaunchArgument(
        'enable_explore_lite',
        default_value='true',  # Default true when launching separately
        description='Enable explore lite mode with map conversion'
    )
    
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )
    
    # Load parameters using the /** pattern (compatible with m-explore-ros2)
    explore_lite_config_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'explore_lite_params.yaml'
    )
    
    # Map converter node (antrobot_ros)
    map_converter_node = Node(
        package='antrobot_ros',
        executable='explore_lite_map_converter',
        namespace=LaunchConfiguration('namespace'),
        name='explore_lite_map_converter',
        parameters=[
            explore_lite_config_path,
            {
                'enable_explore_lite': LaunchConfiguration('enable_explore_lite'),
            }
        ],
        output='screen',
        emulate_tty=True,
    )
    
    # Explore lite node (m-explore-ros2)
    explore_node = Node(
        package='explore_lite',
        executable='explore',
        namespace=LaunchConfiguration('namespace'),
        name='explore',
        parameters=[
            explore_lite_config_path,
        ],
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        enable_explore_lite_arg,
        namespace_arg,
        map_converter_node,
        explore_node,
    ])
