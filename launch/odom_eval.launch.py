#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
Launch file for odometry evaluation node.
This launch file reads the parameter file and starts the odom_eval_node.
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Get package directory
    pkg_share = FindPackageShare('antrobot_ros')
    
    # Default parameter file path
    default_params_file = PathJoinSubstitution([
        pkg_share,
        'config',
        'odom_eval_params.yaml'
    ])
    
    return LaunchDescription([
        # Declare launch arguments
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params_file,
            description='Path to the parameter file for odometry evaluation'
        ),
        
        DeclareLaunchArgument(
            'output_dir',
            default_value='/tmp/odom_eval',
            description='Output directory for trajectory files (overrides param file)'
        ),
        
        DeclareLaunchArgument(
            'reference_frame',
            default_value='odom',
            description='Reference frame for trajectories (overrides param file)'
        ),
        
        # Odometry evaluation node
        Node(
            package='antrobot_ros',
            executable='odom_eval_node',
            name='odom_eval_node',
            output='screen',
            parameters=[
                LaunchConfiguration('params_file'),
                {
                    'output_dir': LaunchConfiguration('output_dir'),
                    'reference_frame': LaunchConfiguration('reference_frame'),
                }
            ],
            # Ensure clean shutdown on Ctrl+C
            emulate_tty=True,
        ),
    ])
