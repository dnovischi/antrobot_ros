# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )
    
    # Parameters for jump detection thresholds
    max_linear_jump_arg = DeclareLaunchArgument(
        'max_linear_jump',
        default_value='0.5',
        description='Maximum allowed position jump in meters'
    )
    
    max_angular_jump_arg = DeclareLaunchArgument(
        'max_angular_jump',
        default_value='0.5',
        description='Maximum allowed angular jump in radians'
    )
    
    max_velocity_jump_arg = DeclareLaunchArgument(
        'max_velocity_jump',
        default_value='2.0',
        description='Maximum allowed velocity change in m/s'
    )
    
    odom_monitor_node = Node(
        package='antrobot_ros',
        executable='odom_monitor',
        namespace=LaunchConfiguration('namespace'),
        name='odom_monitor',
        output='screen',
        parameters=[{
            'wheel_odom_topic': 'odom_wheel',
            'icp_odom_topic': 'odom',
            'lidar_topic': 'scan',
            'max_linear_jump': LaunchConfiguration('max_linear_jump'),
            'max_angular_jump': LaunchConfiguration('max_angular_jump'),
            'max_velocity_jump': LaunchConfiguration('max_velocity_jump'),
            'max_cross_odom_diff': 1.0,
            'min_lidar_points': 50,
            'max_lidar_gap_time': 0.2
        }]
    )
    
    return LaunchDescription([
        namespace_arg,
        max_linear_jump_arg,
        max_angular_jump_arg,
        max_velocity_jump_arg,
        odom_monitor_node
    ])
