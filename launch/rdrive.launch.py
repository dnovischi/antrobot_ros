# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from antrobot_ros.utils import load_node_params


def generate_launch_description():
    robot_namespace_arg = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Namespace for the robot instance'
    )
 
    config_file_path = os.path.join(
        get_package_share_directory('antrobot_ros'),
        'config',
        'antrobot_params.yaml'
    )
    
    launch_params = load_node_params(config_file_path, 'rdrive')
    rdrive_params = {
        'wheel_radius': float(launch_params['wheel_radius']),
        'wheel_separation': float(launch_params['wheel_separation']),
        'encoder_cpr_left': int(launch_params['encoder_cpr_left']),
        'encoder_cpr_right': int(launch_params['encoder_cpr_right']),
        'no_load_rpm_left': float(launch_params['no_load_rpm_left']),
        'no_load_rpm_right': float(launch_params['no_load_rpm_right']),
        # Odometry and TF parameters
        'odom_frequency': float(launch_params['odom_frequency']),
        'odom_topic': launch_params['odom_topic'],
        'publish_tf': bool(launch_params['publish_tf']),
        'odom_frame_id': launch_params['odom_frame_id'],
        'base_frame_id': launch_params['base_frame_id']
    }
    
    # Create the rdrive node
    rdrive_node = Node(
        package='antrobot_ros',
        executable='rdrive_node',
        namespace=LaunchConfiguration('namespace'),
        name='rdrive_node',
        parameters=[rdrive_params]
    )
    
    return LaunchDescription([robot_namespace_arg, rdrive_node])
