#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
Odometry Evaluation Node for recording multiple odometry topics in TUM format.

This node reads a list of topics from parameters and records their trajectories
in TUM format compatible with evo (evaluation toolbox for odometry and SLAM).
"""

import os
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from tf2_ros import TransformListener, Buffer
import tf2_geometry_msgs
from datetime import datetime
import signal
import sys


class OdomEvalNode(Node):
    """Node for recording multiple odometry topics in TUM format."""
    
    def __init__(self):
        super().__init__('odom_eval_node')
        
        # Declare parameters
        self.declare_parameter('output_dir', '/tmp/odom_eval')
        self.declare_parameter('topics', [''])  # Default to empty string list
        self.declare_parameter('reference_frame', 'odom')
        
        # Get parameters
        self.output_dir = self.get_parameter('output_dir').get_parameter_value().string_value
        topics_param = self.get_parameter('topics').get_parameter_value().string_array_value
        # Filter out empty strings
        self.topics = [topic for topic in topics_param if topic.strip()]
        self.reference_frame = self.get_parameter('reference_frame').get_parameter_value().string_value
        
        # Validate topics
        if not self.topics:
            self.get_logger().error("No topics specified in parameters! Please check your parameter file.")
            raise ValueError("No topics specified for recording")
        
        # Create output directory
        os.makedirs(self.output_dir, exist_ok=True)
        
        # Generate timestamp for this session
        self.session_timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        # Initialize data storage - dict mapping topic name to data list
        self.trajectory_data = {}
        self.topic_files = {}  # mapping topic to filename
        
        # TF2 buffer and listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # QoS profile for reliable communication
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Create subscribers for each topic
        self.subscribers = {}
        for topic in self.topics:
            # Clean topic name for filename (remove slashes)
            clean_name = topic.replace('/', '_').strip('_')
            filename = f"{clean_name}_{self.session_timestamp}.tum"
            filepath = os.path.join(self.output_dir, filename)
            
            self.trajectory_data[topic] = []
            self.topic_files[topic] = filepath
            
            # Subscribe to the topic
            self.subscribers[topic] = self.create_subscription(
                Odometry,
                topic,
                lambda msg, topic=topic: self.odom_callback(msg, topic),
                qos_profile
            )
            
            self.get_logger().info(f"Subscribed to {topic} -> {filename}")
        
        # Set up signal handler for graceful shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        self.get_logger().info(f"Odometry evaluation node started")
        self.get_logger().info(f"Recording {len(self.topics)} topics")
        self.get_logger().info(f"Output directory: {self.output_dir}")
        self.get_logger().info(f"Session timestamp: {self.session_timestamp}")
        self.get_logger().info("Press Ctrl+C to stop recording and save trajectories")
    
    def odom_callback(self, msg: Odometry, topic: str):
        """Callback for odometry data from any topic."""
        try:
            # Transform pose to reference frame if needed
            if msg.header.frame_id != self.reference_frame:
                pose_stamped = PoseStamped()
                pose_stamped.header = msg.header
                pose_stamped.pose = msg.pose.pose
                
                # Transform to reference frame
                transformed_pose = self.tf_buffer.transform(
                    pose_stamped, 
                    self.reference_frame,
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                pose = transformed_pose.pose
            else:
                pose = msg.pose.pose
            
            timestamp = self.msg_to_timestamp(msg.header.stamp)
            
            # Convert to TUM format: timestamp x y z qx qy qz qw
            tum_line = f"{timestamp:.6f} {pose.position.x:.6f} {pose.position.y:.6f} {pose.position.z:.6f} " \
                      f"{pose.orientation.x:.6f} {pose.orientation.y:.6f} {pose.orientation.z:.6f} {pose.orientation.w:.6f}\n"
            
            self.trajectory_data[topic].append(tum_line)
            
        except Exception as e:
            self.get_logger().warn(f"Failed to transform {topic} odometry: {str(e)}")
            # Fallback: use original pose
            timestamp = self.msg_to_timestamp(msg.header.stamp)
            pose = msg.pose.pose
            
            tum_line = f"{timestamp:.6f} {pose.position.x:.6f} {pose.position.y:.6f} {pose.position.z:.6f} " \
                      f"{pose.orientation.x:.6f} {pose.orientation.y:.6f} {pose.orientation.z:.6f} {pose.orientation.w:.6f}\n"
            
            self.trajectory_data[topic].append(tum_line)
    
    def msg_to_timestamp(self, stamp):
        """Convert ROS2 timestamp to float seconds."""
        return float(stamp.sec) + float(stamp.nanosec) / 1e9
    
    def save_trajectories(self):
        """Save all trajectories to TUM format files."""
        saved_files = []
        for topic, data in self.trajectory_data.items():
            if len(data) > 0:
                filepath = self.topic_files[topic]
                with open(filepath, 'w') as f:
                    f.writelines(data)
                saved_files.append(filepath)
                self.get_logger().info(f"Saved {len(data)} poses from {topic} to {os.path.basename(filepath)}")
        
        if saved_files:
            self.get_logger().info(f"All trajectories saved to {self.output_dir}")
            return saved_files
        else:
            self.get_logger().warn("No trajectory data to save")
            return []
    
    def signal_handler(self, signum, frame):
        """Handle Ctrl+C gracefully."""
        self.get_logger().info("Received interrupt signal. Saving trajectories...")
        self.save_trajectories()
        self.get_logger().info("Trajectories saved. Exiting...")
        rclpy.shutdown()
        sys.exit(0)


def main(args=None):
    rclpy.init(args=args)
    
    node = OdomEvalNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Keyboard interrupt received. Saving trajectories...")
        node.save_trajectories()
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
