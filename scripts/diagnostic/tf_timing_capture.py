#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
TF Timing and Pose Capture Tool

Monitors for TF timing issues like the ones you're experiencing and captures
the exact state of both odometry sources when these issues occur.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rcl_interfaces.msg import Log
import tf2_ros
import re
import math
from datetime import datetime


class TFTimingCapture(Node):
    def __init__(self):
        super().__init__('tf_timing_capture')
        
        # Subscribe to rosout to catch TF timing errors
        self.rosout_sub = self.create_subscription(
            Log, '/rosout', self.rosout_callback, 10)
        
        # Subscribe to odometry sources
        self.wheel_odom_sub = self.create_subscription(
            Odometry, 'odom_wheel', self.wheel_odom_callback, 10)
        self.icp_odom_sub = self.create_subscription(
            Odometry, 'odom', self.icp_odom_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, 10)
        
        # Store latest poses
        self.latest_wheel_odom = None
        self.latest_icp_odom = None
        self.latest_lidar = None
        
        # TF buffer
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # Patterns to detect timing issues
        self.tf_error_patterns = [
            r"Extrapolation Error looking up target frame",
            r"Message Filter dropping message",
            r"the timestamp on the message is earlier than all the data in the transform cache",
            r"Lookup would require extrapolation into the past"
        ]
        
        print("TF Timing Capture Tool Started")
        print("Monitoring for TF timing issues and capturing poses...")
        print("-" * 60)
        
    def wheel_odom_callback(self, msg):
        self.latest_wheel_odom = msg
        
    def icp_odom_callback(self, msg):
        self.latest_icp_odom = msg
        
    def lidar_callback(self, msg):
        self.latest_lidar = msg
        
    def rosout_callback(self, msg):
        """Monitor log messages for TF timing issues"""
        log_text = msg.msg
        
        # Check if this is a TF timing error
        for pattern in self.tf_error_patterns:
            if re.search(pattern, log_text):
                self.capture_timing_issue(msg, log_text)
                break
                
    def capture_timing_issue(self, log_msg, error_text):
        """Capture complete state when timing issue occurs"""
        timestamp = self.get_clock().now()
        
        print()
        print("=" * 80)
        print(f"TF TIMING ISSUE DETECTED - {timestamp}")
        print("=" * 80)
        print(f"Error: {error_text}")
        print(f"Node: {log_msg.name}")
        print(f"Level: {self.log_level_to_string(log_msg.level)}")
        
        # Extract timestamp from error message if possible
        time_match = re.search(r'time (\d+\.\d+)', error_text)
        if time_match:
            error_time = float(time_match.group(1))
            current_time = timestamp.nanoseconds / 1e9
            time_diff = current_time - error_time
            print(f"Error timestamp: {error_time:.6f}")
            print(f"Current time: {current_time:.6f}")
            print(f"Time difference: {time_diff:.6f}s")
        
        print()
        
        # Capture wheel odometry state
        if self.latest_wheel_odom:
            self.print_odometry_state("WHEEL ODOMETRY", self.latest_wheel_odom)
        else:
            print("WHEEL ODOMETRY: No data available")
            
        print()
        
        # Capture ICP odometry state
        if self.latest_icp_odom:
            self.print_odometry_state("ICP ODOMETRY", self.latest_icp_odom)
        else:
            print("ICP ODOMETRY: No data available")
            
        print()
        
        # Capture lidar state
        if self.latest_lidar:
            self.print_lidar_state()
        else:
            print("LIDAR: No data available")
            
        print()
        
        # Try to capture TF tree state
        self.capture_tf_tree_state()
        
        print("=" * 80)
        print()
        
    def print_odometry_state(self, name, odom_msg):
        """Print detailed odometry state"""
        pose = odom_msg.pose.pose
        twist = odom_msg.twist.twist
        
        yaw = self.quaternion_to_yaw(pose.orientation)
        
        print(f"{name}:")
        print(f"  Topic timestamp: {odom_msg.header.stamp.sec}.{odom_msg.header.stamp.nanosec:09d}")
        print(f"  Frame: {odom_msg.header.frame_id} -> {odom_msg.child_frame_id}")
        print(f"  Position: x={pose.position.x:.6f}, y={pose.position.y:.6f}, z={pose.position.z:.6f}")
        print(f"  Orientation: yaw={math.degrees(yaw):.3f}°")
        print(f"  Quaternion: [{pose.orientation.x:.6f}, {pose.orientation.y:.6f}, {pose.orientation.z:.6f}, {pose.orientation.w:.6f}]")
        print(f"  Linear vel: [{twist.linear.x:.6f}, {twist.linear.y:.6f}, {twist.linear.z:.6f}]")
        print(f"  Angular vel: [{twist.angular.x:.6f}, {twist.angular.y:.6f}, {twist.angular.z:.6f}]")
        
        # Check covariance for issues
        pose_cov = odom_msg.pose.covariance
        if any(abs(x) > 100 for x in pose_cov):
            print(f"  ⚠️  HIGH POSE COVARIANCE detected!")
            
    def print_lidar_state(self):
        """Print lidar state"""
        lidar_msg = self.latest_lidar
        valid_ranges = [r for r in lidar_msg.ranges if lidar_msg.range_min <= r <= lidar_msg.range_max]
        
        print(f"LIDAR:")
        print(f"  Timestamp: {lidar_msg.header.stamp.sec}.{lidar_msg.header.stamp.nanosec:09d}")
        print(f"  Frame: {lidar_msg.header.frame_id}")
        print(f"  Valid points: {len(valid_ranges)}/{len(lidar_msg.ranges)}")
        if valid_ranges:
            print(f"  Range stats: min={min(valid_ranges):.2f}m, max={max(valid_ranges):.2f}m, avg={sum(valid_ranges)/len(valid_ranges):.2f}m")
        
    def capture_tf_tree_state(self):
        """Capture current TF tree state"""
        print("TF TREE STATE:")
        
        critical_transforms = [
            ('map', 'base_link'),
            ('odom', 'base_link'),
            ('odom_wheel', 'base_link'),
            ('base_link', 'rplidar_link'),
            ('odom', 'odom_wheel')
        ]
        
        for parent, child in critical_transforms:
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.01)
                )
                
                tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                current_time = self.get_clock().now()
                age = (current_time - tf_time).nanoseconds / 1e9
                
                print(f"  {parent} -> {child}:")
                print(f"    Translation: [{transform.transform.translation.x:.6f}, {transform.transform.translation.y:.6f}, {transform.transform.translation.z:.6f}]")
                print(f"    Rotation: [{transform.transform.rotation.x:.6f}, {transform.transform.rotation.y:.6f}, {transform.transform.rotation.z:.6f}, {transform.transform.rotation.w:.6f}]")
                print(f"    Age: {age:.6f}s")
                
                if age > 0.5:
                    print(f"    ⚠️  STALE TRANSFORM (>{age:.3f}s old)")
                    
            except tf2_ros.ExtrapolationException as e:
                print(f"  {parent} -> {child}: EXTRAPOLATION ERROR - {str(e)}")
            except tf2_ros.LookupException as e:
                print(f"  {parent} -> {child}: LOOKUP ERROR - {str(e)}")
            except Exception as e:
                print(f"  {parent} -> {child}: ERROR - {str(e)}")
                
    def quaternion_to_yaw(self, q):
        """Convert quaternion to yaw angle"""
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def log_level_to_string(self, level):
        """Convert log level to string"""
        levels = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}
        return levels.get(level, f"UNKNOWN({level})")


def main():
    rclpy.init()
    
    try:
        node = TFTimingCapture()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nTF Timing Capture stopped.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
