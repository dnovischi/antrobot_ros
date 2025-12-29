#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
Quick command-line odometry monitoring tool

Simple tool to monitor odometry topics and immediately detect jumps
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import math
import argparse


class QuickOdomMonitor(Node):
    def __init__(self, wheel_topic='odom_wheel', icp_topic='odom', lidar_topic='scan'):
        super().__init__('quick_odom_monitor')
        
        self.wheel_topic = wheel_topic
        self.icp_topic = icp_topic
        self.lidar_topic = lidar_topic
        
        # Previous poses
        self.prev_wheel = None
        self.prev_icp = None
        self.last_lidar_time = None
        
        # Store latest poses for cross-comparison
        self.latest_wheel = None
        self.latest_icp = None
        
        # Subscribers
        self.wheel_sub = self.create_subscription(
            Odometry, wheel_topic, self.wheel_callback, 10)
        self.icp_sub = self.create_subscription(
            Odometry, icp_topic, self.icp_callback, 10)
        self.lidar_sub = self.create_subscription(
            LaserScan, lidar_topic, self.lidar_callback, 10)
        
        print(f"Monitoring:")
        print(f"  Wheel odom: {wheel_topic}")
        print(f"  ICP odom:   {icp_topic}")
        print(f"  Lidar:      {lidar_topic}")
        print("Press Ctrl+C to stop")
        print("-" * 50)
        
    def wheel_callback(self, msg):
        self.latest_wheel = msg
        self.check_odom_jump(msg, self.prev_wheel, "WHEEL")
        self.prev_wheel = msg
        
    def icp_callback(self, msg):
        self.latest_icp = msg
        self.check_odom_jump(msg, self.prev_icp, "ICP")
        self.prev_icp = msg
        
    def lidar_callback(self, msg):
        current_time = self.get_clock().now()
        valid_points = sum(1 for r in msg.ranges if msg.range_min <= r <= msg.range_max)
        
        if self.last_lidar_time is not None:
            dt = (current_time - self.last_lidar_time).nanoseconds / 1e9
            rate = 1.0 / dt if dt > 0 else 0
            
            if valid_points < 50:
                print(f"⚠️  LIDAR: Low point count {valid_points} (rate: {rate:.1f}Hz)")
            elif rate < 5:
                print(f"⚠️  LIDAR: Low scan rate {rate:.1f}Hz")
                
        self.last_lidar_time = current_time
        
    def check_odom_jump(self, current, previous, source):
        if previous is None:
            return
            
        # Calculate position jump
        dx = current.pose.pose.position.x - previous.pose.pose.position.x
        dy = current.pose.pose.position.y - previous.pose.pose.position.y
        pos_jump = math.sqrt(dx*dx + dy*dy)
        
        # Calculate angular jump
        curr_yaw = self.quaternion_to_yaw(current.pose.pose.orientation)
        prev_yaw = self.quaternion_to_yaw(previous.pose.pose.orientation)
        ang_jump = abs(self.normalize_angle(curr_yaw - prev_yaw))
        
        # Calculate time difference
        dt = (current.header.stamp.sec - previous.header.stamp.sec) + \
             (current.header.stamp.nanosec - previous.header.stamp.nanosec) * 1e-9
        
        vel_jump = pos_jump / dt if dt > 0 else float('inf')
        
        # Check thresholds
        if pos_jump > 0.5:  # 50cm
            print(f"🚨 {source} POSITION JUMP: {pos_jump:.3f}m")
            print(f"   Previous: ({previous.pose.pose.position.x:.3f}, {previous.pose.pose.position.y:.3f}, {math.degrees(self.quaternion_to_yaw(previous.pose.pose.orientation)):.1f}°)")
            print(f"   Current:  ({current.pose.pose.position.x:.3f}, {current.pose.pose.position.y:.3f}, {math.degrees(self.quaternion_to_yaw(current.pose.pose.orientation)):.1f}°)")
            print(f"   Time diff: {dt:.3f}s")
            
            # Show the other odometry source for comparison
            other_msg = self.latest_icp if source == "WHEEL" else self.latest_wheel
            other_name = "ICP" if source == "WHEEL" else "WHEEL"
            if other_msg:
                print(f"   {other_name} at same time: ({other_msg.pose.pose.position.x:.3f}, {other_msg.pose.pose.position.y:.3f}, {math.degrees(self.quaternion_to_yaw(other_msg.pose.pose.orientation)):.1f}°)")
                
                # Calculate difference between sources
                dx_cross = current.pose.pose.position.x - other_msg.pose.pose.position.x
                dy_cross = current.pose.pose.position.y - other_msg.pose.pose.position.y
                cross_diff = math.sqrt(dx_cross*dx_cross + dy_cross*dy_cross)
                print(f"   Cross-odom difference: {cross_diff:.3f}m")
            print()
            
        if ang_jump > 0.5:  # ~30 degrees
            print(f"🚨 {source} ANGULAR JUMP: {ang_jump:.3f}rad ({math.degrees(ang_jump):.1f}°)")
            
        if vel_jump > 2.0:  # 2 m/s
            print(f"🚨 {source} VELOCITY JUMP: {vel_jump:.3f}m/s")
            
    def quaternion_to_yaw(self, q):
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle


def main():
    parser = argparse.ArgumentParser(description='Quick odometry jump monitor')
    parser.add_argument('--wheel-topic', default='odom_wheel', help='Wheel odometry topic')
    parser.add_argument('--icp-topic', default='odom', help='ICP odometry topic')
    parser.add_argument('--lidar-topic', default='scan', help='Lidar scan topic')
    
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        node = QuickOdomMonitor(args.wheel_topic, args.icp_topic, args.lidar_topic)
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nMonitoring stopped.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
