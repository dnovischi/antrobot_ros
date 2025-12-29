#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
TF Timing Diagnostics Tool

Specifically monitors for the timing issues causing the transform cache errors:
- "timestamp on the message is earlier than all the data in the transform cache"
- "Extrapolation Error looking up target frame: Lookup would require extrapolation into the past"

This tool helps diagnose the root cause of odometry jumps related to TF timing.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
import tf2_ros
import math
from datetime import datetime


class TFTimingDiagnostic(Node):
    def __init__(self):
        super().__init__('tf_timing_diagnostic')
        
        # Parameters
        self.declare_parameter('check_interval', 0.5)  # Check every 500ms
        self.declare_parameter('max_age_threshold', 1.0)  # 1 second
        self.declare_parameter('verbose', True)
        
        self.check_interval = self.get_parameter('check_interval').value
        self.max_age_threshold = self.get_parameter('max_age_threshold').value
        self.verbose = self.get_parameter('verbose').value
        
        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=30))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # QoS
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers to monitor message timestamps
        self.odom_wheel_sub = self.create_subscription(
            Odometry, 'odom_wheel', self.odom_wheel_callback, qos)
        self.odom_icp_sub = self.create_subscription(
            Odometry, 'odom', self.odom_icp_callback, qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, 'scan', self.lidar_callback, qos)
        
        # Message tracking
        self.last_messages = {
            'odom_wheel': None,
            'odom_icp': None,
            'scan': None
        }
        
        # Statistics
        self.stats = {
            'tf_errors': 0,
            'old_messages': 0,
            'extrapolation_errors': 0,
            'total_checks': 0
        }
        
        # Timer for periodic checks
        self.create_timer(self.check_interval, self.check_tf_health)
        
        print("="*60)
        print("TF TIMING DIAGNOSTIC TOOL")
        print("="*60)
        print("Monitoring for transform timing issues that cause:")
        print("• 'timestamp earlier than transform cache' errors")
        print("• 'Extrapolation into the past' errors")
        print("• Stale transform data")
        print("-"*60)
        
    def odom_wheel_callback(self, msg):
        self.last_messages['odom_wheel'] = msg
        self.check_message_age(msg, 'odom_wheel')
        
    def odom_icp_callback(self, msg):
        self.last_messages['odom_icp'] = msg
        self.check_message_age(msg, 'odom_icp')
        
    def lidar_callback(self, msg):
        self.last_messages['scan'] = msg
        self.check_message_age(msg, 'scan')
        
    def check_message_age(self, msg, source):
        """Check if message timestamp is significantly old"""
        current_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        age = (current_time - msg_time).nanoseconds / 1e9
        
        if age > self.max_age_threshold:
            self.stats['old_messages'] += 1
            print(f"⚠️  OLD MESSAGE: {source} is {age:.3f}s old (frame: {msg.header.frame_id})")
            print(f"   Current time: {current_time.nanoseconds / 1e9:.6f}")
            print(f"   Message time: {msg_time.nanoseconds / 1e9:.6f}")
            
        elif age < -0.1:  # Message from future
            print(f"🔮 FUTURE MESSAGE: {source} is {-age:.3f}s in the future!")
            
    def check_tf_health(self):
        """Comprehensive TF health check"""
        self.stats['total_checks'] += 1
        current_time = self.get_clock().now()
        
        # Critical transform chains for navigation
        critical_transforms = [
            ('map', 'base_link'),
            ('odom', 'base_link'), 
            ('odom_wheel', 'base_link'),
            ('base_link', 'rplidar_link'),
            ('map', 'odom'),
            ('odom', 'odom_wheel')
        ]
        
        print(f"\n--- TF Health Check #{self.stats['total_checks']} at {datetime.now().strftime('%H:%M:%S.%f')[:-3]} ---")
        
        for parent, child in critical_transforms:
            try:
                # Try to get the latest transform
                transform = self.tf_buffer.lookup_transform(
                    parent, child, rclpy.time.Time(),
                    timeout=rclpy.duration.Duration(seconds=0.1)
                )
                
                # Check transform age
                tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                age = (current_time - tf_time).nanoseconds / 1e9
                
                status = "✅ OK"
                if age > 1.0:
                    status = f"🔴 STALE ({age:.3f}s old)"
                    self.stats['tf_errors'] += 1
                elif age > 0.5:
                    status = f"🟡 OLD ({age:.3f}s)"
                    
                if self.verbose or age > 0.5:
                    print(f"  {parent} -> {child}: {status}")
                    
            except tf2_ros.ExtrapolationException as e:
                self.stats['extrapolation_errors'] += 1
                print(f"  🚨 EXTRAPOLATION ERROR: {parent} -> {child}")
                print(f"     {str(e)}")
                
                # Try to get more details about the error
                try:
                    # Get the time range available in the buffer
                    latest_time = self.tf_buffer.get_latest_common_time(parent, child)
                    print(f"     Latest common time: {latest_time.nanoseconds / 1e9:.6f}")
                    print(f"     Current time:       {current_time.nanoseconds / 1e9:.6f}")
                    print(f"     Time difference:    {(current_time - latest_time).nanoseconds / 1e9:.6f}s")
                except Exception as detail_e:
                    print(f"     Could not get timing details: {str(detail_e)}")
                    
            except tf2_ros.LookupException as e:
                print(f"  ❌ LOOKUP FAILED: {parent} -> {child}")
                print(f"     {str(e)}")
                
            except Exception as e:
                print(f"  ⚠️  OTHER ERROR: {parent} -> {child}: {str(e)}")
                
        # Check message-to-TF timing correlation
        self.check_message_tf_correlation()
        
        # Print summary every 10 checks
        if self.stats['total_checks'] % 10 == 0:
            self.print_summary()
            
    def check_message_tf_correlation(self):
        """Check if message timestamps correlate with TF availability"""
        try:
            # Check if recent messages can be transformed
            for source, msg in self.last_messages.items():
                if msg is None:
                    continue
                    
                try:
                    # Try to transform message to map frame
                    if hasattr(msg, 'pose'):  # Odometry message
                        # Check if we can transform the pose
                        self.tf_buffer.lookup_transform(
                            'map', msg.header.frame_id, 
                            rclpy.time.Time.from_msg(msg.header.stamp),
                            timeout=rclpy.duration.Duration(seconds=0.01)
                        )
                        
                except tf2_ros.ExtrapolationException:
                    print(f"  💥 Cannot transform {source} message (timestamp mismatch)")
                    
                except Exception:
                    pass  # Other errors are expected for some frame combinations
                    
        except Exception as e:
            if self.verbose:
                print(f"  Message-TF correlation check failed: {str(e)}")
                
    def print_summary(self):
        """Print diagnostic summary"""
        print("\n" + "="*60)
        print("TF TIMING DIAGNOSTIC SUMMARY")
        print("="*60)
        print(f"Total checks:          {self.stats['total_checks']}")
        print(f"TF errors:             {self.stats['tf_errors']}")
        print(f"Extrapolation errors:  {self.stats['extrapolation_errors']}")
        print(f"Old messages:          {self.stats['old_messages']}")
        
        # Calculate error rates
        if self.stats['total_checks'] > 0:
            tf_error_rate = (self.stats['tf_errors'] / self.stats['total_checks']) * 100
            extrap_rate = (self.stats['extrapolation_errors'] / self.stats['total_checks']) * 100
            
            print(f"TF error rate:         {tf_error_rate:.1f}%")
            print(f"Extrapolation rate:    {extrap_rate:.1f}%")
            
        print("="*60)
        
        # Recommendations
        if self.stats['extrapolation_errors'] > 0:
            print("\n🔧 RECOMMENDATIONS:")
            print("• Check if all nodes are publishing transforms regularly")
            print("• Verify system clock synchronization")
            print("• Check for nodes using old/cached timestamps")
            print("• Consider increasing TF buffer size")
            
        if self.stats['old_messages'] > 0:
            print("\n📝 OLD MESSAGE ISSUES:")
            print("• Some nodes are publishing messages with old timestamps")
            print("• This causes 'earlier than transform cache' errors")
            print("• Check sensor drivers and odometry nodes for timing issues")


def main():
    rclpy.init()
    
    try:
        node = TFTimingDiagnostic()
        print("Starting TF timing diagnostics... Press Ctrl+C to stop")
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\n\nTF timing diagnostics stopped.")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
