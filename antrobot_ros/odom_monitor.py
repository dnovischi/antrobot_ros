#!/usr/bin/env python3

# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

"""
Comprehensive Odometry Jump Detection and Diagnostics Node

Monitors:
- Wheel odometry (odom_wheel) for jumps and consistency
- ICP odometry (odom) for jumps and consistency  
- Lidar scan health for ICP input quality
- Cross-odometry consistency
- TF tree health
- Timing analysis

When jumps are detected, captures detailed diagnostics including:
- Pose differences
- Timing analysis
- TF tree state
- Lidar scan statistics
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import TransformStamped
import tf2_ros
import tf2_geometry_msgs
import math
import numpy as np
from datetime import datetime
import json


class OdomJumpMonitor(Node):
    def __init__(self):
        super().__init__('odom_monitor')
        
        # Declare parameters
        self.declare_parameter('wheel_odom_topic', 'odom_wheel')
        self.declare_parameter('icp_odom_topic', 'odom')
        self.declare_parameter('lidar_topic', 'scan')
        self.declare_parameter('max_linear_jump', 0.5)  # meters
        self.declare_parameter('max_angular_jump', 0.785)  # radians (45 degrees)
        self.declare_parameter('max_velocity_jump', 2.0)  # m/s
        self.declare_parameter('max_cross_odom_diff', 1.0)  # meters
        self.declare_parameter('min_lidar_points', 50)  # minimum points per scan
        self.declare_parameter('max_lidar_gap_time', 0.2)  # seconds
        self.declare_parameter('timestamp_sync_threshold', 0.5)  # seconds
        
        # Get parameters
        self.wheel_odom_topic = self.get_parameter('wheel_odom_topic').value
        self.icp_odom_topic = self.get_parameter('icp_odom_topic').value
        self.lidar_topic = self.get_parameter('lidar_topic').value
        self.max_linear_jump = self.get_parameter('max_linear_jump').value
        self.max_angular_jump = self.get_parameter('max_angular_jump').value
        self.max_velocity_jump = self.get_parameter('max_velocity_jump').value
        self.max_cross_odom_diff = self.get_parameter('max_cross_odom_diff').value
        self.min_lidar_points = self.get_parameter('min_lidar_points').value
        self.max_lidar_gap_time = self.get_parameter('max_lidar_gap_time').value
        self.timestamp_sync_threshold = self.get_parameter('timestamp_sync_threshold').value
        
        # QoS profiles
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        best_effort_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Subscribers
        self.wheel_odom_sub = self.create_subscription(
            Odometry, self.wheel_odom_topic, self.wheel_odom_callback, reliable_qos)
        self.icp_odom_sub = self.create_subscription(
            Odometry, self.icp_odom_topic, self.icp_odom_callback, reliable_qos)
        self.lidar_sub = self.create_subscription(
            LaserScan, self.lidar_topic, self.lidar_callback, best_effort_qos)
        
        # Publishers
        self.diagnostics_pub = self.create_publisher(
            DiagnosticArray, '/diagnostics', reliable_qos)
        
        # TF2 buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        # State tracking
        self.wheel_odom_history = []
        self.icp_odom_history = []
        self.lidar_stats = {
            'last_scan_time': None,
            'point_count': 0,
            'range_stats': {},
            'scan_rate': 0.0
        }
        
        # Jump detection state
        self.jump_count = 0
        self.last_jump_time = None
        
        # Diagnostics state
        self.wheel_status = DiagnosticStatus.OK
        self.icp_status = DiagnosticStatus.OK
        self.lidar_status = DiagnosticStatus.OK
        self.cross_odom_status = DiagnosticStatus.OK
        self.tf_status = DiagnosticStatus.OK
        
        # TF timing issues tracking
        self.tf_timing_issues = {
            'extrapolation_errors': 0,
            'dropped_messages': 0,
            'last_tf_error_time': None,
            'timestamp_gaps': []
        }
        
        # Message timing analysis
        self.msg_timing = {
            'wheel_odom': {'last_time': None, 'gaps': []},
            'icp_odom': {'last_time': None, 'gaps': []},
            'lidar': {'last_time': None, 'gaps': []}
        }
        
        # Timing
        self.create_timer(1.0, self.publish_diagnostics)
        self.create_timer(0.1, self.check_cross_consistency)
        self.create_timer(2.0, self.detect_tf_timing_issues)  # Check TF timing every 2 seconds
        self.create_timer(1.0, self.analyze_timestamp_synchronization)  # Check sync every second
        
        self.get_logger().info("Odometry Monitor started")
        self.get_logger().info(f"Monitoring: {self.wheel_odom_topic}, {self.icp_odom_topic}, {self.lidar_topic}")
        
    def wheel_odom_callback(self, msg):
        """Process wheel odometry messages"""
        self.analyze_message_timing(msg, 'wheel_odom')
        self.process_odometry(msg, 'wheel', self.wheel_odom_history)
        
    def icp_odom_callback(self, msg):
        """Process ICP odometry messages"""
        self.analyze_message_timing(msg, 'icp_odom')
        self.process_odometry(msg, 'icp', self.icp_odom_history)
        
    def lidar_callback(self, msg):
        """Process lidar scan messages"""
        current_time = self.get_clock().now()
        self.analyze_message_timing(msg, 'lidar')
        
        # Update lidar statistics
        if self.lidar_stats['last_scan_time'] is not None:
            dt = (current_time - self.lidar_stats['last_scan_time']).nanoseconds / 1e9
            if dt > 0:
                self.lidar_stats['scan_rate'] = 1.0 / dt
        
        self.lidar_stats['last_scan_time'] = current_time
        
        # Analyze scan quality
        valid_ranges = [r for r in msg.ranges if msg.range_min <= r <= msg.range_max]
        self.lidar_stats['point_count'] = len(valid_ranges)
        
        if valid_ranges:
            self.lidar_stats['range_stats'] = {
                'min': min(valid_ranges),
                'max': max(valid_ranges),
                'mean': np.mean(valid_ranges),
                'std': np.std(valid_ranges)
            }
        
        # Check lidar health
        if len(valid_ranges) < self.min_lidar_points:
            self.lidar_status = DiagnosticStatus.WARN
            self.get_logger().warn(f"Low lidar point count: {len(valid_ranges)} < {self.min_lidar_points}")
        else:
            self.lidar_status = DiagnosticStatus.OK
            
    def process_odometry(self, msg, source, history):
        """Process odometry message and detect jumps"""
        current_time = self.get_clock().now()
        
        # Extract pose and timing
        pose_data = {
            'timestamp': current_time,
            'ros_time': msg.header.stamp,
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'z': msg.pose.pose.position.z,
            'qx': msg.pose.pose.orientation.x,
            'qy': msg.pose.pose.orientation.y,
            'qz': msg.pose.pose.orientation.z,
            'qw': msg.pose.pose.orientation.w,
            'vx': msg.twist.twist.linear.x,
            'vy': msg.twist.twist.linear.y,
            'vz': msg.twist.twist.linear.z,
            'wx': msg.twist.twist.angular.x,
            'wy': msg.twist.twist.angular.y,
            'wz': msg.twist.twist.angular.z,
            'frame_id': msg.header.frame_id
        }
        
        # Add to history
        history.append(pose_data)
        if len(history) > 100:  # Keep last 100 messages
            history.pop(0)
            
        # Detect jumps if we have previous data
        if len(history) >= 2:
            self.detect_jump(history[-2], history[-1], source)
            
    def detect_jump(self, prev_pose, curr_pose, source):
        """Detect if there's a jump between consecutive poses"""
        # Calculate position difference
        dx = curr_pose['x'] - prev_pose['x']
        dy = curr_pose['y'] - prev_pose['y']
        dz = curr_pose['z'] - prev_pose['z']
        position_jump = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        # Calculate angular difference
        prev_yaw = self.quaternion_to_yaw(prev_pose)
        curr_yaw = self.quaternion_to_yaw(curr_pose)
        angular_jump = abs(self.normalize_angle(curr_yaw - prev_yaw))
        
        # Calculate time difference
        dt_ros = (curr_pose['ros_time'].sec - prev_pose['ros_time'].sec) + \
                 (curr_pose['ros_time'].nanosec - prev_pose['ros_time'].nanosec) * 1e-9
        dt_wall = (curr_pose['timestamp'] - prev_pose['timestamp']).nanoseconds / 1e9
        
        # Calculate velocity changes
        if dt_wall > 0:
            velocity_change = position_jump / dt_wall
        else:
            velocity_change = float('inf')
        
        # Check for jumps
        jump_detected = False
        jump_info = {
            'source': source,
            'timestamp': curr_pose['timestamp'],
            'position_jump': position_jump,
            'angular_jump': angular_jump,
            'velocity_change': velocity_change,
            'dt_ros': dt_ros,
            'dt_wall': dt_wall,
            'prev_pose': prev_pose,
            'curr_pose': curr_pose
        }
        
        if position_jump > self.max_linear_jump:
            jump_detected = True
            self.get_logger().error(f"POSITION JUMP detected in {source}: {position_jump:.3f}m")
            
        if angular_jump > self.max_angular_jump:
            jump_detected = True
            self.get_logger().error(f"ANGULAR JUMP detected in {source}: {angular_jump:.3f}rad ({math.degrees(angular_jump):.1f}°)")
            
        if velocity_change > self.max_velocity_jump:
            jump_detected = True
            self.get_logger().error(f"VELOCITY JUMP detected in {source}: {velocity_change:.3f}m/s")
            
        if jump_detected:
            self.handle_jump_detection(jump_info)
            
    def handle_jump_detection(self, jump_info):
        """Handle detected jump - log detailed information and capture diagnostics"""
        self.jump_count += 1
        self.last_jump_time = self.get_clock().now()
        
        # Update status
        if jump_info['source'] == 'wheel':
            self.wheel_status = DiagnosticStatus.ERROR
        else:
            self.icp_status = DiagnosticStatus.ERROR
            
        # Log detailed jump information with both odometry sources
        self.log_jump_details_with_cross_odom(jump_info)
        
        # Capture TF state
        self.capture_tf_state(jump_info)
        
    def log_jump_details(self, jump_info):
        """Log detailed information about the detected jump"""
        source = jump_info['source']
        timestamp = jump_info['timestamp']
        
        self.get_logger().error("="*60)
        self.get_logger().error(f"ODOMETRY JUMP DETECTED - {source.upper()}")
        self.get_logger().error("="*60)
        
        # Position information
        prev = jump_info['prev_pose']
        curr = jump_info['curr_pose']
        
        self.get_logger().error(f"Position jump: {jump_info['position_jump']:.6f}m")
        self.get_logger().error(f"Angular jump: {jump_info['angular_jump']:.6f}rad ({math.degrees(jump_info['angular_jump']):.2f}°)")
        self.get_logger().error(f"Velocity change: {jump_info['velocity_change']:.6f}m/s")
        
        self.get_logger().error(f"Previous pose: x={prev['x']:.6f}, y={prev['y']:.6f}, yaw={math.degrees(self.quaternion_to_yaw(prev)):.2f}°")
        self.get_logger().error(f"Current pose:  x={curr['x']:.6f}, y={curr['y']:.6f}, yaw={math.degrees(self.quaternion_to_yaw(curr)):.2f}°")
        
        self.get_logger().error(f"Time difference (ROS): {jump_info['dt_ros']:.6f}s")
        self.get_logger().error(f"Time difference (wall): {jump_info['dt_wall']:.6f}s")
        
        # TF timing issues at jump
        if self.tf_timing_issues['last_tf_error_time'] is not None:
            tf_error_age = (current_time - self.tf_timing_issues['last_tf_error_time']).nanoseconds / 1e9
            if tf_error_age < 5.0:  # TF error within last 5 seconds
                self.get_logger().error(f"TF timing error occurred {tf_error_age:.3f}s before this jump!")
                
        self.get_logger().error(f"Total TF extrapolation errors: {self.tf_timing_issues['extrapolation_errors']}")
        
        # Message timing analysis
        for source, timing in self.msg_timing.items():
            if timing['gaps'] and len(timing['gaps']) > 0:
                avg_gap = np.mean(timing['gaps'])
                max_gap = max(timing['gaps'])
                self.get_logger().error(f"{source.upper()} timing: avg={avg_gap:.3f}s, max={max_gap:.3f}s")
        
        self.get_logger().error("="*60)
        
    def log_jump_details_with_cross_odom(self, jump_info):
        """Log detailed information including both odometry sources"""
        source = jump_info['source']
        current_time = jump_info['timestamp']
        
        self.get_logger().error("="*80)
        self.get_logger().error(f"ODOMETRY JUMP DETECTED - {source.upper()}")
        self.get_logger().error("="*80)
        
        # Position information for the jumping source
        prev = jump_info['prev_pose']
        curr = jump_info['curr_pose']
        
        self.get_logger().error(f"Jump in {source.upper()} odometry:")
        self.get_logger().error(f"  Position jump: {jump_info['position_jump']:.6f}m")
        self.get_logger().error(f"  Angular jump: {jump_info['angular_jump']:.6f}rad ({math.degrees(jump_info['angular_jump']):.2f}°)")
        self.get_logger().error(f"  Velocity change: {jump_info['velocity_change']:.6f}m/s")
        self.get_logger().error(f"  Time difference (ROS): {jump_info['dt_ros']:.6f}s")
        self.get_logger().error(f"  Time difference (wall): {jump_info['dt_wall']:.6f}s")
        
        self.get_logger().error("")
        self.get_logger().error(f"{source.upper()} ODOMETRY POSES:")
        self.get_logger().error(f"  Previous: x={prev['x']:.6f}, y={prev['y']:.6f}, z={prev['z']:.6f}")
        self.get_logger().error(f"            yaw={math.degrees(self.quaternion_to_yaw(prev)):.3f}°")
        self.get_logger().error(f"            quat=[{prev['qx']:.6f}, {prev['qy']:.6f}, {prev['qz']:.6f}, {prev['qw']:.6f}]")
        self.get_logger().error(f"            vel=[{prev['vx']:.6f}, {prev['vy']:.6f}, {prev['vz']:.6f}]")
        self.get_logger().error(f"            ang_vel=[{prev['wx']:.6f}, {prev['wy']:.6f}, {prev['wz']:.6f}]")
        self.get_logger().error(f"            timestamp: {prev['ros_time'].sec}.{prev['ros_time'].nanosec:09d}")
        
        self.get_logger().error(f"  Current:  x={curr['x']:.6f}, y={curr['y']:.6f}, z={curr['z']:.6f}")
        self.get_logger().error(f"            yaw={math.degrees(self.quaternion_to_yaw(curr)):.3f}°")
        self.get_logger().error(f"            quat=[{curr['qx']:.6f}, {curr['qy']:.6f}, {curr['qz']:.6f}, {curr['qw']:.6f}]")
        self.get_logger().error(f"            vel=[{curr['vx']:.6f}, {curr['vy']:.6f}, {curr['vz']:.6f}]")
        self.get_logger().error(f"            ang_vel=[{curr['wx']:.6f}, {curr['wy']:.6f}, {curr['wz']:.6f}]")
        self.get_logger().error(f"            timestamp: {curr['ros_time'].sec}.{curr['ros_time'].nanosec:09d}")
        
        # Get the other odometry source at the same time
        other_source = 'wheel' if source == 'icp' else 'icp'
        other_history = self.wheel_odom_history if source == 'icp' else self.icp_odom_history
        other_topic = self.wheel_odom_topic if source == 'icp' else self.icp_odom_topic
        
        if len(other_history) > 0:
            # Find the closest timestamp in the other odometry
            other_pose = self.find_closest_pose_by_time(other_history, curr['ros_time'])
            
            if other_pose:
                self.get_logger().error("")
                self.get_logger().error(f"{other_source.upper()} ODOMETRY AT SAME TIME:")
                self.get_logger().error(f"  Topic: {other_topic}")
                self.get_logger().error(f"  Pose:     x={other_pose['x']:.6f}, y={other_pose['y']:.6f}, z={other_pose['z']:.6f}")
                self.get_logger().error(f"            yaw={math.degrees(self.quaternion_to_yaw(other_pose)):.3f}°")
                self.get_logger().error(f"            quat=[{other_pose['qx']:.6f}, {other_pose['qy']:.6f}, {other_pose['qz']:.6f}, {other_pose['qw']:.6f}]")
                self.get_logger().error(f"            vel=[{other_pose['vx']:.6f}, {other_pose['vy']:.6f}, {other_pose['vz']:.6f}]")
                self.get_logger().error(f"            ang_vel=[{other_pose['wx']:.6f}, {other_pose['wy']:.6f}, {other_pose['wz']:.6f}]")
                self.get_logger().error(f"            timestamp: {other_pose['ros_time'].sec}.{other_pose['ros_time'].nanosec:09d}")
                
                # Calculate time difference between odometry sources
                time_diff = abs((curr['ros_time'].sec + curr['ros_time'].nanosec * 1e-9) - 
                              (other_pose['ros_time'].sec + other_pose['ros_time'].nanosec * 1e-9))
                
                # Calculate position difference between odometry sources
                dx_cross = curr['x'] - other_pose['x']
                dy_cross = curr['y'] - other_pose['y']
                pos_diff = math.sqrt(dx_cross*dx_cross + dy_cross*dy_cross)
                
                # Calculate yaw difference between odometry sources
                yaw_diff = abs(self.normalize_angle(
                    self.quaternion_to_yaw(curr) - self.quaternion_to_yaw(other_pose)
                ))
                
                self.get_logger().error("")
                self.get_logger().error("CROSS-ODOMETRY COMPARISON:")
                self.get_logger().error(f"  Position difference: {pos_diff:.6f}m")
                self.get_logger().error(f"  Yaw difference: {math.degrees(yaw_diff):.3f}°")
                self.get_logger().error(f"  Timestamp difference: {time_diff:.6f}s")
                
                if pos_diff > 0.5:
                    self.get_logger().error(f"  ⚠️  LARGE CROSS-ODOM DIVERGENCE: {pos_diff:.3f}m!")
                    
            else:
                self.get_logger().error(f"")
                self.get_logger().error(f"⚠️  No {other_source.upper()} odometry data available for comparison")
        
        # TF timing issues at jump
        if self.tf_timing_issues['last_tf_error_time'] is not None:
            tf_error_age = (current_time - self.tf_timing_issues['last_tf_error_time']).nanoseconds / 1e9
            if tf_error_age < 5.0:  # TF error within last 5 seconds
                self.get_logger().error("")
                self.get_logger().error(f"⚠️  TF timing error occurred {tf_error_age:.3f}s before this jump!")
                
        self.get_logger().error("")
        self.get_logger().error(f"Total TF extrapolation errors: {self.tf_timing_issues['extrapolation_errors']}")
        
        # Message timing analysis
        self.get_logger().error("")
        self.get_logger().error("MESSAGE TIMING ANALYSIS:")
        for msg_source, timing in self.msg_timing.items():
            if timing['gaps'] and len(timing['gaps']) > 0:
                avg_gap = np.mean(timing['gaps'])
                max_gap = max(timing['gaps'])
                min_gap = min(timing['gaps'])
                self.get_logger().error(f"  {msg_source.upper()}: avg={avg_gap:.3f}s, min={min_gap:.3f}s, max={max_gap:.3f}s")
        
        # Lidar statistics
        if self.lidar_stats['point_count'] > 0:
            self.get_logger().error("")
            self.get_logger().error("LIDAR STATUS AT JUMP:")
            self.get_logger().error(f"  Point count: {self.lidar_stats['point_count']}")
            self.get_logger().error(f"  Scan rate: {self.lidar_stats['scan_rate']:.1f}Hz")
            if 'range_stats' in self.lidar_stats and self.lidar_stats['range_stats']:
                ranges = self.lidar_stats['range_stats']
                self.get_logger().error(f"  Range stats: min={ranges['min']:.2f}m, max={ranges['max']:.2f}m, mean={ranges['mean']:.2f}m, std={ranges['std']:.2f}m")
        
        self.get_logger().error("="*80)
        
    def find_closest_pose_by_time(self, history, target_time):
        """Find the pose in history with timestamp closest to target_time"""
        if not history:
            return None
            
        target_stamp = target_time.sec + target_time.nanosec * 1e-9
        closest_pose = None
        min_time_diff = float('inf')
        
        for pose in history:
            pose_stamp = pose['ros_time'].sec + pose['ros_time'].nanosec * 1e-9
            time_diff = abs(pose_stamp - target_stamp)
            
            if time_diff < min_time_diff:
                min_time_diff = time_diff
                closest_pose = pose
                
        # Only return if the closest pose is within 1 second
        if min_time_diff < 1.0:
            return closest_pose
        else:
            return None
        
    def capture_tf_state(self, jump_info):
        """Capture and log TF tree state at time of jump"""
        try:
            # Get key transforms
            transforms = {}
            frame_pairs = [
                ('odom', 'base_link'),
                ('odom_wheel', 'base_link'),
                ('odom', 'odom_wheel'),
                ('base_link', 'rplidar_link')
            ]
            
            for parent, child in frame_pairs:
                try:
                    transform = self.tf_buffer.lookup_transform(
                        parent, child, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.1)
                    )
                    transforms[f"{parent}->{child}"] = {
                        'translation': [
                            transform.transform.translation.x,
                            transform.transform.translation.y,
                            transform.transform.translation.z
                        ],
                        'rotation': [
                            transform.transform.rotation.x,
                            transform.transform.rotation.y,
                            transform.transform.rotation.z,
                            transform.transform.rotation.w
                        ]
                    }
                except Exception as e:
                    transforms[f"{parent}->{child}"] = f"Failed: {str(e)}"
                    
            self.get_logger().error("TF State at jump:")
            for tf_name, tf_data in transforms.items():
                self.get_logger().error(f"  {tf_name}: {tf_data}")
                
        except Exception as e:
            self.get_logger().error(f"Failed to capture TF state: {str(e)}")
            
    def check_cross_consistency(self):
        """Check consistency between wheel and ICP odometry"""
        if len(self.wheel_odom_history) > 0 and len(self.icp_odom_history) > 0:
            wheel_latest = self.wheel_odom_history[-1]
            icp_latest = self.icp_odom_history[-1]
            
            # Calculate position difference
            dx = wheel_latest['x'] - icp_latest['x']
            dy = wheel_latest['y'] - icp_latest['y']
            diff = math.sqrt(dx*dx + dy*dy)
            
            if diff > self.max_cross_odom_diff:
                self.cross_odom_status = DiagnosticStatus.WARN
                if self.jump_count == 0:  # Only log if no jumps detected
                    self.get_logger().warn(f"Large cross-odometry difference: {diff:.3f}m")
            else:
                self.cross_odom_status = DiagnosticStatus.OK
                
    def check_lidar_timing(self):
        """Check if lidar data is arriving regularly"""
        if self.lidar_stats['last_scan_time'] is not None:
            dt = (self.get_clock().now() - self.lidar_stats['last_scan_time']).nanoseconds / 1e9
            if dt > self.max_lidar_gap_time:
                self.lidar_status = DiagnosticStatus.ERROR
                self.get_logger().error(f"Lidar data timeout: {dt:.3f}s since last scan")
            
    def publish_diagnostics(self):
        """Publish diagnostic information"""
        self.check_lidar_timing()
        
        diag_array = DiagnosticArray()
        diag_array.header.stamp = self.get_clock().now().to_msg()
        
        # Wheel odometry diagnostics
        wheel_diag = DiagnosticStatus()
        wheel_diag.name = "wheel_odometry_monitor"
        wheel_diag.level = self.wheel_status
        wheel_diag.message = "Wheel odometry jump detection"
        wheel_diag.values = [
            KeyValue(key="topic", value=self.wheel_odom_topic),
            KeyValue(key="messages_received", value=str(len(self.wheel_odom_history))),
            KeyValue(key="max_linear_jump_threshold", value=f"{self.max_linear_jump}m"),
            KeyValue(key="max_angular_jump_threshold", value=f"{self.max_angular_jump}rad")
        ]
        
        # ICP odometry diagnostics
        icp_diag = DiagnosticStatus()
        icp_diag.name = "icp_odometry_monitor"
        icp_diag.level = self.icp_status
        icp_diag.message = "ICP odometry jump detection"
        icp_diag.values = [
            KeyValue(key="topic", value=self.icp_odom_topic),
            KeyValue(key="messages_received", value=str(len(self.icp_odom_history))),
            KeyValue(key="max_linear_jump_threshold", value=f"{self.max_linear_jump}m"),
            KeyValue(key="max_angular_jump_threshold", value=f"{self.max_angular_jump}rad")
        ]
        
        # Lidar diagnostics
        lidar_diag = DiagnosticStatus()
        lidar_diag.name = "lidar_health_monitor"
        lidar_diag.level = self.lidar_status
        lidar_diag.message = "Lidar health for ICP input"
        lidar_diag.values = [
            KeyValue(key="topic", value=self.lidar_topic),
            KeyValue(key="point_count", value=str(self.lidar_stats['point_count'])),
            KeyValue(key="scan_rate", value=f"{self.lidar_stats['scan_rate']:.1f}Hz"),
            KeyValue(key="min_points_threshold", value=str(self.min_lidar_points))
        ]
        
        # Cross-odometry consistency
        cross_diag = DiagnosticStatus()
        cross_diag.name = "cross_odometry_consistency"
        cross_diag.level = self.cross_odom_status
        cross_diag.message = "Wheel vs ICP odometry consistency"
        cross_diag.values = [
            KeyValue(key="max_difference_threshold", value=f"{self.max_cross_odom_diff}m")
        ]
        
        # TF timing diagnostics
        tf_timing_diag = DiagnosticStatus()
        tf_timing_diag.name = "tf_timing_monitor"
        tf_timing_diag.level = self.tf_status
        tf_timing_diag.message = "TF timing and synchronization health"
        tf_timing_diag.values = [
            KeyValue(key="extrapolation_errors", value=str(self.tf_timing_issues['extrapolation_errors'])),
            KeyValue(key="dropped_messages", value=str(self.tf_timing_issues['dropped_messages'])),
            KeyValue(key="last_tf_error", value=str(self.tf_timing_issues['last_tf_error_time']) if self.tf_timing_issues['last_tf_error_time'] else "None")
        ]

        # Overall summary
        summary_diag = DiagnosticStatus()
        summary_diag.name = "odometry_jump_summary"
        summary_diag.level = max(self.wheel_status, self.icp_status, self.lidar_status)
        summary_diag.message = f"Total jumps detected: {self.jump_count}"
        summary_diag.values = [
            KeyValue(key="total_jumps", value=str(self.jump_count)),
            KeyValue(key="last_jump_time", value=str(self.last_jump_time) if self.last_jump_time else "None")
        ]
        
        diag_array.status = [wheel_diag, icp_diag, lidar_diag, cross_diag, tf_timing_diag, summary_diag]
        self.diagnostics_pub.publish(diag_array)
        
        # Reset status levels if they were warnings/errors
        if self.wheel_status != DiagnosticStatus.ERROR:
            self.wheel_status = DiagnosticStatus.OK
        if self.icp_status != DiagnosticStatus.ERROR:
            self.icp_status = DiagnosticStatus.OK
            
    def quaternion_to_yaw(self, pose_data):
        """Convert quaternion to yaw angle"""
        qx, qy, qz, qw = pose_data['qx'], pose_data['qy'], pose_data['qz'], pose_data['qw']
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        return math.atan2(siny_cosp, cosy_cosp)
        
    def normalize_angle(self, angle):
        """Normalize angle to [-pi, pi]"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def analyze_message_timing(self, msg, source):
        """Analyze message timing for timestamp issues"""
        current_wall_time = self.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        # Check for timestamp issues
        time_diff = (current_wall_time - msg_time).nanoseconds / 1e9
        
        # Detect old timestamps (likely cause of TF extrapolation errors)
        if abs(time_diff) > 1.0:  # Message is more than 1 second old/future
            self.get_logger().warn(f"TIMING ISSUE in {source}: Message timestamp is {time_diff:.3f}s different from wall time")
            if time_diff > 0:
                self.get_logger().warn(f"  Message is {time_diff:.3f}s OLD - this causes TF extrapolation errors!")
            else:
                self.get_logger().warn(f"  Message is {-time_diff:.3f}s in the FUTURE")
        
        # Track message gaps
        timing_data = self.msg_timing[source]
        if timing_data['last_time'] is not None:
            gap = (msg_time - timing_data['last_time']).nanoseconds / 1e9
            timing_data['gaps'].append(gap)
            
            # Keep only recent gaps
            if len(timing_data['gaps']) > 20:
                timing_data['gaps'].pop(0)
                
            # Detect irregular timing
            if gap > 0.5:  # More than 500ms gap
                self.get_logger().warn(f"LARGE MESSAGE GAP in {source}: {gap:.3f}s")
                
        timing_data['last_time'] = msg_time
        
    def detect_tf_timing_issues(self):
        """Detect TF timing and synchronization issues"""
        try:
            current_time = self.get_clock().now()
            
            # Check key transforms for timing issues
            critical_transforms = [
                ('map', 'base_link'),
                ('odom', 'base_link'),
                ('odom_wheel', 'base_link'),
                ('base_link', 'rplidar_link')
            ]
            
            for parent, child in critical_transforms:
                try:
                    # Try to get the latest transform
                    transform = self.tf_buffer.lookup_transform(
                        parent, child, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=0.01)
                    )
                    
                    # Check timestamp freshness
                    tf_time = rclpy.time.Time.from_msg(transform.header.stamp)
                    
                    # Handle special cases
                    if tf_time.nanoseconds == 0:
                        self.get_logger().debug(f"TF {parent}->{child}: Zero timestamp, skipping age check")
                        continue
                        
                    age_duration = current_time - tf_time
                    age = age_duration.nanoseconds / 1e9
                    
                    # Sanity check: age should be positive and reasonable
                    if age < 0:
                        self.get_logger().debug(f"TF {parent}->{child}: Negative age {age:.3f}s (future transform?), skipping")
                        continue
                    elif age > 3600:  # More than 1 hour old seems unreasonable
                        self.get_logger().debug(f"TF {parent}->{child}: Unreasonably old age {age:.3f}s, skipping")
                        continue
                    
                    # Debug log the actual values
                    self.get_logger().debug(f"TF {parent}->{child}: current_time={current_time.nanoseconds/1e9:.3f}, tf_time={tf_time.nanoseconds/1e9:.3f}, age={age:.3f}s")
                    
                    if age > 1.0:  # Transform is more than 1 second old
                        self.get_logger().error(f"STALE TF: {parent}->{child} is {age:.3f}s old!")
                        self.tf_timing_issues['extrapolation_errors'] += 1
                        
                except tf2_ros.ExtrapolationException as e:
                    self.get_logger().error(f"TF EXTRAPOLATION ERROR: {parent}->{child}: {str(e)}")
                    self.tf_timing_issues['extrapolation_errors'] += 1
                    self.tf_timing_issues['last_tf_error_time'] = current_time
                    
                except tf2_ros.LookupException as e:
                    self.get_logger().error(f"TF LOOKUP ERROR: {parent}->{child}: {str(e)}")
                    
                except Exception as e:
                    self.get_logger().debug(f"TF check failed for {parent}->{child}: {str(e)}")
                    
        except Exception as e:
            self.get_logger().debug(f"TF timing check failed: {str(e)}")
            
    def analyze_timestamp_synchronization(self):
        """Analyze timestamp synchronization between odometry sources"""
        if len(self.wheel_odom_history) > 0 and len(self.icp_odom_history) > 0:
            wheel_latest = self.wheel_odom_history[-1]
            icp_latest = self.icp_odom_history[-1]
            
            # Compare timestamps
            wheel_time = wheel_latest['ros_time']
            icp_time = icp_latest['ros_time']
            
            wheel_stamp = wheel_time.sec + wheel_time.nanosec * 1e-9
            icp_stamp = icp_time.sec + icp_time.nanosec * 1e-9
            
            time_sync_diff = abs(wheel_stamp - icp_stamp)
            
            # Use configurable threshold - ICP typically has processing delay after wheel odometry
            if time_sync_diff > self.timestamp_sync_threshold:
                self.get_logger().warn(f"LARGE TIMESTAMP SYNC ISSUE: Wheel and ICP odometry timestamps differ by {time_sync_diff:.3f}s")
                self.get_logger().warn(f"  Wheel timestamp: {wheel_stamp:.6f}")
                self.get_logger().warn(f"  ICP timestamp:   {icp_stamp:.6f}")
                self.get_logger().warn(f"  This may indicate processing delays or timing issues in the ICP pipeline")
            elif time_sync_diff > self.timestamp_sync_threshold * 0.4:  # Log as debug for moderate differences
                self.get_logger().debug(f"Normal ICP processing delay: {time_sync_diff:.3f}s between wheel and ICP odometry")


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OdomJumpMonitor()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
