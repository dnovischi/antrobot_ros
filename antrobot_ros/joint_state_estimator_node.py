#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
import math
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
# TODO: Add initial pose service for this node
class JointStateEstimator(Node):
    """
    Joint State Estimator Node
    
    Estimates joint states (wheel positions and velocities) from odometry data.
    Supports two modes:
    1. RDrive mode (default): Uses twist data directly from RDrive odometry
    2. ICP mode (optional): Estimates from pose changes in ICP-based odometry
    
    The mode is controlled by the 'use_icp_estimation' parameter.
    """
    def __init__(self):
        super().__init__('joint_state_estimator')

        # Define a QoS profile for joint state data:
        joint_state_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,             # Only keep the most recent message.
            depth=5,                                         # Depth of 5 (note only the latest update is needed)
            reliability=QoSReliabilityPolicy.BEST_EFFORT,    # Occasional loss is acceptable.
            durability=QoSDurabilityPolicy.VOLATILE,         # Data is transient; no need for storage.
            deadline=Duration(seconds=0),                    # No deadline enforcement.
            lifespan=Duration(seconds=0),                    # Messages do not expire.
            liveliness=QoSLivelinessPolicy.AUTOMATIC,        # Default liveliness behavior.
            liveliness_lease_duration=Duration(seconds=0)    # Liveliness lease duration not set.
        )

        # Declare parameters
        self.declare_parameter('odom', 'wheel_odom')  # Default to RDrive odometry
        self.declare_parameter('joint_state_topic', 'joint_states')
        self.declare_parameter('wheel_radius', 0.03)
        self.declare_parameter('wheel_separation', 0.219)
        self.declare_parameter('publish_frequency', 10.0)  # Default frequency is 10 Hz
        self.declare_parameter('use_icp_estimation', False)  # Use RDrive by default, ICP as option
        self.declare_parameter('velocity_correction_factor', 0.1)  # Velocity correction factor for hybrid estimation

        # Get parameters
        self.odom_topic = self.get_parameter('odom').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.publish_frequency = self.get_parameter('publish_frequency').value
        self.use_icp_estimation = self.get_parameter('use_icp_estimation').value
        self.velocity_correction_factor = self.get_parameter('velocity_correction_factor').value

        # Initial joint state
        self.left_wheel_pos = 0.0
        self.right_wheel_pos = 0.0
        self.left_wheel_vel = 0.0
        self.right_wheel_vel = 0.0
        
        # For RDrive estimation: track last time and pose for hybrid estimation
        self.last_time_rdrive = None
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0
        
        # Subscriber to odometry (RDrive or ICP-based)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, joint_state_qos)

        # Publisher for joint states
        self.joint_pub = self.create_publisher(JointState, self.joint_state_topic, joint_state_qos)

        # Store last time and pose for velocity calculation (only needed for ICP estimation)
        self.last_time = self.get_clock().now()
        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.0

        # Log the mode being used
        if self.use_icp_estimation:
            self.get_logger().info(f'Joint state estimator using ICP-based estimation from topic: {self.odom_topic}')
        else:
            self.get_logger().info(f'Joint state estimator using RDrive odometry from topic: {self.odom_topic}')

        # Timer to periodically publish joint states
        self.timer_period = 1.0 / self.publish_frequency  # seconds
        self.timer = self.create_timer(self.timer_period, self.timer_callback)

    def timer_callback(self):
        # Publish joint states periodically
        self.publish_joint_states(self.left_wheel_pos, self.right_wheel_pos, self.left_wheel_vel, self.right_wheel_vel)

    def odom_callback(self, msg):
        if self.use_icp_estimation:
            # ICP-based estimation: derive joint states from pose changes
            self.icp_estimation(msg)
        else:
            # RDrive-based: use twist data directly from odometry
            self.rdrive_estimation(msg)

    def rdrive_estimation(self, msg):
        """Estimate joint states using hybrid approach: pose-based positions with velocity corrections"""
        # Get velocities directly from RDrive odometry twist (for immediate use and corrections)
        v = msg.twist.twist.linear.x
        omega = msg.twist.twist.angular.z
        
        # Convert to wheel velocities using differential drive kinematics
        self.left_wheel_vel = (v - omega * self.wheel_separation / 2.0) / self.wheel_radius
        self.right_wheel_vel = (v + omega * self.wheel_separation / 2.0) / self.wheel_radius
        
        # Get pose and timestamp from odometry message
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.yaw_from_quaternion(msg.pose.pose.orientation)
        current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        
        # Initialize on first call
        if self.last_time_rdrive is None:
            self.last_time_rdrive = current_time
            self.last_x = x
            self.last_y = y
            self.last_theta = theta
            return
        
        # Calculate time difference and pose changes
        dt = (current_time - self.last_time_rdrive).nanoseconds / 1e9
        
        if dt > 0:
            # Method 1: Calculate wheel positions from pose changes (primary method)
            dx = x - self.last_x
            dy = y - self.last_y
            dtheta = theta - self.last_theta
            
            # Handle angle wraparound
            dtheta = math.atan2(math.sin(dtheta), math.cos(dtheta))
            # Distance traveled by robot center
            distance_increment = math.sqrt(dx**2 + dy**2)
            
            # Determine direction of travel (forward/backward) using velocity
            if v < 0:  # Moving backward
                distance_increment = -distance_increment
            
            # Calculate wheel position increments from pose changes
            left_pos_increment = distance_increment - (dtheta * self.wheel_separation / 2.0)
            right_pos_increment = distance_increment + (dtheta * self.wheel_separation / 2.0)
            
            # Calculate expected increments from velocities (for correction)
            left_vel_increment = self.left_wheel_vel * dt * self.wheel_radius
            right_vel_increment = self.right_wheel_vel * dt * self.wheel_radius
            
            # Hybrid approach: Use pose-based as primary, velocity-based for correction
            # Apply a small correction factor from velocity estimates
            corrected_left_increment = (left_pos_increment * (1 - self.velocity_correction_factor) + 
                                      left_vel_increment * self.velocity_correction_factor)
            corrected_right_increment = (right_pos_increment * (1 - self.velocity_correction_factor) + 
                                       right_vel_increment * self.velocity_correction_factor)
            
            # Update wheel positions with corrected increments
            self.left_wheel_pos += corrected_left_increment / self.wheel_radius
            self.right_wheel_pos += corrected_right_increment / self.wheel_radius
        
        # Update last pose and time
        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.last_time_rdrive = current_time

    def icp_estimation(self, msg):
        """Estimate joint states using ICP pose changes (legacy method)"""
        # Get pose from ICP odometry
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        theta = self.yaw_from_quaternion(msg.pose.pose.orientation)

        # Get current time
        current_time = self.get_clock().now()

        # Calculate time difference
        dt = (current_time - self.last_time).nanoseconds / 1e9  # Convert to seconds

        if dt > 0:
            # Estimate velocities from pose changes
            v = math.sqrt((x - self.last_x)**2 + (y - self.last_y)**2) / dt
            omega = (theta - self.last_theta) / dt

            # Compute wheel positions from cumulative pose
            total_distance = math.sqrt(x**2 + y**2)
            self.left_wheel_pos = (total_distance - theta * self.wheel_separation / 2.0) / self.wheel_radius
            self.right_wheel_pos = (total_distance + theta * self.wheel_separation / 2.0) / self.wheel_radius

            # Compute velocities
            self.left_wheel_vel = (v / self.wheel_radius) - ((omega * self.wheel_separation) / (2 * self.wheel_radius))
            self.right_wheel_vel = (v / self.wheel_radius) + ((omega * self.wheel_separation) / (2 * self.wheel_radius))

        # Update last pose and time
        self.last_x = x
        self.last_y = y
        self.last_theta = theta
        self.last_time = current_time

    def yaw_from_quaternion(self, q):
        """Extract yaw (rotation about Z) from quaternion"""
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def publish_joint_states(self, left_pos, right_pos, left_vel, right_vel):
        # Add NaN validation before publishing
        if (math.isnan(left_pos) or math.isnan(right_pos) or 
            math.isnan(left_vel) or math.isnan(right_vel)):
            self.get_logger().warn(f"NaN detected in joint states: pos=[{left_pos}, {right_pos}], vel=[{left_vel}, {right_vel}]")
            return  # Don't publish invalid data
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['wheel_left_joint', 'wheel_right_joint']
        msg.position = [left_pos, right_pos]
        msg.velocity = [left_vel, right_vel]
        msg.effort = []

        self.joint_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JointStateEstimator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
