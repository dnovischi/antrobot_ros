#!/usr/bin/env python3
# Copyright (C) 2025 Dan Novischi. All rights reserved.
# This software may be modified and distributed under the terms of the
# GNU Lesser General Public License v3 or any later version.

import rclpy
from rclpy.qos import QoSProfile, QoSHistoryPolicy, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSLivelinessPolicy
from rclpy.duration import Duration
from rclpy.node import Node
from rcl_interfaces.msg import ParameterDescriptor
from antrobot_ros.rdrive import RDrive
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster
from builtin_interfaces.msg import Time
from nav2_msgs.srv import SetInitialPose
import math  # Replace tf_transformations with math


class RDriveNode(Node):
    def __init__(self):
        super().__init__('rdrive_node')
                
        # Declare parameters with default values (in case the antrobot_param.yaml is somehow missing)
        self.declare_parameter(
            'wheel_radius', 
            0.03, 
            ParameterDescriptor(description='Radius of the wheels')
        )
        self.declare_parameter(
            'wheel_separation', 
            0.219, 
            ParameterDescriptor(description='Separation between the wheels')
        )
        self.declare_parameter(
            'encoder_cpr_left', 
            2940, 
            ParameterDescriptor(description='Encoder counts per revolution for the left wheel')
        )
        self.declare_parameter(
            'encoder_cpr_right', 
            2940, 
            ParameterDescriptor(description='Encoder counts per revolution for the right wheel')
        )
        self.declare_parameter(
            'no_load_rpm_left', 
            100.0, 
            ParameterDescriptor(description='No-load RPM for the left motor')
        )
        self.declare_parameter(
            'no_load_rpm_right', 
            100.0, 
            ParameterDescriptor(description='No-load RPM for the right motor')
        )
        
        # Odometry and TF parameters
        self.declare_parameter(
            'odom_frequency', 
            20.0, 
            ParameterDescriptor(description='Odometry publishing frequency (Hz)')
        )
        self.declare_parameter(
            'odom_topic', 
            'odom', 
            ParameterDescriptor(description='Topic name for wheel encoder odometry data')
        )
        self.declare_parameter(
            'publish_tf', 
            True,
            ParameterDescriptor(description='Whether to publish TF transforms for wheel odometry')
        )
        
        self.declare_parameter(
            'invert_odom_tf', 
            False,
            ParameterDescriptor(description='Whether to invert TF transform (base_frame -> odom_frame)')
        )
        
        self.declare_parameter(
            'odom_frame_id', 
            'odom', 
            ParameterDescriptor(description='Frame ID for the odometry parent frame')
        )
        self.declare_parameter(
            'base_frame_id', 
            'base_link', 
            ParameterDescriptor(description='Frame ID for the robot base frame')
        )
        
        # Get the config rdrive parameters
        self.wheel_radius = self.get_parameter('wheel_radius').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        self.encoder_cpr_left = self.get_parameter('encoder_cpr_left').value
        self.encoder_cpr_right = self.get_parameter('encoder_cpr_right').value
        self.no_load_rpm_left = self.get_parameter('no_load_rpm_left').value
        self.no_load_rpm_right = self.get_parameter('no_load_rpm_right').value
        
        # Get odometry and TF parameters
        self.odom_frequency = self.get_parameter('odom_frequency').value
        self.odom_topic = self.get_parameter('odom_topic').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.invert_odom_tf = self.get_parameter('invert_odom_tf').value
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        # Validate odometry frequency
        if not (1.0 <= self.odom_frequency <= 100.0):
            self.get_logger().warn(f'Odometry frequency {self.odom_frequency} out of range [1-100], setting to 20 Hz')
            self.odom_frequency = 20.0
        
        # Define a QoS profile for command velocity
        cmd_vel_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            deadline=Duration(seconds=0),
            lifespan=Duration(seconds=0),
            liveliness=QoSLivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(seconds=0)
        )
        
        # Create the velocity command subscription
        self.cmd_vel_subscriber = self.create_subscription(
            msg_type=Twist, 
            topic='cmd_vel',
            callback=self.__cmd_vel_callback, 
            qos_profile=cmd_vel_qos
        )
        
        # Create odometry publisher
        odom_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE
        )
        
        self.odom_publisher = self.create_publisher(
            Odometry,
            self.odom_topic,
            odom_qos
        )
        
        # Create TF broadcaster (only if publishing TF)
        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)
        else:
            self.tf_broadcaster = None
        
        # Create odometry timer
        self.odom_timer = self.create_timer(
            1.0 / self.odom_frequency,
            self.publish_odometry
        )
        
        # Initialize odometry message
        self.odom_msg = Odometry()
        self.odom_msg.header.frame_id = self.odom_frame_id
        self.odom_msg.child_frame_id = self.base_frame_id
        
        # Add previous valid odometry data storage
        self.prev_valid_odom = {
            'x': 0.0,
            'y': 0.0, 
            'theta': 0.0,
            'linear_vel': 0.0,
            'angular_vel': 0.0
        }
        
        # Set covariance matrices (tune these values based on your robot's accuracy)
        pose_covariance = [
            0.1, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.1, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.1, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.1, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.1, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.1
        ]
        
        twist_covariance = [
            0.05, 0.0, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.05, 0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.05, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.05, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.05, 0.0,
            0.0, 0.0, 0.0, 0.0, 0.0, 0.05
        ]
        
        self.odom_msg.pose.covariance = pose_covariance
        self.odom_msg.twist.covariance = twist_covariance
        
        # Set node internal rdrive state
        self.drive_state = False
        
        # Instantiate RDrive
        self.drive = RDrive()
        
        # Log RDrive configuration
        self.get_logger().info(f'RDrive odometry topic: {self.odom_topic}')
        self.get_logger().info(f'RDrive odometry frequency: {self.odom_frequency} Hz')
        self.get_logger().info(f'RDrive publish TF: {self.publish_tf}')
        self.get_logger().info(f'RDrive invert TF: {self.invert_odom_tf}')
        if self.publish_tf:
            if self.invert_odom_tf:
                self.get_logger().info(f'RDrive TF (inverted): {self.base_frame_id} -> {self.odom_frame_id}')
            else:
                self.get_logger().info(f'RDrive TF (normal): {self.odom_frame_id} -> {self.base_frame_id}')
        else:
            self.get_logger().info('RDrive TF publishing disabled')

        # Create set pose service
        self.set_pose_service = self.create_service(
            SetInitialPose,
            '~/set_pose',
            self.set_pose_callback
        )
        
        self.get_logger().info('RDrive set_pose service available at: ~/set_pose')

    def __cmd_vel_callback(self, msg: Twist) -> None:
        v = msg.linear.x
        omega = msg.angular.z
        
        # Compute feasible command respecting robot constraints
        v_feasible, omega_feasible = self._compute_feasible_command(v, omega)
        
        self.drive.cmd_vel(v_feasible, omega_feasible)
    
    def _compute_feasible_command(self, v, omega):
        """
        Compute feasible linear and angular velocities based on robot constraints.
        
        For infeasible commands:
        - Scale command to fit within wheel velocity constraints while preserving signs
        - Apply additional 1% reduction to the scaled result for safety margin
        
        Args:
            v: Desired linear velocity (m/s)
            omega: Desired angular velocity (rad/s)
            
        Returns:
            tuple: (v_feasible, omega_feasible) in m/s and rad/s
        """
        # Calculate maximum wheel velocities from no-load RPM
        # Convert RPM to rad/s: RPM * (2*pi/60)
        max_wheel_vel_left = (self.no_load_rpm_left * 2.0 * math.pi / 60.0) * self.wheel_radius
        max_wheel_vel_right = (self.no_load_rpm_right * 2.0 * math.pi / 60.0) * self.wheel_radius
        
        # Use the smaller of the two as the constraint (conservative approach)
        max_wheel_vel = min(max_wheel_vel_left, max_wheel_vel_right)
        
        # Differential drive kinematics:
        # v_left = v - (omega * wheel_separation / 2)
        # v_right = v + (omega * wheel_separation / 2)
        half_separation = self.wheel_separation / 2.0
        
        # Calculate required wheel velocities for the desired command
        v_left_desired = v - omega * half_separation
        v_right_desired = v + omega * half_separation
        
        # Check if the desired velocities are within constraints
        max_desired_wheel_vel = max(abs(v_left_desired), abs(v_right_desired))
        
        if max_desired_wheel_vel <= max_wheel_vel:
            # Command is feasible as-is
            return v, omega
        
        # Command is not feasible, scale to fit constraints then reduce by 1%
        # while preserving signs and maintaining turn radius
        
        if abs(omega) < 1e-6:  # Pure linear motion
            # Scale linear velocity to maximum feasible while preserving sign
            if v > 0:
                v_scaled = max_wheel_vel
            elif v < 0:
                v_scaled = -max_wheel_vel
            else:
                v_scaled = 0.0
            omega_scaled = 0.0
        else:
            # For turning motion, maintain the turn radius: R = v / omega
            # Scale the command to fit within wheel velocity constraints
            scale_factor = max_wheel_vel / max_desired_wheel_vel
            
            # Apply scaling factor (preserves signs automatically)
            v_scaled = v * scale_factor
            omega_scaled = omega * scale_factor
        
        # Apply 1% reduction to the scaled command while preserving signs
        v_feasible = v_scaled * 0.99
        omega_feasible = omega_scaled * 0.99
        
        self.get_logger().debug(f'Scaled then reduced: scale_factor={(max_wheel_vel / max_desired_wheel_vel):.3f}, final_reduction=1%')
        
        # Log processing if significant changes occurred
        if abs(v - v_feasible) > 0.01 or abs(omega - omega_feasible) > 0.01:
            total_reduction = v_feasible / v if abs(v) > 1e-6 else 1.0
            self.get_logger().debug(f'Velocity constrained: ({v:.3f}, {omega:.3f}) -> '
                                  f'({v_feasible:.3f}, {omega_feasible:.3f}) '
                                  f'[total factor: {total_reduction:.3f}] '
                                  f'[wheel_vels: L={v_left_desired:.3f}->{v_feasible - omega_feasible * half_separation:.3f}, '
                                  f'R={v_right_desired:.3f}->{v_feasible + omega_feasible * half_separation:.3f}]')
        
        return v_feasible, omega_feasible
    
    def set_pose_callback(self, request, response):
        """Service callback to set the robot's pose."""
        try:
            # Extract pose from request
            x = request.pose.pose.pose.position.x
            y = request.pose.pose.pose.position.y
            
            # Convert quaternion to theta (yaw angle)
            # For 2D navigation, we only care about rotation around Z axis
            qz = request.pose.pose.pose.orientation.z
            qw = request.pose.pose.pose.orientation.w
            theta = 2.0 * math.atan2(qz, qw)
            
            # Set the pose in the drive system
            if self.drive_state:
                self.drive.set_pose(x=x, y=y, theta=theta)
                
                self.get_logger().info(f'RDrive pose set to: x={x:.3f}, y={y:.3f}, theta={theta:.3f}')
            else:
                self.get_logger().error('Cannot set pose: RDrive not initialized')
                
        except Exception as e:
            self.get_logger().error(f'Failed to set RDrive pose: {str(e)}')
            
        return response
    
    def publish_odometry(self):
        """Timer callback for odometry publishing."""
        if not self.drive_state:
            return
            
        # Get odometry data from drive
        odom_data = self.drive.get_odom()
        
        if odom_data is None:
            return
            
        # Parse odometry data: [timestamp_counts, x, y, theta, linear_velocity, angular_velocity]
        _, x, y, theta, linear_vel, angular_vel = odom_data
        
        # Check for NaN values and use previous valid data if found
        if (math.isnan(x) or math.isnan(y) or math.isnan(theta) or 
            math.isnan(linear_vel) or math.isnan(angular_vel)):
            
            self.get_logger().warn(f'NaN detected in odometry data: x={x}, y={y}, theta={theta}, '
                                 f'linear_vel={linear_vel}, angular_vel={angular_vel}. '
                                 f'Using previous valid values.')
            
            # Use previous valid values
            x = self.prev_valid_odom['x']
            y = self.prev_valid_odom['y']
            theta = self.prev_valid_odom['theta']
            linear_vel = self.prev_valid_odom['linear_vel']
            angular_vel = self.prev_valid_odom['angular_vel']
        else:
            # Update previous valid values with current valid data
            self.prev_valid_odom['x'] = x
            self.prev_valid_odom['y'] = y
            self.prev_valid_odom['theta'] = theta
            self.prev_valid_odom['linear_vel'] = linear_vel
            self.prev_valid_odom['angular_vel'] = angular_vel
        
        # Use current ROS time for odometry timestamp
        # Since RDrive computation is very fast (<1ms), this provides better
        # synchronization with other ROS data and evo trajectory alignment
        current_time = self.get_clock().now()
        self.odom_msg.header.stamp = current_time.to_msg()
        
        # Set position
        self.odom_msg.pose.pose.position.x = x
        self.odom_msg.pose.pose.position.y = y
        self.odom_msg.pose.pose.position.z = 0.0
        
        # Convert theta to quaternion using math (for 2D rotation)
        self.odom_msg.pose.pose.orientation.x = 0.0
        self.odom_msg.pose.pose.orientation.y = 0.0
        self.odom_msg.pose.pose.orientation.z = math.sin(theta / 2.0)
        self.odom_msg.pose.pose.orientation.w = math.cos(theta / 2.0)
        
        # Set velocity
        self.odom_msg.twist.twist.linear.x = linear_vel
        self.odom_msg.twist.twist.linear.y = 0.0
        self.odom_msg.twist.twist.linear.z = 0.0
        self.odom_msg.twist.twist.angular.x = 0.0
        self.odom_msg.twist.twist.angular.y = 0.0
        self.odom_msg.twist.twist.angular.z = angular_vel
        
        # Publish odometry
        self.odom_publisher.publish(self.odom_msg)
        
        # Publish TF transform (only if enabled)
        if self.publish_tf and self.tf_broadcaster:
            self.publish_tf_transform(current_time.to_msg(), x, y, theta)

    def publish_tf_transform(self, timestamp, x, y, theta):
        if not self.tf_broadcaster:
            self.get_logger().warn('TF broadcaster not initialized but publish_tf_transform called')
            return
            
        transform = TransformStamped()
        transform.header.stamp = timestamp
        
        if self.invert_odom_tf:
            # When inverted: base_link → odom_frame (odom_frame is child of base_link)
            # This means we need to compute the inverse transform
            transform.header.frame_id = self.base_frame_id
            transform.child_frame_id = self.odom_frame_id
            
            # For the inverse transform, we need to:
            # 1. Rotate the translation by -theta, then negate
            # 2. Negate the rotation
            cos_theta = math.cos(-theta)
            sin_theta = math.sin(-theta)
            
            # Rotate translation by -theta and negate
            inv_x = -(x * cos_theta - y * sin_theta)
            inv_y = -(x * sin_theta + y * cos_theta)
            
            transform.transform.translation.x = inv_x
            transform.transform.translation.y = inv_y
            transform.transform.translation.z = 0.0
            
            # Negate the rotation
            inv_theta = -theta
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(inv_theta / 2.0)
            transform.transform.rotation.w = math.cos(inv_theta / 2.0)
            
            self.get_logger().debug(f'Publishing inverted TF: {self.base_frame_id} → {self.odom_frame_id}, '
                                  f'pos=({inv_x:.3f}, {inv_y:.3f}), theta={inv_theta:.3f}')
        else:
            # Normal transform: odom_frame → base_link (base_link is child of odom_frame)
            transform.header.frame_id = self.odom_frame_id
            transform.child_frame_id = self.base_frame_id
            transform.transform.translation.x = x
            transform.transform.translation.y = y
            transform.transform.translation.z = 0.0
            transform.transform.rotation.x = 0.0
            transform.transform.rotation.y = 0.0
            transform.transform.rotation.z = math.sin(theta / 2.0)
            transform.transform.rotation.w = math.cos(theta / 2.0)
            
            self.get_logger().debug(f'Publishing normal TF: {self.odom_frame_id} → {self.base_frame_id}, '
                                  f'pos=({x:.3f}, {y:.3f}), theta={theta:.3f}')
        
        try:
            self.tf_broadcaster.sendTransform(transform)
        except Exception as e:
            self.get_logger().error(f'Failed to publish TF transform: {str(e)}')
    
    def drive_init(self) -> None:
        self.get_logger().info("RDrive initializing...")
        self.drive_state = self.drive.enable()
        if self.drive_state:
            try:
                self.drive.set_wheel_radius(self.wheel_radius)
                self.get_logger().info('Set RDrive wheel radius: "%.3f"' % self.wheel_radius)
            except Exception as e:
                self.get_logger().error('Failed to set wheel radius: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            try:
                self.drive.set_wheel_separation(self.wheel_separation)
                self.get_logger().info('Set RDrive wheel separation: "%.3f"' % self.wheel_separation)
            except Exception as e:
                self.get_logger().error('Failed to set wheel separation: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            try:
                self.drive.set_encoder_cpr(0, self.encoder_cpr_left)
                self.get_logger().info('Set RDrive encoder CPR left: "%d"' % self.encoder_cpr_left)
            except Exception as e:
                self.get_logger().error('Failed to set encoder CPR left: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            try:
                self.drive.set_encoder_cpr(1, self.encoder_cpr_right)
                self.get_logger().info('Set RDrive encoder CPR right: "%d"' % self.encoder_cpr_right)
            except Exception as e:
                self.get_logger().error('Failed to set encoder CPR right: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            try:
                self.drive.set_no_load_rpm(0, self.no_load_rpm_left)
                self.get_logger().info('Set RDrive no-load RPM left: "%.2f"' % self.no_load_rpm_left)
            except Exception as e:
                self.get_logger().error('Failed to set no-load RPM left: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            try:
                self.drive.set_no_load_rpm(1, self.no_load_rpm_right)
                self.get_logger().info('Set RDrive no-load RPM right: "%.2f"' % self.no_load_rpm_right)
            except Exception as e:
                self.get_logger().error('Failed to set no-load RPM right: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            try:
                self.drive.cmd_vel(0, 0)
                self.get_logger().info('RDrive running ...')
            except Exception as e:
                self.get_logger().error('Failed to send initial velocity command: %s' % str(e))
                self.drive.disable()
                self.drive_state = False
                return
            
            self.drive.set_pose(x=0.0, y=0.0, theta=0.0)
        else:
            self.get_logger().error('RDrive failed to initialize!')
    
    def get_drive_state(self) -> bool:
        return self.drive_state

    def drive_shutdown(self) -> None:
        if self.drive_state:
            # Stop any rdrive command
            self.drive.cmd_vel(0, 0)
            
            # Disable rdrive
            self.drive_state = self.drive.disable() 
            
            if not self.drive_state:
                self.get_logger().info('RDrive shutdown.')
    
    def destroy_node(self):
        self.drive_shutdown()
        return super().destroy_node()
            
        
def main(args=None):
    # Initialize ros client lib
    rclpy.init(args=args)
    
    # Instantiate the rdrive node
    rdirve_node = RDriveNode()
    
    # Initialize rdrive
    rdirve_node.drive_init()
    
    # If initialization failed
    if not rdirve_node.get_drive_state():
        rdirve_node.destroy_node()
        rclpy.shutdown()
        return
    try:
        # Run rdrive node
        rclpy.spin(rdirve_node)
    except KeyboardInterrupt:
        rdirve_node.get_logger().info('KeyboardInterrupt received, shutting down...')
    finally:
        # Don't wait for the garbage collector, free resources now!
        rdirve_node.destroy_node()
    
    # Shutdown ros client lib
    rclpy.shutdown()
        
if __name__ == "__main__":
    main()



