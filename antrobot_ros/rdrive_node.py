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
import time


class PoseCorrector:
    """
    Detects and corrects pose jumps in odometry data based on command velocities and timestamps.
    """
    
    def __init__(self, max_linear_jump=0.5, max_angular_jump=0.785, max_velocity_jump=2.0, max_dt=0.5):
        """
        Initialize the pose corrector.
        
        Args:
            max_linear_jump: Maximum allowable linear position jump (meters)
            max_angular_jump: Maximum allowable angular jump (radians)  
            max_velocity_jump: Maximum allowable velocity change (m/s)
            max_dt: Maximum time difference for correction (seconds)
        """
        self.max_linear_jump = max_linear_jump
        self.max_angular_jump = max_angular_jump
        self.max_velocity_jump = max_velocity_jump
        self.max_dt = max_dt
        
        # Previous state
        self.prev_pose = {'x': 0.0, 'y': 0.0, 'theta': 0.0}
        self.prev_velocity = {'linear': 0.0, 'angular': 0.0}
        self.prev_timestamp = None
        self.prev_commands = {'linear': 0.0, 'angular': 0.0}
        
        # Correction statistics
        self.correction_count = 0
        self.total_measurements = 0
        
    def update_commands(self, linear_cmd, angular_cmd):
        """Update the current command velocities."""
        self.prev_commands['linear'] = linear_cmd
        self.prev_commands['angular'] = angular_cmd
    
    def predict_pose(self, dt):
        """
        Predict the expected pose based on previous pose and command velocities.
        
        Args:
            dt: Time difference (seconds)
            
        Returns:
            dict: Predicted pose {'x', 'y', 'theta'}
        """
        if dt <= 0 or dt > self.max_dt:
            return self.prev_pose.copy()
            
        # Use command velocities for prediction (more reliable than reported velocities during jumps)
        v = self.prev_commands['linear']
        omega = self.prev_commands['angular']
        
        # Predict new pose using differential drive kinematics
        if abs(omega) < 1e-6:  # Straight line motion
            dx = v * dt * math.cos(self.prev_pose['theta'])
            dy = v * dt * math.sin(self.prev_pose['theta'])
            dtheta = 0.0
        else:  # Curved motion
            # Radius of curvature
            R = v / omega
            
            # Change in orientation
            dtheta = omega * dt
            
            # Position change in robot frame
            dx_robot = R * math.sin(dtheta)
            dy_robot = R * (1 - math.cos(dtheta))
            
            # Transform to global frame
            cos_theta = math.cos(self.prev_pose['theta'])
            sin_theta = math.sin(self.prev_pose['theta'])
            
            dx = dx_robot * cos_theta - dy_robot * sin_theta
            dy = dx_robot * sin_theta + dy_robot * cos_theta
        
        predicted_pose = {
            'x': self.prev_pose['x'] + dx,
            'y': self.prev_pose['y'] + dy,
            'theta': self.prev_pose['theta'] + dtheta
        }
        
        # Normalize theta to [-pi, pi]
        predicted_pose['theta'] = math.atan2(math.sin(predicted_pose['theta']), 
                                           math.cos(predicted_pose['theta']))
        
        return predicted_pose
    
    def detect_jump(self, current_pose, current_velocity, current_timestamp):
        """
        Detect if there's a jump in pose or velocity data.
        
        Args:
            current_pose: dict with 'x', 'y', 'theta'
            current_velocity: dict with 'linear', 'angular'
            current_timestamp: timestamp in seconds
            
        Returns:
            tuple: (has_jump, jump_info, corrected_pose)
        """
        self.total_measurements += 1
        
        if self.prev_timestamp is None:
            # First measurement, no correction needed
            self.prev_pose = current_pose.copy()
            self.prev_velocity = current_velocity.copy()
            self.prev_timestamp = current_timestamp
            return False, {}, current_pose
        
        # Calculate time difference
        dt = current_timestamp - self.prev_timestamp
        
        if dt <= 0 or dt > self.max_dt:
            # Invalid time difference, skip correction
            self.prev_pose = current_pose.copy()
            self.prev_velocity = current_velocity.copy()
            self.prev_timestamp = current_timestamp
            return False, {"reason": f"Invalid dt: {dt}"}, current_pose
        
        # Calculate position and angle differences
        dx = current_pose['x'] - self.prev_pose['x']
        dy = current_pose['y'] - self.prev_pose['y']
        position_jump = math.sqrt(dx*dx + dy*dy)
        
        angle_diff = current_pose['theta'] - self.prev_pose['theta']
        # Normalize angle difference to [-pi, pi]
        angle_jump = abs(math.atan2(math.sin(angle_diff), math.cos(angle_diff)))
        
        # Calculate velocity jumps
        velocity_jump = abs(current_velocity['linear'] - self.prev_velocity['linear'])
        angular_velocity_jump = abs(current_velocity['angular'] - self.prev_velocity['angular'])
        
        # Check for jumps
        has_position_jump = position_jump > self.max_linear_jump
        has_angle_jump = angle_jump > self.max_angular_jump
        has_velocity_jump = velocity_jump > self.max_velocity_jump
        
        jump_detected = has_position_jump or has_angle_jump or has_velocity_jump
        
        jump_info = {
            'position_jump': position_jump,
            'angle_jump': angle_jump,
            'velocity_jump': velocity_jump,
            'angular_velocity_jump': angular_velocity_jump,
            'dt': dt,
            'has_position_jump': has_position_jump,
            'has_angle_jump': has_angle_jump,
            'has_velocity_jump': has_velocity_jump
        }
        
        corrected_pose = current_pose.copy()
        
        if jump_detected:
            self.correction_count += 1
            
            # Predict expected pose based on previous state and commands
            predicted_pose = self.predict_pose(dt)
            
            # Use predicted pose instead of jumped pose
            corrected_pose = predicted_pose.copy()
            
            jump_info.update({
                'predicted_pose': predicted_pose,
                'original_pose': current_pose.copy(),
                'corrected': True
            })
        else:
            jump_info['corrected'] = False  # No correction applied
        
        # Update previous state with corrected values
        self.prev_pose = corrected_pose.copy()
        self.prev_velocity = current_velocity.copy()
        self.prev_timestamp = current_timestamp
        
        return jump_detected, jump_info, corrected_pose
    
    def get_correction_stats(self):
        """Get correction statistics."""
        correction_rate = (self.correction_count / self.total_measurements * 100) if self.total_measurements > 0 else 0
        return {
            'total_corrections': self.correction_count,
            'total_measurements': self.total_measurements,
            'correction_rate_percent': correction_rate
        }


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
        
        # Initialize pose corrector
        self.declare_parameter('max_linear_jump', 0.5, 
                             ParameterDescriptor(description='Maximum allowable linear position jump (meters)'))
        self.declare_parameter('max_angular_jump', 0.785, 
                             ParameterDescriptor(description='Maximum allowable angular jump (radians)'))
        self.declare_parameter('max_velocity_jump', 2.0,
                             ParameterDescriptor(description='Maximum allowable velocity change (m/s)'))
        self.declare_parameter('enable_jump_correction', True,
                             ParameterDescriptor(description='Enable pose jump detection and correction'))
        
        max_linear_jump = self.get_parameter('max_linear_jump').value
        max_angular_jump = self.get_parameter('max_angular_jump').value
        max_velocity_jump = self.get_parameter('max_velocity_jump').value
        self.enable_jump_correction = self.get_parameter('enable_jump_correction').value
        
        self.pose_corrector = PoseCorrector(
            max_linear_jump=max_linear_jump,
            max_angular_jump=max_angular_jump,
            max_velocity_jump=max_velocity_jump
        )
        
        # Track current command for correction
        self.current_cmd_vel = {'linear': 0.0, 'angular': 0.0}
        
        # Statistics timer
        self.stats_timer = self.create_timer(10.0, self.log_correction_stats)
        
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
        
        # Store current command for pose correction
        self.current_cmd_vel['linear'] = v
        self.current_cmd_vel['angular'] = omega
        
        # Update pose corrector with current commands
        if self.enable_jump_correction:
            self.pose_corrector.update_commands(v, omega)
        
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
    
    def log_correction_stats(self):
        """Log pose correction statistics periodically."""
        if self.enable_jump_correction:
            stats = self.pose_corrector.get_correction_stats()
            if stats['total_measurements'] > 0:
                self.get_logger().info(
                    f"Pose correction stats: {stats['total_corrections']} corrections "
                    f"out of {stats['total_measurements']} measurements "
                    f"({stats['correction_rate_percent']:.1f}% correction rate)"
                )

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
        timestamp_counts, x_raw, y_raw, theta_raw, linear_vel_raw, angular_vel_raw = odom_data
        
        # Check for NaN values and use previous valid data if found
        if (math.isnan(x_raw) or math.isnan(y_raw) or math.isnan(theta_raw) or 
            math.isnan(linear_vel_raw) or math.isnan(angular_vel_raw)):
            
            self.get_logger().warn(f'NaN detected in odometry data: x={x_raw}, y={y_raw}, theta={theta_raw}, '
                                 f'linear_vel={linear_vel_raw}, angular_vel={angular_vel_raw}. '
                                 f'Using previous valid values.')
            
            # Use previous valid values
            x = self.prev_valid_odom['x']
            y = self.prev_valid_odom['y']
            theta = self.prev_valid_odom['theta']
            linear_vel = self.prev_valid_odom['linear_vel']
            angular_vel = self.prev_valid_odom['angular_vel']
        else:
            x, y, theta = x_raw, y_raw, theta_raw
            linear_vel, angular_vel = linear_vel_raw, angular_vel_raw
        
        # Apply jump detection and correction
        current_pose = {'x': x, 'y': y, 'theta': theta}
        current_velocity = {'linear': linear_vel, 'angular': angular_vel}
        current_timestamp = time.time()  # Use wall time for consistency
        
        if self.enable_jump_correction:
            has_jump, jump_info, corrected_pose = self.pose_corrector.detect_jump(
                current_pose, current_velocity, current_timestamp
            )
            
            if has_jump:
                # Log jump detection
                self.get_logger().warn(
                    f"POSE JUMP DETECTED AND CORRECTED:\n"
                    f"  Position jump: {jump_info['position_jump']:.3f}m\n"
                    f"  Angular jump: {jump_info['angle_jump']:.3f}rad ({math.degrees(jump_info['angle_jump']):.1f}°)\n"
                    f"  Velocity jump: {jump_info['velocity_jump']:.3f}m/s\n"
                    f"  Time difference: {jump_info['dt']:.3f}s\n"
                    f"  Original pose: x={current_pose['x']:.3f}, y={current_pose['y']:.3f}, θ={current_pose['theta']:.3f}\n"
                    f"  Corrected pose: x={corrected_pose['x']:.3f}, y={corrected_pose['y']:.3f}, θ={corrected_pose['theta']:.3f}\n"
                    f"  Command: v={self.current_cmd_vel['linear']:.3f}, ω={self.current_cmd_vel['angular']:.3f}"
                )
                
                # Update the hardware with corrected pose
                try:
                    self.drive.set_pose(corrected_pose['x'], corrected_pose['y'], corrected_pose['theta'])
                    self.get_logger().info(f"Hardware pose updated to corrected values")
                except Exception as e:
                    self.get_logger().error(f"Failed to update hardware pose: {e}")
                
            # Use corrected pose
            x = corrected_pose['x']
            y = corrected_pose['y'] 
            theta = corrected_pose['theta']
        
        # Update previous valid values with final corrected data
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
            # This creates an inverted TF tree structure
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



