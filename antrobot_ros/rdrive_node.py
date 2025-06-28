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
            50.0, 
            ParameterDescriptor(description='Odometry publishing frequency (Hz)')
        )
        self.declare_parameter(
            'odom_topic', 
            'wheel_odom', 
            ParameterDescriptor(description='Topic name for wheel encoder odometry data')
        )
        self.declare_parameter(
            'publish_tf', 
            False, 
            ParameterDescriptor(description='Whether to publish TF transforms for wheel odometry')
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
        self.odom_frame_id = self.get_parameter('odom_frame_id').value
        self.base_frame_id = self.get_parameter('base_frame_id').value
        
        # Validate odometry frequency
        if not (1.0 <= self.odom_frequency <= 100.0):
            self.get_logger().warn(f'Odometry frequency {self.odom_frequency} out of range [1-100], setting to 50 Hz')
            self.odom_frequency = 50.0
        
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
        
        # Relative timestamp tracking for odometry
        self.first_hardware_timestamp = None
        self.last_hardware_timestamp = None
        self.odometry_start_time = None
        
        # Instantiate RDrive
        self.drive = RDrive()
        
        # Log RDrive configuration
        self.get_logger().info(f'RDrive odometry topic: {self.odom_topic}')
        self.get_logger().info(f'RDrive odometry frequency: {self.odom_frequency} Hz')
        self.get_logger().info(f'RDrive publish TF: {self.publish_tf}')
        if self.publish_tf:
            self.get_logger().info(f'RDrive TF: {self.odom_frame_id} -> {self.base_frame_id}')

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
        self.drive.cmd_vel(v, omega)
    
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
                
                # Reset timestamp tracking to align with new pose
                self.first_hardware_timestamp = None
                self.last_hardware_timestamp = None
                self.odometry_start_time = None
                
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
        rdrive_timestamp_counts_raw, x, y, theta, linear_vel, angular_vel = odom_data
        
        # Handle uint64_t timestamp from RDrive hardware
        # Python may interpret large uint64_t values as negative signed integers
        # Convert to proper unsigned 64-bit value
        if rdrive_timestamp_counts_raw < 0:
            # Convert negative signed interpretation to positive unsigned value
            rdrive_timestamp_counts = rdrive_timestamp_counts_raw + (2**64)
            self.get_logger().debug(f'Converted negative timestamp: {rdrive_timestamp_counts_raw} -> {rdrive_timestamp_counts}')
        else:
            rdrive_timestamp_counts = rdrive_timestamp_counts_raw
        
        # Convert RDrive timestamp counts to nanoseconds
        rdrive_timestamp_ns = rdrive_timestamp_counts * 100_000
        
        # Initialize relative timestamp tracking on first reading
        if self.first_hardware_timestamp is None:
            self.first_hardware_timestamp = rdrive_timestamp_counts
            self.odometry_start_time = self.get_clock().now()
            self.last_hardware_timestamp = rdrive_timestamp_counts
            self.get_logger().info(f'RDrive odometry timestamps initialized:')
            self.get_logger().info(f'  Starting hardware timestamp: {rdrive_timestamp_counts} counts')
            self.get_logger().info(f'  Starting ROS time: {self.odometry_start_time.nanoseconds / 1e9:.6f} seconds')
        
        # Calculate relative timestamp from first reading
        if rdrive_timestamp_counts >= self.first_hardware_timestamp:
            relative_timestamp_counts = rdrive_timestamp_counts - self.first_hardware_timestamp
        else:
            # Handle uint64_t wraparound (very unlikely but possible)
            relative_timestamp_counts = (2**64 - self.first_hardware_timestamp) + rdrive_timestamp_counts
        
        # Convert relative counts to nanoseconds and add to start time
        relative_timestamp_ns = relative_timestamp_counts * 100_000
        odometry_timestamp_ns = self.odometry_start_time.nanoseconds + relative_timestamp_ns
        
        # Optional safety check for timestamp consistency
        if self.last_hardware_timestamp is not None:
            # Calculate time delta, handling uint64_t properly
            if rdrive_timestamp_counts >= self.last_hardware_timestamp:
                time_delta = rdrive_timestamp_counts - self.last_hardware_timestamp
            else:
                # Handle uint64_t wraparound (very unlikely but possible)
                time_delta = (2**64 - self.last_hardware_timestamp) + rdrive_timestamp_counts
            
            # Log if time jumps are unexpectedly large (more than 1 second)
            if time_delta > 10000:  # 10000 counts = 1 second
                self.get_logger().debug(f'Large timestamp delta: {time_delta} counts ({time_delta * 0.0001:.3f}s)')
                
        self.last_hardware_timestamp = rdrive_timestamp_counts
        
        # Create ROS2 timestamp from calculated time
        ros_timestamp = Time()
        
        # Ensure timestamp values are within valid ranges
        total_seconds = odometry_timestamp_ns // 1_000_000_000
        remaining_nanoseconds = odometry_timestamp_ns % 1_000_000_000
        
        # ROS2 Time.sec is a 32-bit signed integer, so clamp to valid range
        if total_seconds > 2147483647:  # Max int32
            self.get_logger().warn(f'Timestamp seconds too large: {total_seconds}, using current ROS time instead')
            current_time = self.get_clock().now()
            ros_timestamp = current_time.to_msg()
        elif total_seconds < 0:
            self.get_logger().warn(f'Timestamp seconds negative: {total_seconds}, using current ROS time instead')
            current_time = self.get_clock().now()
            ros_timestamp = current_time.to_msg()
        else:
            ros_timestamp.sec = int(total_seconds)
            ros_timestamp.nanosec = int(remaining_nanoseconds)
        
        # Use calculated timestamp for odometry
        self.odom_msg.header.stamp = ros_timestamp
        
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
            self.publish_tf_transform(ros_timestamp, x, y, theta)

    def publish_tf_transform(self, ros_timestamp, x, y, theta):
        """Publish the transform from odom_frame_id to base_frame_id."""
        transform = TransformStamped()
        
        transform.header.stamp = ros_timestamp
        transform.header.frame_id = self.odom_frame_id
        transform.child_frame_id = self.base_frame_id
        
        transform.transform.translation.x = x
        transform.transform.translation.y = y
        transform.transform.translation.z = 0.0
        
        # Convert theta to quaternion using math (for 2D rotation)
        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = math.sin(theta / 2.0)
        transform.transform.rotation.w = math.cos(theta / 2.0)
        
        self.tf_broadcaster.sendTransform(transform)
    
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



