#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster
import math
from gpiozero import Button
import math
import threading
import time

class SimpleEncoderNode(Node):
    def __init__(self):
        super().__init__('simple_encoder_node')
        
        # Robot physical parameters
        self.wheel_radius = 0.05  # meters
        self.wheel_separation = 0.24  # meters
        self.ticks_per_revolution = 20  # Adjust based on your encoder
        
        # Encoder setup (single pin per wheel)
        self.left_encoder = Button(20, pull_up=True)  # GPIO 20
        self.right_encoder = Button(21, pull_up=True)  # GPIO 21
        
        # Encoder callbacks
        self.left_encoder.when_pressed = self.left_encoder_callback
        self.right_encoder.when_pressed = self.right_encoder_callback
        
        # Encoder counts
        self.left_count = 0
        self.right_count = 0
        self.last_left_count = 0
        self.last_right_count = 0
        
        # Wheel angles (in radians)
        self.left_wheel_angle = 0.0
        self.right_wheel_angle = 0.0
        
        # Odometry
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_time = time.time()
        
        # ROS2 setup
        self.odom_publisher = self.create_publisher(Odometry, 'odom', 10)
        self.joint_state_publisher = self.create_publisher(JointState, 'joint_states', 10)
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing odometry
        self.timer = self.create_timer(0.05, self.publish_odometry)  # 20Hz
        
        self.get_logger().info('Simple Encoder Node started')
        self.get_logger().info(f'Left encoder: GPIO 20, Right encoder: GPIO 21')
        
    def left_encoder_callback(self):
        """Called when left encoder pin goes high"""
        self.left_count += 1
        # Update wheel angle
        self.left_wheel_angle = (self.left_count / self.ticks_per_revolution) * 2 * math.pi
        
    def right_encoder_callback(self):
        """Called when right encoder pin goes high"""
        self.right_count += 1
        # Update wheel angle
        self.right_wheel_angle = (self.right_count / self.ticks_per_revolution) * 2 * math.pi
    
    def calculate_odometry(self):
        """Calculate robot pose from encoder data"""
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            return
            
        # Get encoder deltas
        delta_left = self.left_count - self.last_left_count
        delta_right = self.right_count - self.last_right_count
        
        # Convert ticks to distance
        distance_per_tick = (2 * math.pi * self.wheel_radius) / self.ticks_per_revolution
        left_distance = delta_left * distance_per_tick
        right_distance = delta_right * distance_per_tick
        
        # Calculate robot motion
        delta_distance = (left_distance + right_distance) / 2.0
        delta_theta = (right_distance - left_distance) / self.wheel_separation
        
        # Update robot pose
        self.x += delta_distance * math.cos(self.theta + delta_theta/2.0)
        self.y += delta_distance * math.sin(self.theta + delta_theta/2.0)
        self.theta += delta_theta
        
        # Normalize theta
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
        
        # Calculate velocities
        linear_velocity = delta_distance / dt
        angular_velocity = delta_theta / dt
        
        # Update for next iteration
        self.last_left_count = self.left_count
        self.last_right_count = self.right_count
        self.last_time = current_time
        
        return linear_velocity, angular_velocity
    
    def publish_joint_states(self):
        """Publish joint states for visualization"""
        joint_state = JointState()
        joint_state.header.stamp = self.get_clock().now().to_msg()
        joint_state.name = ['left_wheel_joint', 'right_wheel_joint']
        joint_state.position = [self.left_wheel_angle, self.right_wheel_angle]
        joint_state.velocity = []
        joint_state.effort = []
        
        self.joint_state_publisher.publish(joint_state)
    
    def publish_odometry(self):
        """Publish odometry message"""
        velocities = self.calculate_odometry()
        if velocities is None:
            return
            
        linear_vel, angular_vel = velocities
        current_time = self.get_clock().now()
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time.to_msg()
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_link'
        
        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (quaternion from yaw)
        cos_half_yaw = math.cos(self.theta * 0.5)
        sin_half_yaw = math.sin(self.theta * 0.5)
        
        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = sin_half_yaw
        odom.pose.pose.orientation.w = cos_half_yaw
        
        # Velocity
        odom.twist.twist.linear.x = linear_vel
        odom.twist.twist.linear.y = 0.0
        odom.twist.twist.angular.z = angular_vel
        
        # Covariance (tune these values)
        odom.pose.covariance[0] = 0.001  # x
        odom.pose.covariance[7] = 0.001  # y
        odom.pose.covariance[35] = 0.001 # yaw
        
        odom.twist.covariance[0] = 0.001  # vx
        odom.twist.covariance[35] = 0.001 # vyaw
        
        # Publish odometry
        self.odom_publisher.publish(odom)
        
        # Publish joint states for wheel visualization
        self.publish_joint_states()
        
        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time.to_msg()
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_link'
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = odom.pose.pose.orientation
        
        self.tf_broadcaster.sendTransform(transform)
        
    def destroy_node(self):
        """Clean shutdown"""
        if hasattr(self, 'left_encoder'):
            self.left_encoder.close()
        if hasattr(self, 'right_encoder'):
            self.right_encoder.close()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = SimpleEncoderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt received')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
