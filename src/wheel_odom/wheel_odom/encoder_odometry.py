#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped, Quaternion
from tf2_ros import TransformBroadcaster

class EncoderOdometryNode(Node):
    def __init__(self):
        super().__init__('encoder_odometry_node')
        
        # Declare parameters
        self.declare_parameter('wheel_diameter', 0.127)  # 5 inches in meters
        self.declare_parameter('encoder_resolution', 16)  # 16 holes per revolution
        self.declare_parameter('wheel_separation', 0.3)   # Distance between wheels in meters
        self.declare_parameter('publish_rate', 10.0)     # Publishing rate in Hz
        
        # Get parameters
        wheel_diameter = self.get_parameter('wheel_diameter').value
        encoder_resolution = self.get_parameter('encoder_resolution').value
        self.wheel_separation = self.get_parameter('wheel_separation').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Calculate constants
        self.wheel_circumference = math.pi * wheel_diameter
        self.meters_per_tick = self.wheel_circumference / encoder_resolution
        
        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.last_left_encoder = 0
        self.last_right_encoder = 0
        self.first_left_reading = True
        self.first_right_reading = True
        
        # Create subscribers for encoder ticks
        self.left_encoder_sub = self.create_subscription(
            Int32,
            'left_encoder_ticks',
            self.left_encoder_callback,
            10)
        
        self.right_encoder_sub = self.create_subscription(
            Int32,
            'right_encoder_ticks',
            self.right_encoder_callback,
            10)
        
        # Create publisher for odometry
        self.odom_pub = self.create_publisher(Odometry, 'odom', 10)
        
        # Create transform broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # Timer for publishing odometry
        self.timer = self.create_timer(1.0/publish_rate, self.publish_odometry)
        
        self.get_logger().info('Encoder odometry node initialized')
    
    def left_encoder_callback(self, msg):
        if self.first_left_reading:
            self.last_left_encoder = msg.data
            self.first_left_reading = False
            return
            
        delta_ticks = self.calculate_delta_ticks(msg.data, self.last_left_encoder)
        self.last_left_encoder = msg.data
        self.update_odometry(delta_ticks, 0)
    
    def right_encoder_callback(self, msg):
        if self.first_right_reading:
            self.last_right_encoder = msg.data
            self.first_right_reading = False
            return
            
        delta_ticks = self.calculate_delta_ticks(msg.data, self.last_right_encoder)
        self.last_right_encoder = msg.data
        self.update_odometry(0, delta_ticks)
    
    def calculate_delta_ticks(self, current, previous):
        delta = current - previous
        # Handle counter overflow (32-bit unsigned int)
        if abs(delta) > 1000:  # Threshold to detect overflow
            if delta < 0:
                delta = (2**32 - 1) - previous + current
            else:
                delta = -((2**32 - 1) - current + previous)
        return delta
    
    def update_odometry(self, left_ticks, right_ticks):
        # Convert ticks to distance
        left_distance = left_ticks * self.meters_per_tick
        right_distance = right_ticks * self.meters_per_tick
        
        # Calculate linear and angular displacement
        linear_displacement = (left_distance + right_distance) / 2.0
        angular_displacement = (right_distance - left_distance) / self.wheel_separation
        
        # Update pose
        if abs(angular_displacement) < 1e-6:  # Straight line motion
            self.x += linear_displacement * math.cos(self.theta)
            self.y += linear_displacement * math.sin(self.theta)
        else:
            # Circular arc motion
            radius = linear_displacement / angular_displacement
            old_theta = self.theta
            self.theta += angular_displacement
            self.x += radius * (math.sin(self.theta) - math.sin(old_theta))
            self.y -= radius * (math.cos(self.theta) - math.cos(old_theta))
        
        # Normalize angle to -pi to pi
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))
    
    def publish_odometry(self):
        current_time = self.get_clock().now().to_msg()
        
        # Create quaternion from yaw
        q = Quaternion()
        q.z = math.sin(self.theta / 2.0)
        q.w = math.cos(self.theta / 2.0)
        
        # Create odometry message
        odom = Odometry()
        odom.header.stamp = current_time
        odom.header.frame_id = "odom"
        odom.child_frame_id = "base_link"
        
        # Set position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = q
        
        # Publish odometry message
        self.odom_pub.publish(odom)
        
        # Publish transform
        transform = TransformStamped()
        transform.header.stamp = current_time
        transform.header.frame_id = "odom"
        transform.child_frame_id = "base_link"
        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0
        transform.transform.rotation = q
        
        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)
    node = EncoderOdometryNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()