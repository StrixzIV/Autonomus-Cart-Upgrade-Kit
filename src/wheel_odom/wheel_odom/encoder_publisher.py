#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
import RPi.GPIO as GPIO

L_ENCODER_PIN = 17
R_ENCODER_PIN = 18

class EncoderPublisher(Node):
    
    def __init__(self):
        
        super().__init__('encoder_publisher_node')
        
        # Declare parameters
        self.declare_parameter('left_encoder_pin', L_ENCODER_PIN)
        self.declare_parameter('right_encoder_pin', R_ENCODER_PIN)
        self.declare_parameter('publish_rate', 10.0)
        
        # Get parameters
        self.left_encoder_pin = self.get_parameter('left_encoder_pin').value
        self.right_encoder_pin = self.get_parameter('right_encoder_pin').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Set up GPIO
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(self.left_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        GPIO.setup(self.right_encoder_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)
        
        # Counter variables
        self.left_count = 0
        self.right_count = 0
        
        # Create publishers
        self.left_pub = self.create_publisher(Int32, 'left_encoder_ticks', 10)
        self.right_pub = self.create_publisher(Int32, 'right_encoder_ticks', 10)
        
        # Set up GPIO event detection
        GPIO.add_event_detect(self.left_encoder_pin, GPIO.RISING, 
                              callback=self.left_encoder_callback, bouncetime=5)
        GPIO.add_event_detect(self.right_encoder_pin, GPIO.RISING, 
                              callback=self.right_encoder_callback, bouncetime=5)
        
        # Timer for publishing encoder counts
        self.timer = self.create_timer(1.0/publish_rate, self.publish_encoder_counts)
        
        self.get_logger().info('Encoder publisher initialized on Raspberry Pi')
    
    def left_encoder_callback(self, channel):
        self.left_count += 1
    
    def right_encoder_callback(self, channel):
        self.right_count += 1
    
    def publish_encoder_counts(self):
        # Create and publish messages with current counts
        left_msg = Int32()
        left_msg.data = self.left_count
        self.left_pub.publish(left_msg)
        
        right_msg = Int32()
        right_msg.data = self.right_count
        self.right_pub.publish(right_msg)
    
    def destroy_node(self):
        # Clean up GPIO on shutdown
        GPIO.cleanup()
        super().destroy_node()


def main(args=None):
    
    rclpy.init(args=args)
    
    node = EncoderPublisher()
    
    try:
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        pass
    
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()