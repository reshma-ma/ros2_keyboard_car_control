#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import math

class VelocityPublisher(Node):
    def __init__(self):
        super().__init__('velocity_publisher_node')
        self.subscription = self.create_subscription(
            String,
            'keyboard_input',
            self.listener_callback,
            10)
        self.pub = self.create_publisher(Twist, 'cmd_vel', 10)
        self.linear_vel = 0.2
        self.angular_vel = math.radians(20)
        self.vel = Twist()
        self.get_logger().info("Velocity Publisher Node Initialized")
    
    def stop(self):
        self.vel.linear.x = 0.0
        self.vel.linear.y = 0.0
        self.vel.linear.z = 0.0
        self.vel.angular.x = 0.0
        self.vel.angular.y = 0.0
        self.vel.angular.z = 0.0
        self.pub.publish(self.vel)
    
    def listener_callback(self, msg):
        key = msg.data
        if key == "w":
            self.get_logger().info("forward")
            self.vel.linear.x = self.linear_vel
            self.vel.angular.z = 0.0
        elif key == "a":
            self.get_logger().info("turn left")
            self.vel.angular.z = self.angular_vel
            self.vel.linear.x = 0.0
        elif key == "x":
            self.get_logger().info("back")
            self.vel.linear.x = -self.linear_vel
            self.vel.angular.z = 0.0
        elif key == "d":
            self.get_logger().info("turn right")
            self.vel.angular.z = -self.angular_vel
            self.vel.linear.x = 0.0
        elif key == "s":
            self.get_logger().info("stop")
            self.stop()
        self.pub.publish(self.vel)
        self.get_logger().info(f"Velocity: Linear={self.vel.linear.x} Angular={self.vel.angular.z}")

def main(args=None):
    rclpy.init(args=args)
    node = VelocityPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
