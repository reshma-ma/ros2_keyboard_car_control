#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import termios
import tty

class KeyboardInput(Node):
    def __init__(self):
        super().__init__('keyboard_input_node')
        self.pub = self.create_publisher(String, 'keyboard_input', 10)
        self.get_logger().info("Keyboard Input Node Initialized")
        self.settings = termios.tcgetattr(sys.stdin)  # Save the current terminal settings

    def get_key(self):
        tty.setraw(sys.stdin.fileno())  # Set the terminal to raw mode
        key = sys.stdin.read(1)  # Read a single character from stdin
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)  # Restore terminal settings
        return key

    def spin(self):
        while rclpy.ok():
            key = self.get_key()
            if key:
                msg = String()
                msg.data = key
                self.pub.publish(msg)
                self.get_logger().info(f"Published Key: {key}")

def main(args=None):
    rclpy.init(args=args)
    node = KeyboardInput()
    try:
        node.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

