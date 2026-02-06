#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist

class SimpleJoyToTwist(Node):
    def __init__(self):
        super().__init__('simple_joy_to_twist')
        self.sub = self.create_subscription(Joy, '/joy', self.callback, 10)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.get_logger().info("Node started - move left stick to publish cmd_vel")
    
    def callback(self, msg):
        twist = Twist()
        if len(msg.axes) > 1:
            twist.linear.x = float(msg.axes[1])  # Left stick up/down
            twist.angular.z = float(msg.axes[0])  # Left stick left/right
        self.pub.publish(twist)
        self.get_logger().info(f"Published: linear={twist.linear.x:.2f}, angular={twist.angular.z:.2f}")

def main():
    rclpy.init()
    node = SimpleJoyToTwist()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
