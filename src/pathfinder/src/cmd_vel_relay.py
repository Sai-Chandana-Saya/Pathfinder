#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class CmdVelRelay(Node):
    def __init__(self):
        super().__init__('cmd_vel_relay')
        
        # Create subscriber to /cmd_vel_nav
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10)
        
        # Create publisher to /cmd_vel
        self.publisher = self.create_publisher(
            Twist,
            '/cmd_vel',
            10)
        
        self.get_logger().info('CmdVel relay node started: /cmd_vel_nav -> /cmd_vel')
    
    def cmd_vel_callback(self, msg):
        # Simply relay the message to /cmd_vel
        self.publisher.publish(msg)
        
        # Optional: log for debugging
        # self.get_logger().info(f'Relaying cmd_vel: linear.x={msg.linear.x}, angular.z={msg.angular.z}')

def main():
    rclpy.init()
    node = CmdVelRelay()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()