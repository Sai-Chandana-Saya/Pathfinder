#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
import sys
import select
import tty
import termios
import threading

# Define key mappings
KEY_BINDINGS = {
    'w': (0.2, 0.0),    # Forward
    's': (-0.2, 0.0),   # Backward
    'a': (0.0, 1.0),    # Turn left
    'd': (0.0, -1.0),   # Turn right
    ' ': (0.0, 0.0),    # Stop
    'x': (0.0, 0.0),    # Stop
}

class TeleopKeyboard(Node):
    def __init__(self):
        super().__init__('teleop_keyboard')
        
        # Create publisher for cmd_vel with TwistStamped
        self.publisher_ = self.create_publisher(TwistStamped, '/cmd_vel', 10)
        
        # Initialize twist stamped message
        self.twist_stamped_msg = TwistStamped()
        
        # Set the frame_id (important for TwistStamped)
        self.twist_stamped_msg.header.frame_id = 'base_link'
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop Keyboard Node Started (TwistStamped)')
        self.get_logger().info('Control Your Robot!')
        self.get_logger().info('---------------------------')
        self.get_logger().info('w: Move forward')
        self.get_logger().info('s: Move backward') 
        self.get_logger().info('a: Turn left')
        self.get_logger().info('d: Turn right')
        self.get_logger().info('x or space: Stop')
        self.get_logger().info('q: Quit')
        self.get_logger().info('---------------------------')
        
    def get_key(self):
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
        if rlist:
            key = sys.stdin.read(1)
        else:
            key = ''
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
        return key
    
    def run(self):
        try:
            while rclpy.ok():
                key = self.get_key()
                if key == 'q':
                    break
                    
                if key in KEY_BINDINGS:
                    linear, angular = KEY_BINDINGS[key]
                    
                    # Update the twist data
                    self.twist_stamped_msg.twist.linear.x = linear
                    self.twist_stamped_msg.twist.angular.z = angular
                    
                    # Update the header timestamp
                    self.twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
                    
                    self.publisher_.publish(self.twist_stamped_msg)
                    self.get_logger().info(f'Publishing TwistStamped: linear={linear:.1f}, angular={angular:.1f}')
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot before exiting
            self.twist_stamped_msg.twist.linear.x = 0.0
            self.twist_stamped_msg.twist.angular.z = 0.0
            self.twist_stamped_msg.header.stamp = self.get_clock().now().to_msg()
            self.publisher_.publish(self.twist_stamped_msg)
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)
            self.get_logger().info('Teleop node shutting down...')

def main(args=None):
    rclpy.init(args=args)
    node = TeleopKeyboard()
    
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the robot
        stop_msg = TwistStamped()
        stop_msg.header.stamp = node.get_clock().now().to_msg()
        stop_msg.header.frame_id = 'base_link'
        stop_msg.twist.linear.x = 0.0
        stop_msg.twist.angular.z = 0.0
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()