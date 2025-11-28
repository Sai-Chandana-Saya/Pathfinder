#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
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
        
        # Create publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # Initialize twist message
        self.twist_msg = Twist()
        
        # Save terminal settings
        self.old_settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Teleop Keyboard Node Started')
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
                    self.twist_msg.linear.x = linear
                    self.twist_msg.angular.z = angular
                    
                    self.publisher_.publish(self.twist_msg)
                    self.get_logger().info(f'Publishing: linear={linear:.1f}, angular={angular:.1f}')
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # Stop the robot before exiting
            self.twist_msg.linear.x = 0.0
            self.twist_msg.angular.z = 0.0
            self.publisher_.publish(self.twist_msg)
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
        stop_msg = Twist()
        node.publisher_.publish(stop_msg)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
