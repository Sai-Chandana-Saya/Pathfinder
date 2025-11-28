from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='pathfinder',
            executable='cmd_vel_relay',  
            name='cmd_vel_relay',
            output='screen',
        )
    ])