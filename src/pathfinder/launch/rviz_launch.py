from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # Get the package share directory
    pkg_pathfinder_dir = get_package_share_directory('pathfinder')
    
    # Construct the path to the RViz config file
    rviz_config_path = os.path.join(
        pkg_pathfinder_dir, 
        'rviz', 
        'test_config.rviz'
    )

    # RViz2 Node
    rviz2_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d' + rviz_config_path],

    )
    
    ld = LaunchDescription()
    ld.add_action(rviz2_node)

    return ld