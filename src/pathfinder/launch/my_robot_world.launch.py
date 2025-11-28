import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import AppendEnvironmentVariable
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_file_dir = os.path.join(get_package_share_directory('pathfinder'), 'launch')
    ros_gz_sim = get_package_share_directory('ros_gz_sim')
    pkg_pathfinder = get_package_share_directory('pathfinder')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    x_pose = LaunchConfiguration('x_pose', default='-2.0')
    y_pose = LaunchConfiguration('y_pose', default='-0.5')

    set_env_vars_resources = AppendEnvironmentVariable(
            'GZ_SIM_RESOURCE_PATH',
            os.path.join(pkg_pathfinder,
                'meshes'))

    #Process the URDF xacro file
    urdf_file = os.path.join(pkg_pathfinder,'urdf','my_robot.urdf')
    with open(urdf_file,'r') as f:
        robot_description = f.read()

    robot_description = xacro.process(urdf_file)

    # urdf_file = os.path.join(pkg_pathfinder,'urdf','robot.urdf')
    # with open(urdf_file,'r') as f:
    #     robot_description = f.read()

    # robot_description = xacro.process(urdf_file)

    

    robot_state_publisher_cmd = Node(
        package = 'robot_state_publisher',
        executable='robot_state_publisher',
        output = 'screen',
        parameters=[{
            'robot_description' : robot_description,
            'use_sim_time' : True
        }]
    )


    world_path = os.path.join(
        pkg_pathfinder,
        'worlds',
        'turtlebot3_world.world'
    )

    model_path = os.path.join(pkg_pathfinder,'models','model.sdf')

    gzserver_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': ['-r -s -v2 ', world_path], 'on_exit_shutdown': 'true'}.items()
    )

    gzclient_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': '-g -v2 ', 'on_exit_shutdown': 'true'}.items()
    )

    spawn_turtlebot_cmd = Node(
        package='ros_gz_sim', 
        executable='create',
        name='spawn_robot',
        output='screen',
        parameters=[{'use_sim_time': True}],
        arguments=[
            '-file', model_path,
            '-name', 'pathfinder',
            '-allow_renaming', 'true' ,
            '-x', x_pose,
            '-y', y_pose,
            '-z', '0.01'
        ],
    )

    # spawn_turtlebot_cmd = Node(
    #     package='ros_gz_sim', 
    #     executable='create',
    #     name='spawn_robot',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-name', 'pathfinder',
    #         '-allow_renaming', 'true' ,
    #         '-x', x_pose,
    #         '-y', y_pose,
    #         '-z', '0.01'
    #     ],
    # )



    # Config file path
    bridge_params = os.path.join(pkg_pathfinder,'config','ros_gz_bridge.yaml')
    
    start_gazebo_ros_bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '--ros-args',
            '-p',
            f'config_file:={bridge_params}',
        ],
        output='screen',
    )


    start_gazebo_ros_image_bridge_cmd = Node(
        package='ros_gz_image',
        executable='image_bridge',
        arguments=['/camera/image_raw'],
        output='screen',
    )

    ld = LaunchDescription()

    # Add the commands to the launch description
    ld.add_action(gzserver_cmd)
    ld.add_action(gzclient_cmd)
    ld.add_action(spawn_turtlebot_cmd)
    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(start_gazebo_ros_bridge_cmd)
    ld.add_action(set_env_vars_resources)
    ld.add_action(start_gazebo_ros_image_bridge_cmd)

    return ld
