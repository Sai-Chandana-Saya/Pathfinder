### Pathfinder
Robot SLAM and Autonomous Navigation


## Requirements
1. Ubunut 24.04
2. ROS2 Jazzy

## Setting up the ROS2 workspace
# 1. Cloning the repository into ros2_ws
``` bash
mkdir -p ros2_ws
cd ros2_ws
git clone https://github.com/Sai-Chandana-Saya/Pathfinder.git .
```

# 2. Building and Sourcing the workspace
```bash
colcon build
source install/setup.bash
```

## SLAM
# Launching the robot in the Gazebo world
```bash
ros2 launch pathfinder my_robot_world.launch.py
```
# Start a Navigation launch file
```bash
ros2 launch pathfinder navigation_launch.py use_sim_time:=True
```
# Start SLAM with slam_toolbox
```bash
ros2 launch pathfinder online_async_launch.py
```
# Start Rviz
```bash
ros2 run rviz2 rviz2
```
# Control the robot
```bash
ros2 run pathfinder teleop_keyboard_twist.py
```
# Save the map
```bash
ros2 run nav2_map_server map_server_cli -f ~/map
```


