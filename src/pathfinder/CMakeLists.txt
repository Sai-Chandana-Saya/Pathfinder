cmake_minimum_required(VERSION 3.8)
project(pathfinder)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)

# Find other dependencies your nodes need
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)

# Install directories
install(
  DIRECTORY urdf launch meshes rviz config maps worlds
  DESTINATION share/${PROJECT_NAME}/
)

# Install Python scripts as executables - FIXED PATH
install(PROGRAMS
  src/teleop_keyboard_twistamped.py
  src/teleop_keyboard_twist.py
  src/cmd_vel_relay.py
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files
install(DIRECTORY
  launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()