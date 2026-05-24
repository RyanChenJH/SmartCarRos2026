cmake_minimum_required(VERSION 3.5)
project(racecar)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
include_directories(${tf2_ros_INCLUDE_DIRS})
find_package(tf2_geometry_msgs REQUIRED)
find_package(message_filters REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(nav2_amcl REQUIRED)
find_package(action_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)

set(dependencies
  geometry_msgs
  sensor_msgs
  rclcpp
  std_msgs
  visualization_msgs
  tf2
  tf2_ros
  tf2_geometry_msgs
  nav_msgs
  message_filters
  ament_cmake
  nav2_amcl
  action_msgs
  nav2_msgs
  std_srvs
)

# Include Cpp "include" directory
include_directories(include)

# Create Cpp executable
# add_executable(car_controller_new src/car_controller_new.cpp)
add_executable(laser_2d_new src/laser_2d_new.cpp)
add_executable(my_car_control_new src/my_car_control_new.cpp)
add_executable(scan_node src/scan_node.cpp)
add_executable(car_control_all src/car_control_all.cpp)
# ament_target_dependencies(car_controller_new ${dependencies})
ament_target_dependencies(laser_2d_new ${dependencies})
ament_target_dependencies(my_car_control_new ${dependencies})
ament_target_dependencies(scan_node ${dependencies})
ament_target_dependencies(car_control_all ${dependencies})

# Install Cpp executables
install(TARGETS
#  car_controller_new
  laser_2d_new
  my_car_control_new
  scan_node
  car_control_all
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  scripts/racecar_teleop.py
  scripts/racecar_helper.py
  scripts/race_manager.py
  scripts/get.py
  scripts/detct.py
  scripts/go.py
  scripts/line_follow.py
  scripts/line_follow2.py
  scripts/zhuitong.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  map
  maps
  DESTINATION share/${PROJECT_NAME}
)
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)


ament_package()