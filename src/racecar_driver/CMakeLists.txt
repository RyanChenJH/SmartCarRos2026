cmake_minimum_required(VERSION 3.5)
project(racecar_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

SET(CMAKE_EXE_LINKER_FLAGS " -no-pie")

include_directories(
  include
)

add_executable(${PROJECT_NAME}_node src/racecar_driver_node.cpp)
add_executable(${PROJECT_NAME}_node_one src/racecar_driver_node_one.cpp)

ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  geometry_msgs
)

ament_target_dependencies(${PROJECT_NAME}_node_one
  rclcpp
  geometry_msgs
)

target_link_libraries(${PROJECT_NAME}_node
  ${PROJECT_SOURCE_DIR}/lib/libracecar_driver.a
)

target_link_libraries(${PROJECT_NAME}_node_one
  ${PROJECT_SOURCE_DIR}/lib/libracecar_driver.a
)



install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  ${PROJECT_NAME}_node_one
  DESTINATION lib/${PROJECT_NAME}
)



ament_package()