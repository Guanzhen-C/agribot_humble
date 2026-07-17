cmake_minimum_required(VERSION 3.8)
project(scout_base)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scout_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(ugv_sdk REQUIRED)

add_executable(scout_base_node src/scout_base_node.cpp)
ament_target_dependencies(scout_base_node
  geometry_msgs
  nav_msgs
  rclcpp
  scout_msgs
  tf2
  tf2_ros
  ugv_sdk
)
target_link_libraries(scout_base_node ugv_sdk::ugv_sdk)

install(TARGETS scout_base_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
