cmake_minimum_required(VERSION 3.14)
project(livox_ros_driver2 VERSION 1.2.6 LANGUAGES C CXX)

set(CMAKE_C_STANDARD 99)
set(CMAKE_C_STANDARD_REQUIRED ON)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Wno-unused-parameter)
endif()

find_package(ament_cmake REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcutils REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
pkg_check_modules(APR REQUIRED apr-1)

set(interface_target "${PROJECT_NAME}_interfaces")
rosidl_generate_interfaces(${interface_target}
  "msg/CustomPoint.msg"
  "msg/CustomMsg.msg"
  DEPENDENCIES builtin_interfaces std_msgs
)

# Build the pinned Livox-SDK2 source with the driver so no global /usr/local
# installation is required.
add_subdirectory(3rdparty/Livox-SDK2/sdk_core livox_sdk2)

set(driver_sources
  src/livox_ros_driver2.cpp
  src/lddc.cpp
  src/driver_node.cpp
  src/lds.cpp
  src/lds_lidar.cpp
  src/comm/comm.cpp
  src/comm/ldq.cpp
  src/comm/semaphore.cpp
  src/comm/lidar_imu_data_queue.cpp
  src/comm/cache_index.cpp
  src/comm/pub_handler.cpp
  src/parse_cfg_file/parse_cfg_file.cpp
  src/parse_cfg_file/parse_livox_lidar_cfg.cpp
  src/call_back/lidar_common_callback.cpp
  src/call_back/livox_lidar_callback.cpp
)

add_library(${PROJECT_NAME} SHARED ${driver_sources})
target_include_directories(${PROJECT_NAME} PRIVATE
  3rdparty
  src
  ${APR_INCLUDE_DIRS}
)
ament_target_dependencies(${PROJECT_NAME}
  rcl_interfaces
  rclcpp
  rclcpp_components
  rcutils
  sensor_msgs
  std_msgs
)

rosidl_get_typesupport_target(cpp_typesupport_target
  ${interface_target} "rosidl_typesupport_cpp")
target_link_libraries(${PROJECT_NAME}
  livox_lidar_sdk_shared
  "${cpp_typesupport_target}"
  ${APR_LIBRARIES}
)

rclcpp_components_register_node(${PROJECT_NAME}
  PLUGIN "livox_ros::DriverNode"
  EXECUTABLE ${PROJECT_NAME}_node
)

install(TARGETS ${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch_ROS2/
  DESTINATION share/${PROJECT_NAME}/launch
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
