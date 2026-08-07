cmake_minimum_required(VERSION 3.8)
project(agribot_ackermann_mppi)

set(_system_protobuf_lib_dir "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
if(EXISTS "${_system_protobuf_lib_dir}/libprotobuf.so")
  # Gazebo 11 on Ubuntu focal is built against the system protobuf ABI.
  set(Protobuf_INCLUDE_DIR "/usr/include" CACHE PATH "" FORCE)
  set(Protobuf_LIBRARY "${_system_protobuf_lib_dir}/libprotobuf.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_LIBRARY_RELEASE "${_system_protobuf_lib_dir}/libprotobuf.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_LITE_LIBRARY "${_system_protobuf_lib_dir}/libprotobuf-lite.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_LITE_LIBRARY_RELEASE "${_system_protobuf_lib_dir}/libprotobuf-lite.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc" CACHE FILEPATH "" FORCE)
  set(Protobuf_PROTOC_LIBRARY "${_system_protobuf_lib_dir}/libprotoc.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_PROTOC_LIBRARY_RELEASE "${_system_protobuf_lib_dir}/libprotoc.so" CACHE FILEPATH "" FORCE)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

set(_system_protobuf_include_shim "${CMAKE_CURRENT_BINARY_DIR}/system_protobuf_include")
file(MAKE_DIRECTORY "${_system_protobuf_include_shim}")
file(REMOVE "${_system_protobuf_include_shim}/google")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/include/google
          "${_system_protobuf_include_shim}/google"
)

add_library(agribot_ackermann_mppi_drive SHARED
  src/ackermann_drive_plugin.cpp
)

target_include_directories(agribot_ackermann_mppi_drive PRIVATE
  "${_system_protobuf_include_shim}"
  ${gazebo_dev_INCLUDE_DIRS}
)

ament_target_dependencies(agribot_ackermann_mppi_drive
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  tf2_ros
  gazebo_ros
)

target_link_libraries(agribot_ackermann_mppi_drive
  ${gazebo_dev_LIBRARIES}
)

install(TARGETS
  agribot_ackermann_mppi_drive
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/ackermann_gazebo_sim.py
  scripts/sim_navsat_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  behavior_trees
  config
  launch
  models
  urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
