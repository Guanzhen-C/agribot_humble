cmake_minimum_required(VERSION 3.8)
project(agribot_hardware_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scout_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(CHASSIS_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/can/include
  ${CMAKE_CURRENT_SOURCE_DIR}/differential/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ackermann/include
)

add_library(chassis_can_common
  can/src/chassis_can_common.cpp
  can/src/chassis_can_node.cpp
)
target_include_directories(chassis_can_common PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(chassis_can_common
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  scout_msgs
  std_msgs
)

add_library(differential_chassis_can
  differential/src/differential_can_protocol.cpp
  differential/src/differential_chassis_adapter.cpp
)
target_include_directories(differential_chassis_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/differential/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(differential_chassis_can chassis_can_common)
ament_target_dependencies(differential_chassis_can geometry_msgs rclcpp scout_msgs)

add_library(ackermann_chassis_can
  ackermann/src/ackermann_can_protocol.cpp
  ackermann/src/ackermann_chassis_adapter.cpp
)
target_include_directories(ackermann_chassis_can PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ackermann/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ackermann_chassis_can chassis_can_common)
ament_target_dependencies(ackermann_chassis_can geometry_msgs rclcpp scout_msgs)

add_executable(
  differential_chassis_can_node
  differential/src/differential_chassis_main.cpp
)
target_include_directories(differential_chassis_can_node PRIVATE ${CHASSIS_INCLUDE_DIRS})
target_link_libraries(
  differential_chassis_can_node
  chassis_can_common
  differential_chassis_can
)

add_executable(
  ackermann_chassis_can_node
  ackermann/src/ackermann_chassis_main.cpp
)
target_include_directories(ackermann_chassis_can_node PRIVATE ${CHASSIS_INCLUDE_DIRS})
target_link_libraries(
  ackermann_chassis_can_node
  chassis_can_common
  ackermann_chassis_can
)

add_executable(
  rtk_eskf_localization
  localization/navsat/src/rtk_eskf_localization_node.cpp
  localization/navsat/src/rtk_gi_engine.cpp
  localization/navsat/third_party/kf_gins/kf-gins/insmech.cpp
)
set_source_files_properties(
  localization/navsat/third_party/kf_gins/kf-gins/insmech.cpp
  PROPERTIES COMPILE_OPTIONS "-Wno-unused-parameter"
)
target_include_directories(rtk_eskf_localization PRIVATE
  localization/navsat/include
  localization/navsat/third_party/kf_gins
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(rtk_eskf_localization Eigen3::Eigen)
ament_target_dependencies(rtk_eskf_localization
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
)

install(PROGRAMS
  scripts/configure_can.sh
  scripts/configure_c16_network.sh
  scripts/rtk_nmea_node.py
  scripts/vehicle_command_gate.py
  scripts/vehicle_preflight.py
  localization/fastlio/scripts/fastlio_odom_bridge.py
  localization/navsat/scripts/navsat_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(DIRECTORY
  differential/config
  DESTINATION share/${PROJECT_NAME}/differential
)
install(DIRECTORY
  ackermann/config
  ackermann/behavior_trees
  DESTINATION share/${PROJECT_NAME}/ackermann
)
install(DIRECTORY differential/launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
install(DIRECTORY ackermann/launch/ DESTINATION share/${PROJECT_NAME}/launch
  FILES_MATCHING PATTERN "*.launch.py"
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES LICENSE DESTINATION share/${PROJECT_NAME})
install(FILES
  localization/navsat/third_party/kf_gins/LICENSE
  localization/navsat/third_party/kf_gins/README.md
  DESTINATION share/${PROJECT_NAME}/third_party/kf_gins
)
install(FILES differential/README.md DESTINATION share/${PROJECT_NAME}/differential)
install(FILES ackermann/README.md DESTINATION share/${PROJECT_NAME}/ackermann)
install(DIRECTORY can/include/ DESTINATION include)
install(DIRECTORY differential/include/ DESTINATION include)
install(DIRECTORY ackermann/include/ DESTINATION include)
install(DIRECTORY localization/navsat/include/ DESTINATION include)
install(TARGETS
  chassis_can_common
  differential_chassis_can
  ackermann_chassis_can
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  differential_chassis_can_node
  ackermann_chassis_can_node
  rtk_eskf_localization
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  chassis_can_common
  differential_chassis_can
  ackermann_chassis_can
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(
    test_differential_can_protocol
    differential/test/test_differential_can_protocol.cpp
  )
  target_link_libraries(test_differential_can_protocol differential_chassis_can)
  ament_add_gtest(
    test_ackermann_can_protocol
    ackermann/test/test_ackermann_can_protocol.cpp
  )
  target_link_libraries(test_ackermann_can_protocol ackermann_chassis_can)
  ament_add_pytest_test(test_launch_validation test/test_launch_validation.py)
  ament_add_pytest_test(test_navigation_configs test/test_navigation_configs.py)
  ament_add_pytest_test(test_rtk_nmea test/test_rtk_nmea.py)
  ament_add_pytest_test(
    test_vehicle_directory_layout
    test/test_vehicle_directory_layout.py
  )
  ament_add_pytest_test(
    test_self_contained_runtime
    test/test_self_contained_runtime.py
  )
  ament_add_pytest_test(test_vehicle_safety test/test_vehicle_safety.py)
endif()

ament_package()
