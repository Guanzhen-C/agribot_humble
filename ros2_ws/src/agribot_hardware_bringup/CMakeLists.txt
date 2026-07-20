cmake_minimum_required(VERSION 3.8)
project(agribot_hardware_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scout_msgs REQUIRED)
find_package(std_msgs REQUIRED)

add_library(chassis_can_protocol src/chassis_can_protocol.cpp)
target_include_directories(chassis_can_protocol PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(chassis_can_node src/chassis_can_node.cpp)
target_link_libraries(chassis_can_node chassis_can_protocol)
ament_target_dependencies(chassis_can_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  scout_msgs
  std_msgs
)

install(PROGRAMS
  scripts/configure_can.sh
  scripts/configure_c16_network.sh
  scripts/rtk_nmea_node.py
  scripts/vehicle_command_gate.py
  scripts/vehicle_preflight.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
  PATTERN "*.pyc" EXCLUDE
)
install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)
install(TARGETS chassis_can_protocol
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS chassis_can_node
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(chassis_can_protocol)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_chassis_can_protocol test/test_chassis_can_protocol.cpp)
  target_link_libraries(test_chassis_can_protocol chassis_can_protocol)
  ament_add_pytest_test(test_launch_validation test/test_launch_validation.py)
  ament_add_pytest_test(test_navigation_configs test/test_navigation_configs.py)
  ament_add_pytest_test(test_rtk_nmea test/test_rtk_nmea.py)
  ament_add_pytest_test(test_vehicle_safety test/test_vehicle_safety.py)
endif()

ament_package()
