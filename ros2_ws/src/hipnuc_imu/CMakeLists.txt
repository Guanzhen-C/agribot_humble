cmake_minimum_required(VERSION 3.8)
project(hipnuc_imu)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

add_executable(hipnuc_imu_node src/hipnuc_imu_node.cpp)
ament_target_dependencies(hipnuc_imu_node rclcpp sensor_msgs)

install(TARGETS hipnuc_imu_node DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch udev DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
