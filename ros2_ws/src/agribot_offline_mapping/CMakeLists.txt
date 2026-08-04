cmake_minimum_required(VERSION 3.8)
project(agribot_offline_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(agribot_hardware_bringup REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED IMPORTED_TARGET geographiclib)

add_executable(lslidar_lio_sam_adapter src/lslidar_lio_sam_adapter.cpp)
ament_target_dependencies(lslidar_lio_sam_adapter rclcpp sensor_msgs)

add_executable(rtk_odometry_adapter src/rtk_odometry_adapter.cpp)
target_link_libraries(rtk_odometry_adapter Eigen3::Eigen PkgConfig::GEOGRAPHICLIB)
ament_target_dependencies(
  rtk_odometry_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(map_georeference_exporter src/map_georeference_exporter.cpp)
target_include_directories(map_georeference_exporter PRIVATE include)
target_link_libraries(map_georeference_exporter Eigen3::Eigen yaml-cpp)
ament_target_dependencies(
  map_georeference_exporter
  agribot_hardware_bringup
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS
  lslidar_lio_sam_adapter
  rtk_odometry_adapter
  map_georeference_exporter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/finalize_lio_sam_map.py DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES third_party.repos README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_georeference_fit test/test_georeference_fit.cpp)
  target_include_directories(test_georeference_fit PRIVATE include)
  target_link_libraries(test_georeference_fit Eigen3::Eigen)
  ament_add_pytest_test(test_finalize_lio_sam_map test/test_finalize_lio_sam_map.py)
  ament_add_pytest_test(test_offline_contracts test/test_offline_contracts.py)
endif()

ament_export_include_directories(include)
ament_package()
