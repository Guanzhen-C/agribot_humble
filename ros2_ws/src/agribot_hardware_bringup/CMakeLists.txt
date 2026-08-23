cmake_minimum_required(VERSION 3.8)
project(agribot_hardware_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geographic_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(GTSAM REQUIRED)
find_package(GTSAM_UNSTABLE REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(PCL REQUIRED COMPONENTS
  common
  features
  filters
  io
  kdtree
  registration
  search
)
find_package(pcl_conversions REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(scout_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED IMPORTED_TARGET geographiclib)

set(CHASSIS_INCLUDE_DIRS
  ${CMAKE_CURRENT_SOURCE_DIR}/can/include
  ${CMAKE_CURRENT_SOURCE_DIR}/differential/include
  ${CMAKE_CURRENT_SOURCE_DIR}/ackermann/include
)

add_library(chassis_can_common
  can/src/chassis_can_common.cpp
  can/src/chassis_can_node.cpp
  can/src/chassis_can_transport.cpp
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

add_library(ackermann_chassis_serial
  ackermann/src/ackermann_serial_protocol.cpp
)
target_include_directories(ackermann_chassis_serial PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/can/include>
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/ackermann/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(ackermann_chassis_serial ackermann_chassis_can)

add_library(semantic_proximity_layer SHARED
  semantic/src/semantic_proximity_layer.cpp
)
target_include_directories(semantic_proximity_layer PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/semantic/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(semantic_proximity_layer
  nav2_costmap_2d
  nav_msgs
  pluginlib
  rclcpp
)
pluginlib_export_plugin_description_file(
  nav2_costmap_2d semantic_proximity_layer.xml
)

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
  ackermann_chassis_serial_node
  ackermann/src/ackermann_serial_node.cpp
)
target_include_directories(ackermann_chassis_serial_node PRIVATE ${CHASSIS_INCLUDE_DIRS})
target_link_libraries(
  ackermann_chassis_serial_node
  ackermann_chassis_can
  ackermann_chassis_serial
)
ament_target_dependencies(ackermann_chassis_serial_node
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  scout_msgs
  sensor_msgs
  std_msgs
)

add_executable(
  ackermann_joint_state_publisher
  ackermann/src/ackermann_joint_state_publisher.cpp
)
ament_target_dependencies(ackermann_joint_state_publisher
  nav_msgs
  rclcpp
  sensor_msgs
)

add_executable(
  camera_trigger_pps_lock
  time_sync/src/camera_trigger_pps_lock.cpp
)

add_executable(
  rtk_eskf_localization
  localization/navsat/src/rtk_eskf_localization_node.cpp
  localization/navsat/src/rtk_gi_engine.cpp
  localization/navsat/third_party/kf_gins/kf-gins/insmech.cpp
)

add_executable(
  rtk_map_initializer
  localization/navsat/src/rtk_map_initializer.cpp
)
target_include_directories(rtk_map_initializer PRIVATE
  localization/navsat/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(
  rtk_map_initializer
  Eigen3::Eigen
  PkgConfig::GEOGRAPHICLIB
  yaml-cpp
)
ament_target_dependencies(rtk_map_initializer
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(
  georeference_test_bridge
  localization/navsat/src/georeference_test_bridge.cpp
)
target_include_directories(georeference_test_bridge PRIVATE
  localization/navsat/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(
  georeference_test_bridge
  Eigen3::Eigen
  PkgConfig::GEOGRAPHICLIB
  yaml-cpp
)
ament_target_dependencies(georeference_test_bridge
  geographic_msgs
  geometry_msgs
  rclcpp
  std_msgs
)

add_executable(
  fastlivo_rtk_fusion
  localization/fusion/src/fastlivo_rtk_fusion_node.cpp
)
target_include_directories(fastlivo_rtk_fusion PRIVATE
  localization/fusion/include
  localization/navsat/include
  ${EIGEN3_INCLUDE_DIRS}
)
target_link_libraries(
  fastlivo_rtk_fusion
  Eigen3::Eigen
  PkgConfig::GEOGRAPHICLIB
  gtsam
  gtsam_unstable
  yaml-cpp
)
ament_target_dependencies(fastlivo_rtk_fusion
  diagnostic_msgs
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
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

add_executable(
  pcd_map_builder
  localization/pcd/src/pcd_map_builder.cpp
)
target_include_directories(pcd_map_builder SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_map_builder Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(pcd_map_builder
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_srvs
  tf2_ros
)

add_executable(
  pcd_initial_localizer
  localization/pcd/src/pcd_initial_localizer.cpp
)
target_include_directories(pcd_initial_localizer SYSTEM PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(pcd_initial_localizer Eigen3::Eigen ${PCL_LIBRARIES})
ament_target_dependencies(pcd_initial_localizer
  geometry_msgs
  nav_msgs
  pcl_conversions
  rclcpp
  sensor_msgs
  std_msgs
  tf2_ros
)

install(PROGRAMS
  scripts/configure_can.sh
  scripts/configure_c16_network.sh
  scripts/configure_camera_trigger_pwm.sh
  scripts/c16_pps_phase_tool.py
  scripts/check_camera_trigger_pwm.sh
  scripts/check_c16_gptp.sh
  scripts/check_rtk_time_sync.sh
  scripts/install_camera_trigger_pwm.sh
  scripts/install_c16_gptp.sh
  scripts/install_rtk_time_sync.sh
  scripts/initialization_coordinator.py
  scripts/outdoor_stage_check.py
  scripts/pcd_to_nav2_map.py
  scripts/planner_validation_bridge.py
  scripts/planner_validation_follow_path.py
  scripts/publish_geodetic_pose.py
  scripts/visual_place_recognizer.py
  scripts/rtk_nmea_node.py
  scripts/rtk_time_feeder.py
  scripts/sensor_time_sync_monitor.py
  scripts/start_wheeltec_car_gui.sh
  scripts/wheeltec_car_gui.py
  localization/fastlio/scripts/fastlio_odom_bridge.py
  localization/navsat/scripts/navsat_pose_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch meshes models rviz systemd udev urdf DESTINATION share/${PROJECT_NAME}
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
install(FILES
  desktop/README.md
  desktop/wheeltec-car-gui.desktop
  DESTINATION share/${PROJECT_NAME}/desktop
)
install(DIRECTORY can/include/ DESTINATION include)
install(DIRECTORY differential/include/ DESTINATION include)
install(DIRECTORY ackermann/include/ DESTINATION include)
install(DIRECTORY localization/navsat/include/ DESTINATION include)
install(DIRECTORY localization/fusion/include/ DESTINATION include)
install(DIRECTORY semantic/include/ DESTINATION include)
install(TARGETS
  chassis_can_common
  differential_chassis_can
  ackermann_chassis_can
  ackermann_chassis_serial
  semantic_proximity_layer
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS
  differential_chassis_can_node
  ackermann_chassis_can_node
  ackermann_chassis_serial_node
  ackermann_joint_state_publisher
  camera_trigger_pps_lock
  rtk_eskf_localization
  rtk_map_initializer
  georeference_test_bridge
  fastlivo_rtk_fusion
  pcd_map_builder
  pcd_initial_localizer
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

ament_export_include_directories(include)
ament_export_libraries(
  chassis_can_common
  differential_chassis_can
  ackermann_chassis_can
  ackermann_chassis_serial
  semantic_proximity_layer
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
  ament_add_gtest(
    test_ackermann_serial_protocol
    ackermann/test/test_ackermann_serial_protocol.cpp
  )
  target_link_libraries(
    test_ackermann_serial_protocol
    ackermann_chassis_serial
  )
  ament_add_gtest(
    test_chassis_can_transport
    can/test/test_chassis_can_transport.cpp
  )
  target_link_libraries(test_chassis_can_transport chassis_can_common)
  ament_add_gtest(
    test_navsat_frame_conversions
    localization/navsat/test/test_navsat_frame_conversions.cpp
  )
  target_include_directories(test_navsat_frame_conversions PRIVATE
    localization/navsat/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_navsat_frame_conversions Eigen3::Eigen)
  ament_add_gtest(
    test_map_georeference
    localization/navsat/test/test_map_georeference.cpp
  )
  target_include_directories(test_map_georeference PRIVATE
    localization/navsat/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_map_georeference Eigen3::Eigen yaml-cpp)
  ament_add_gtest(
    test_georeference_test_conversions
    localization/navsat/test/test_georeference_test_conversions.cpp
  )
  target_include_directories(test_georeference_test_conversions PRIVATE
    localization/navsat/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_georeference_test_conversions Eigen3::Eigen)
  ament_add_gtest(
    test_horizontal_antenna_factor
    localization/fusion/test/test_horizontal_antenna_factor.cpp
  )
  target_include_directories(test_horizontal_antenna_factor PRIVATE
    localization/fusion/include
    ${EIGEN3_INCLUDE_DIRS}
  )
  target_link_libraries(test_horizontal_antenna_factor Eigen3::Eigen gtsam)
  ament_add_pytest_test(test_launch_validation test/test_launch_validation.py)
  ament_add_pytest_test(
    test_initialization_coordinator
    test/test_initialization_coordinator.py
  )
  ament_add_pytest_test(
    test_visual_place_recognizer
    test/test_visual_place_recognizer.py
  )
  ament_add_pytest_test(test_navigation_configs test/test_navigation_configs.py)
  ament_add_pytest_test(test_pcd_to_nav2_map test/test_pcd_to_nav2_map.py)
  ament_add_pytest_test(test_c16_gptp test/test_c16_gptp.py)
  ament_add_pytest_test(test_c16_pps_phase_tool test/test_c16_pps_phase_tool.py)
  ament_add_pytest_test(
    test_camera_trigger_pwm
    test/test_camera_trigger_pwm.py
  )
  ament_add_pytest_test(test_rtk_nmea test/test_rtk_nmea.py)
  ament_add_pytest_test(test_rtk_time_feeder test/test_rtk_time_feeder.py)
  ament_add_pytest_test(
    test_sensor_time_sync_monitor
    test/test_sensor_time_sync_monitor.py
  )
  ament_add_pytest_test(
    test_publish_geodetic_pose
    test/test_publish_geodetic_pose.py
  )
  ament_add_pytest_test(
    test_vehicle_directory_layout
    test/test_vehicle_directory_layout.py
  )
  ament_add_pytest_test(
    test_self_contained_runtime
    test/test_self_contained_runtime.py
  )
  ament_add_pytest_test(
    test_planner_validation
    test/test_planner_validation.py
  )
  ament_add_pytest_test(
    test_wheeltec_car_gui
    test/test_wheeltec_car_gui.py
  )
  ament_add_pytest_test(
    test_fastlivo_rtk_fusion
    test/test_fastlivo_rtk_fusion.py
  )
  ament_add_pytest_test(
    test_outdoor_0811_experiment
    test/test_outdoor_0811_experiment.py
  )
  ament_add_pytest_test(
    test_outdoor_stage_check
    test/test_outdoor_stage_check.py
  )
endif()

ament_package()
