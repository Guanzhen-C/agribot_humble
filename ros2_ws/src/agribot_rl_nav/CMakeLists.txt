cmake_minimum_required(VERSION 3.8)
project(agribot_rl_nav)

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(noah_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_executable(rtk_eskf_localization
  src/rtk_eskf_localization_node.cpp
  src/rtk_gi_engine.cpp
  ${CMAKE_CURRENT_SOURCE_DIR}/../KF-GINS/src/kf-gins/insmech.cpp
)

target_include_directories(rtk_eskf_localization PRIVATE
  include
  ${CMAKE_CURRENT_SOURCE_DIR}/../KF-GINS/src
  ${EIGEN3_INCLUDE_DIRS}
)

target_link_libraries(rtk_eskf_localization Eigen3::Eigen)

ament_target_dependencies(rtk_eskf_localization
  geometry_msgs
  nav_msgs
  noah_msgs
  rclcpp
  sensor_msgs
)

install(TARGETS
  rtk_eskf_localization
  DESTINATION lib/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/depth_rl_data_collector.py
  scripts/depth_rl_policy_node.py
  scripts/ground_truth_scan_mapper.py
  scripts/odometry_tf_broadcaster.py
  scripts/navsat_to_local_odom.py
  scripts/navsat_pose_bridge.py
  scripts/navsat_truth_error_monitor.py
  scripts/fastlio_compare.py
  scripts/fastlio_yaw_diagnose.py
  scripts/train_depth_bc.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
