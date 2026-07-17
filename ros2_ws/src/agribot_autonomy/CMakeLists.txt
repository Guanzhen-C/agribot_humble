cmake_minimum_required(VERSION 3.8)
project(agribot_autonomy)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/fastlio_odom_bridge.py
  scripts/ground_truth_display_bridge.py
  scripts/ground_truth_localization.py
  scripts/ground_truth_printer.py
  scripts/imu_frame_bridge.py
  scripts/initial_pose_sender.py
  scripts/kiss_localization.py
  scripts/localization_pose_printer.py
  scripts/pointcloud_frame_relay.py
  scripts/snake_waypoint_runner.py
  scripts/topic_ready_gate.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  maps
  rviz
  behavior_trees
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
