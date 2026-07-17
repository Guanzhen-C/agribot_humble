cmake_minimum_required(VERSION 3.8)
project(scout_gazebo)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/odom_to_ground_truth.py
  scripts/simulated_joint_state_publisher.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch meshes urdf worlds
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
