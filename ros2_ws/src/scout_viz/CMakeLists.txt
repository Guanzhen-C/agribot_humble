cmake_minimum_required(VERSION 3.8)
project(scout_viz)

find_package(ament_cmake REQUIRED)

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/run_rviz2_clean_env.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
