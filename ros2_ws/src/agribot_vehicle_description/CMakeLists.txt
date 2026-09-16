cmake_minimum_required(VERSION 3.8)
project(agribot_vehicle_description)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config generated launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/unity_sync_and_simulate.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
