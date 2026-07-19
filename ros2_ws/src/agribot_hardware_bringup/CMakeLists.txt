cmake_minimum_required(VERSION 3.8)
project(agribot_hardware_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/configure_c16_network.sh
  scripts/rtk_nmea_node.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
