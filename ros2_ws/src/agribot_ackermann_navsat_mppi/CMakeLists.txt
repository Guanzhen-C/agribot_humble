cmake_minimum_required(VERSION 3.8)
project(agribot_ackermann_navsat_mppi)

find_package(ament_cmake REQUIRED)

install(DIRECTORY config launch DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
