cmake_minimum_required(VERSION 3.8)
project(agribot_hardware_bringup)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/configure_can.sh
  scripts/configure_c16_network.sh
  scripts/rtk_nmea_node.py
  scripts/vehicle_command_gate.py
  scripts/vehicle_preflight.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES README.md DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_rtk_nmea test/test_rtk_nmea.py)
  ament_add_pytest_test(test_vehicle_safety test/test_vehicle_safety.py)
endif()

ament_package()
