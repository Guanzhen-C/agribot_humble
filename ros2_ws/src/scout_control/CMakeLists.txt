cmake_minimum_required(VERSION 3.8)
project(scout_control)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/cmd_vel_mux.py
  scripts/joystick_translator.py
  scripts/scout_control_translator.py
  scripts/teleop_keyboard.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
