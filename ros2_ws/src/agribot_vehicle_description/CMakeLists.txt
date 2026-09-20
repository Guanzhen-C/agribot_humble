cmake_minimum_required(VERSION 3.8)
project(agribot_vehicle_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)

if(BUILD_TESTING)
  add_test(
    NAME test_algorithm_matrix
    COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/test/test_algorithm_matrix.py
  )
  set_tests_properties(test_algorithm_matrix PROPERTIES TIMEOUT 90)
endif()

install(DIRECTORY config generated launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/unity_sync_and_simulate.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
