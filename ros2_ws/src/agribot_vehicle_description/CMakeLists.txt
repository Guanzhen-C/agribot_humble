cmake_minimum_required(VERSION 3.8)
project(agribot_vehicle_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(gazebo REQUIRED)

add_library(agribot_simulation_sensor_collision_filter SHARED
  src/simulation_sensor_collision_filter.cpp)
target_compile_features(agribot_simulation_sensor_collision_filter PRIVATE cxx_std_17)
target_include_directories(agribot_simulation_sensor_collision_filter PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(agribot_simulation_sensor_collision_filter ${GAZEBO_LIBRARIES})
install(TARGETS agribot_simulation_sensor_collision_filter LIBRARY DESTINATION lib)

add_library(agribot_simulation_sensor_visual SHARED src/simulation_sensor_visual.cpp)
target_compile_features(agribot_simulation_sensor_visual PRIVATE cxx_std_17)
target_include_directories(agribot_simulation_sensor_visual PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(agribot_simulation_sensor_visual ${GAZEBO_LIBRARIES})
install(TARGETS agribot_simulation_sensor_visual LIBRARY DESTINATION lib)

option(AGRIBOT_BUILD_GAZEBO_TRANSPORT_COMPAT
  "Build the scoped Gazebo 11.10.2 / oneTBB shutdown fix" ON)
if(AGRIBOT_BUILD_GAZEBO_TRANSPORT_COMPAT)
  add_subdirectory(compat/gazebo_transport)
endif()

add_library(agribot_ackermann_trajectory_generator SHARED
  src/ackermann_trajectory_generator.cpp)
target_compile_features(agribot_ackermann_trajectory_generator PUBLIC cxx_std_17)
ament_target_dependencies(agribot_ackermann_trajectory_generator dwb_core dwb_plugins pluginlib)
pluginlib_export_plugin_description_file(dwb_core trajectory_plugins.xml)
install(TARGETS agribot_ackermann_trajectory_generator LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_ackermann_trajectory_generator test/test_ackermann_trajectory_generator.cpp)
  target_compile_features(test_ackermann_trajectory_generator PUBLIC cxx_std_17)
  ament_target_dependencies(test_ackermann_trajectory_generator dwb_core dwb_plugins pluginlib)
  if(TARGET agribot_gazebo_transport_compat)
    find_package(TBB REQUIRED)
    ament_add_gtest(test_gazebo_io_lifetime test/test_gazebo_io_lifetime.cpp TIMEOUT 30
      ENV "LD_LIBRARY_PATH=${CMAKE_CURRENT_BINARY_DIR}/compat/gazebo_transport/lib:$ENV{LD_LIBRARY_PATH}")
    target_compile_features(test_gazebo_io_lifetime PRIVATE cxx_std_17)
    target_include_directories(test_gazebo_io_lifetime PRIVATE
      $<TARGET_PROPERTY:agribot_gazebo_transport_compat,INCLUDE_DIRECTORIES>)
    target_link_libraries(test_gazebo_io_lifetime agribot_gazebo_transport_compat TBB::tbb)
  endif()
  ament_add_pytest_test(test_algorithm_matrix
    test/test_algorithm_matrix.py TIMEOUT 90)
  ament_add_pytest_test(test_runtime_acceptance_helpers
    test/test_runtime_acceptance_helpers.py TIMEOUT 60)
  ament_add_pytest_test(test_bridge_shutdown
    test/test_bridge_shutdown.py TIMEOUT 60)
  ament_add_pytest_test(test_simulation_lidar_time
    test/test_simulation_lidar_time.py TIMEOUT 60)
  ament_add_pytest_test(test_simulation_vehicle_cloud_filter
    test/test_simulation_vehicle_cloud_filter.py TIMEOUT 60)
  ament_add_pytest_test(test_simulation_initial_pose
    test/test_simulation_initial_pose.py TIMEOUT 60)
endif()

install(DIRECTORY config generated launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/unity_sync_and_simulate.sh
  scripts/verify_algorithm_runtime.py
  scripts/simulation_lidar_time_adapter.py
  scripts/simulation_vehicle_cloud_filter.py
  scripts/simulation_initial_pose.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
