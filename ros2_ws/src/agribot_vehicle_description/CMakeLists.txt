cmake_minimum_required(VERSION 3.8)
project(agribot_vehicle_description)

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(dwb_core REQUIRED)
find_package(dwb_plugins REQUIRED)
find_package(pluginlib REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_regulated_pure_pursuit_controller REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
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

add_library(agribot_simulation_visual_rotor SHARED src/simulation_visual_rotor.cpp)
target_compile_features(agribot_simulation_visual_rotor PRIVATE cxx_std_17)
target_include_directories(agribot_simulation_visual_rotor PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(agribot_simulation_visual_rotor ${GAZEBO_LIBRARIES})
install(TARGETS agribot_simulation_visual_rotor LIBRARY DESTINATION lib)

find_library(GAZEBO_WHEEL_SLIP_PLUGIN NAMES WheelSlipPlugin
  PATHS ${GAZEBO_PLUGIN_PATH} REQUIRED)
add_library(agribot_simulation_tracked_wheel_slip SHARED src/simulation_tracked_wheel_slip.cpp)
target_compile_features(agribot_simulation_tracked_wheel_slip PRIVATE cxx_std_17)
target_include_directories(agribot_simulation_tracked_wheel_slip PRIVATE ${GAZEBO_INCLUDE_DIRS})
target_link_libraries(agribot_simulation_tracked_wheel_slip
  ${GAZEBO_LIBRARIES} ${GAZEBO_WHEEL_SLIP_PLUGIN})
install(TARGETS agribot_simulation_tracked_wheel_slip LIBRARY DESTINATION lib)

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

add_library(agribot_route_progress_goal_checker SHARED src/route_progress_goal_checker.cpp)
target_compile_features(agribot_route_progress_goal_checker PUBLIC cxx_std_17)
ament_target_dependencies(agribot_route_progress_goal_checker nav2_core nav2_costmap_2d pluginlib std_msgs geometry_msgs)
pluginlib_export_plugin_description_file(nav2_core goal_checker_plugins.xml)
install(TARGETS agribot_route_progress_goal_checker LIBRARY DESTINATION lib)

add_library(agribot_position_only_rpp_controller SHARED src/position_only_rpp_controller.cpp)
target_compile_features(agribot_position_only_rpp_controller PUBLIC cxx_std_17)
ament_target_dependencies(agribot_position_only_rpp_controller
  nav2_core nav2_regulated_pure_pursuit_controller pluginlib)
pluginlib_export_plugin_description_file(nav2_core position_only_rpp_plugins.xml)
install(TARGETS agribot_position_only_rpp_controller LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(nav2_behavior_tree REQUIRED)
  find_package(ament_index_cpp REQUIRED)
  ament_add_gtest(test_tracked_slip_policy test/test_tracked_slip_policy.cpp)
  target_compile_features(test_tracked_slip_policy PRIVATE cxx_std_17)
  target_include_directories(test_tracked_slip_policy PRIVATE src)
  ament_add_gtest(test_initial_plan_recovery test/test_initial_plan_recovery.cpp)
  target_compile_features(test_initial_plan_recovery PUBLIC cxx_std_17)
  target_compile_definitions(test_initial_plan_recovery PRIVATE
    AGRIBOT_TEST_BT_DIR="${CMAKE_CURRENT_SOURCE_DIR}/config/behavior_trees")
  ament_target_dependencies(test_initial_plan_recovery nav2_behavior_tree ament_index_cpp)
  ament_add_gtest(test_route_progress_goal_checker test/test_route_progress_goal_checker.cpp)
  target_compile_features(test_route_progress_goal_checker PUBLIC cxx_std_17)
  ament_target_dependencies(test_route_progress_goal_checker nav2_core nav2_costmap_2d pluginlib std_msgs geometry_msgs)
  ament_add_gtest(test_position_terminal_stop test/test_position_terminal_stop.cpp)
  target_compile_features(test_position_terminal_stop PUBLIC cxx_std_17)
  ament_target_dependencies(test_position_terminal_stop nav2_core pluginlib)
  ament_add_gtest(test_position_only_rpp test/test_position_only_rpp.cpp)
  target_compile_features(test_position_only_rpp PUBLIC cxx_std_17)
  target_include_directories(test_position_only_rpp PRIVATE src)
  ament_target_dependencies(test_position_only_rpp
    nav2_core nav2_regulated_pure_pursuit_controller pluginlib)
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
  ament_add_pytest_test(test_simulation_implement_policy
    test/test_simulation_implement_policy.py TIMEOUT 90)
  ament_add_pytest_test(test_simulation_tracked_contact
    test/test_simulation_tracked_contact.py TIMEOUT 60)
  ament_add_pytest_test(test_simulation_terminal_stop
    test/test_simulation_terminal_stop.py TIMEOUT 180)
endif()

install(DIRECTORY config generated launch DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/unity_sync_and_simulate.sh
  scripts/verify_algorithm_runtime.py
  scripts/simulation_lidar_time_adapter.py
  scripts/simulation_vehicle_cloud_filter.py
  scripts/simulation_initial_pose.py
  scripts/simulation_visual_belts.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
