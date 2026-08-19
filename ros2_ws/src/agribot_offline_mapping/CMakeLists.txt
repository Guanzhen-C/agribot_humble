cmake_minimum_required(VERSION 3.8)
project(agribot_offline_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(agribot_hardware_bringup REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(PkgConfig REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(yaml-cpp REQUIRED)
pkg_check_modules(GEOGRAPHICLIB REQUIRED IMPORTED_TARGET geographiclib)

add_executable(lslidar_lio_sam_adapter src/lslidar_lio_sam_adapter.cpp)
target_include_directories(lslidar_lio_sam_adapter PRIVATE include)
target_link_libraries(lslidar_lio_sam_adapter Eigen3::Eigen)
ament_target_dependencies(lslidar_lio_sam_adapter rclcpp sensor_msgs)

add_executable(rtk_odometry_adapter src/rtk_odometry_adapter.cpp)
target_include_directories(rtk_odometry_adapter PRIVATE include)
target_link_libraries(rtk_odometry_adapter Eigen3::Eigen PkgConfig::GEOGRAPHICLIB)
ament_target_dependencies(
  rtk_odometry_adapter
  geometry_msgs
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
)

add_executable(map_georeference_exporter src/map_georeference_exporter.cpp)
target_include_directories(map_georeference_exporter PRIVATE include)
target_link_libraries(map_georeference_exporter Eigen3::Eigen yaml-cpp)
ament_target_dependencies(
  map_georeference_exporter
  agribot_hardware_bringup
  nav_msgs
  rclcpp
  sensor_msgs
  std_msgs
  std_srvs
)

install(TARGETS
  lslidar_lio_sam_adapter
  rtk_odometry_adapter
  map_georeference_exporter
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS
  scripts/ollama_semantic_client.py
  scripts/build_semantic_model_context.py
  scripts/build_map_semantic_navigation_graph.py
  scripts/build_semantic_navigation_graph.py
  scripts/describe_opengraph_instances_ollama.py
  scripts/export_opengraph_dataset.py
  scripts/finalize_lio_sam_map.py
  scripts/import_semantic_graph_neo4j.py
  scripts/mapping_result_trajectory_publisher.py
  scripts/plan_semantic_route.py
  scripts/plan_semantic_task_bailian.py
  scripts/publish_opengraph_semantic_map.py
  scripts/publish_semantic_navigation_graph.py
  scripts/publish_semantic_route.py
  scripts/refine_semantic_navigation_graph.py
  scripts/run_indoor_fastlivo_fusion_validation.py
  scripts/run_localization_comparison.py
  scripts/run_rtk_mapping_pipeline.py
  scripts/semantic_graph_neo4j.py
  scripts/semantic_planner_server.py
  scripts/upsert_semantic_landmarks_neo4j.py
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config deploy launch rviz DESTINATION share/${PROJECT_NAME})
install(FILES third_party.repos README.md DESTINATION share/${PROJECT_NAME})
install(DIRECTORY include/ DESTINATION include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_gtest(test_georeference_fit test/test_georeference_fit.cpp)
  target_include_directories(test_georeference_fit PRIVATE include)
  target_link_libraries(test_georeference_fit Eigen3::Eigen)
  ament_add_gtest(test_c16_scan_time test/test_c16_scan_time.cpp)
  target_include_directories(test_c16_scan_time PRIVATE include)
  ament_add_gtest(test_rear_exclusion_filter test/test_rear_exclusion_filter.cpp)
  target_include_directories(test_rear_exclusion_filter PRIVATE include)
  target_link_libraries(test_rear_exclusion_filter Eigen3::Eigen)
  ament_add_gtest(test_rtk_heading_policy test/test_rtk_heading_policy.cpp)
  target_include_directories(test_rtk_heading_policy PRIVATE include)
  ament_add_gtest(test_rtk_lever_arm test/test_rtk_lever_arm.cpp)
  target_include_directories(test_rtk_lever_arm PRIVATE include)
  target_link_libraries(test_rtk_lever_arm Eigen3::Eigen)
  ament_add_pytest_test(test_finalize_lio_sam_map test/test_finalize_lio_sam_map.py)
  ament_add_pytest_test(
    test_indoor_fastlivo_fusion_validation
    test/test_indoor_fastlivo_fusion_validation.py
  )
  ament_add_pytest_test(
    test_localization_comparison_runner
    test/test_localization_comparison_runner.py
  )
  ament_add_pytest_test(test_offline_contracts test/test_offline_contracts.py)
  ament_add_pytest_test(
    test_semantic_navigation_graph
    test/test_semantic_navigation_graph.py
  )
  ament_add_pytest_test(
    test_ollama_instance_semantics
    test/test_ollama_instance_semantics.py
  )
  ament_add_pytest_test(
    test_map_semantic_navigation_graph
    test/test_map_semantic_navigation_graph.py
  )
  ament_add_pytest_test(
    test_refine_semantic_navigation_graph
    test/test_refine_semantic_navigation_graph.py
  )
  ament_add_pytest_test(
    test_semantic_model_context
    test/test_semantic_model_context.py
  )
  ament_add_pytest_test(
    test_semantic_graph_neo4j
    test/test_semantic_graph_neo4j.py
  )
  ament_add_pytest_test(
    test_semantic_graph_embedding_reuse
    test/test_semantic_graph_embedding_reuse.py
  )
  ament_add_pytest_test(
    test_semantic_route_planner
    test/test_semantic_route_planner.py
  )
  ament_add_pytest_test(
    test_semantic_task_bailian
    test/test_semantic_task_bailian.py
  )
  ament_add_pytest_test(
    test_semantic_planner_server
    test/test_semantic_planner_server.py
  )
  ament_add_pytest_test(
    test_semantic_smac_planner_validation
    test/test_semantic_smac_planner_validation.py
  )
  ament_add_pytest_test(test_rtk_mapping_pipeline test/test_rtk_mapping_pipeline.py)
  ament_add_pytest_test(test_trajectory_comparison test/test_trajectory_comparison.py)
endif()

ament_export_include_directories(include)
ament_package()
