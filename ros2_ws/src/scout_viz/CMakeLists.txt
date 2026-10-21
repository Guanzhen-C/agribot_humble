cmake_minimum_required(VERSION 3.8)
project(scout_viz)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(action_msgs REQUIRED)

add_library(scout_image_display SHARED src/image_display.cpp)
target_compile_features(scout_image_display PRIVATE cxx_std_17)
ament_target_dependencies(scout_image_display pluginlib rviz_default_plugins)
pluginlib_export_plugin_description_file(rviz_common plugins.xml)
install(TARGETS scout_image_display LIBRARY DESTINATION lib)

add_library(scout_navigation_path_display SHARED src/navigation_path_display.cpp)
target_compile_features(scout_navigation_path_display PRIVATE cxx_std_17)
ament_target_dependencies(scout_navigation_path_display pluginlib rviz_default_plugins action_msgs)
pluginlib_export_plugin_description_file(rviz_common navigation_path_plugins.xml)
install(TARGETS scout_navigation_path_display LIBRARY DESTINATION lib)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_navigation_task_state test/test_navigation_task_state.cpp)
  target_compile_features(test_navigation_task_state PRIVATE cxx_std_17)
  target_include_directories(test_navigation_task_state PRIVATE src)
  ament_target_dependencies(test_navigation_task_state action_msgs)
endif()

install(DIRECTORY launch rviz
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/run_rviz2_clean_env.sh
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
