cmake_minimum_required(VERSION 3.8)
project(agribot_differential_fastlivo_mppi)

find_package(ament_cmake REQUIRED)

set(_system_protobuf_lib_dir "/usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}")
if(EXISTS "${_system_protobuf_lib_dir}/libprotobuf.so")
  set(Protobuf_INCLUDE_DIR "/usr/include" CACHE PATH "" FORCE)
  set(Protobuf_LIBRARY "${_system_protobuf_lib_dir}/libprotobuf.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_LIBRARY_RELEASE "${_system_protobuf_lib_dir}/libprotobuf.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_LITE_LIBRARY "${_system_protobuf_lib_dir}/libprotobuf-lite.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_LITE_LIBRARY_RELEASE "${_system_protobuf_lib_dir}/libprotobuf-lite.so" CACHE FILEPATH "" FORCE)
  set(Protobuf_PROTOC_EXECUTABLE "/usr/bin/protoc" CACHE FILEPATH "" FORCE)
endif()

find_package(gazebo_dev REQUIRED)

set(_system_protobuf_include_shim "${CMAKE_CURRENT_BINARY_DIR}/system_protobuf_include")
file(MAKE_DIRECTORY "${_system_protobuf_include_shim}")
file(REMOVE "${_system_protobuf_include_shim}/google")
execute_process(
  COMMAND ${CMAKE_COMMAND} -E create_symlink /usr/include/google
          "${_system_protobuf_include_shim}/google"
)

add_library(agribot_differential_gazebo_render_workaround SHARED
  src/gazebo_render_path_workaround.c
)
target_link_libraries(agribot_differential_gazebo_render_workaround
  ${CMAKE_DL_LIBS}
)

add_library(agribot_differential_gazebo_scene_sync SHARED
  src/gazebo_scene_sync_plugin.cpp
)
target_include_directories(agribot_differential_gazebo_scene_sync PRIVATE
  "${_system_protobuf_include_shim}"
  ${gazebo_dev_INCLUDE_DIRS}
)
target_link_libraries(agribot_differential_gazebo_scene_sync
  ${gazebo_dev_LIBRARIES}
)

install(TARGETS
  agribot_differential_gazebo_render_workaround
  agribot_differential_gazebo_scene_sync
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/differential_sim_cmd_bridge.py
  scripts/differential_sim_sensor_bridge.py
  scripts/differential_sim_validator.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})

ament_package()
