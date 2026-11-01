cmake_minimum_required(VERSION 3.8)
project(agribot_differential_fastlivo_mppi)

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
ament_python_install_package(${PROJECT_NAME})

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
find_package(gazebo_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_library(GAZEBO_SIMPLE_TRACK_PLUGIN NAMES SimpleTrackedVehiclePlugin
  PATHS ${gazebo_dev_PLUGIN_PATH} /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/gazebo-11/plugins REQUIRED)
find_library(GAZEBO_TRACK_PLUGIN NAMES TrackedVehiclePlugin
  PATHS ${gazebo_dev_PLUGIN_PATH} /usr/lib/${CMAKE_LIBRARY_ARCHITECTURE}/gazebo-11/plugins REQUIRED)
find_path(GAZEBO_PLUGIN_HEADER_ROOT plugins/TrackedVehiclePlugin.hh
  HINTS ${gazebo_dev_INCLUDE_DIRS} PATH_SUFFIXES gazebo REQUIRED)
find_library(GAZEBO_ODE_LIBRARY NAMES gazebo_ode REQUIRED)

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

add_library(agribot_scanned_track_drive SHARED src/scanned_track_drive.cpp)
target_compile_features(agribot_scanned_track_drive PRIVATE cxx_std_17)
target_include_directories(agribot_scanned_track_drive PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include"
  "${_system_protobuf_include_shim}" ${gazebo_dev_INCLUDE_DIRS} ${GAZEBO_PLUGIN_HEADER_ROOT})
ament_target_dependencies(agribot_scanned_track_drive gazebo_ros geometry_msgs)
target_link_libraries(agribot_scanned_track_drive
  ${gazebo_dev_LIBRARIES} ${GAZEBO_SIMPLE_TRACK_PLUGIN} ${GAZEBO_TRACK_PLUGIN} ${GAZEBO_ODE_LIBRARY})

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
  agribot_scanned_track_drive
  LIBRARY DESTINATION lib
)

install(PROGRAMS
  scripts/differential_sim_cmd_bridge.py
  scripts/differential_sim_sensor_bridge.py
  scripts/differential_sim_validator.py
  scripts/test_saved_path_gazebo.py
  scripts/prepare_scanned_route_validation.py
  scripts/add_scanned_ground_support.py
  scripts/prepare_scanned_contact_proxy.py
  scripts/run_scanned_saved_route.py
  scripts/display_scanned_route_result.py
  scripts/fuxi_farm_profile.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  urdf
  scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES README.md DESTINATION share/${PROJECT_NAME})
install(FILES
  ${PROJECT_NAME}/saved_route_runtime/LICENSE
  ${PROJECT_NAME}/saved_route_runtime/snapshot.json
  DESTINATION share/${PROJECT_NAME}/saved_route_runtime
)

if(BUILD_TESTING)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_scanned_route_preparation
    test/test_scanned_route_preparation.py TIMEOUT 60)
  ament_add_pytest_test(test_scanned_route_display
    test/test_scanned_route_display.py TIMEOUT 60)
  ament_add_pytest_test(test_fuxi_farm_profile
    test/test_fuxi_farm_profile.py TIMEOUT 60)
endif()

ament_package()
