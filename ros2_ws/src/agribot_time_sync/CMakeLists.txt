cmake_minimum_required(VERSION 3.8)
project(agribot_time_sync)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)

add_library(agribot_time_sync INTERFACE)
target_include_directories(agribot_time_sync INTERFACE
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(DIRECTORY include/ DESTINATION include)
install(TARGETS agribot_time_sync EXPORT export_agribot_time_sync)
ament_export_targets(export_agribot_time_sync HAS_LIBRARY_TARGET)
ament_export_include_directories(include)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_affine_clock_mapper test/test_affine_clock_mapper.cpp)
  target_link_libraries(test_affine_clock_mapper agribot_time_sync)
endif()

ament_package()
