# Copyright(c) 2020 eaibot limited.
cmake_minimum_required(VERSION 3.5)
project(ydlidar_ros2_driver C CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

####################find package#####################################
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

############## YDLIDAR SDK START#####################################
set(YDLIDAR_SDK_LOCAL_PATH "${CMAKE_CURRENT_SOURCE_DIR}/../YDLidar-SDK")
find_package(ydlidar_sdk QUIET)
if(NOT ydlidar_sdk_FOUND)
  if(EXISTS "${YDLIDAR_SDK_LOCAL_PATH}/CMakeLists.txt")
    set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
    set(BUILD_TEST OFF CACHE BOOL "" FORCE)
    set(BUILD_CSHARP OFF CACHE BOOL "" FORCE)
    add_subdirectory(
      "${YDLIDAR_SDK_LOCAL_PATH}"
      "${CMAKE_CURRENT_BINARY_DIR}/ydlidar_sdk"
    )
    set(YDLIDAR_SDK_INCLUDE_DIRS
      "${YDLIDAR_SDK_LOCAL_PATH}"
      "${YDLIDAR_SDK_LOCAL_PATH}/src"
      "${YDLIDAR_SDK_LOCAL_PATH}/core"
      "${CMAKE_CURRENT_BINARY_DIR}/ydlidar_sdk"
    )
    set(YDLIDAR_SDK_LIBRARIES ydlidar_sdk)
  else()
    message(FATAL_ERROR "Could not find ydlidar_sdk and no local YDLidar-SDK checkout is available.")
  endif()
endif()
############## YDLIDAR SDK END#####################################

# Link library directories
if(YDLIDAR_SDK_LIBRARY_DIRS)
  link_directories(${YDLIDAR_SDK_LIBRARY_DIRS})
endif()

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_node src/${PROJECT_NAME}_node.cpp)
target_include_directories(${PROJECT_NAME}_node SYSTEM PRIVATE
  ${YDLIDAR_SDK_INCLUDE_DIRS}
)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_node
  rclcpp
  sensor_msgs
  std_srvs
)

target_link_libraries(${PROJECT_NAME}_node
  ${YDLIDAR_SDK_LIBRARIES}
)

#---------------------------------------------------------------------------------------
# generate excutable and add libraries
#---------------------------------------------------------------------------------------
add_executable(${PROJECT_NAME}_client src/${PROJECT_NAME}_client.cpp)
#---------------------------------------------------------------------------------------
# link libraries
#--------------------------------------------------------------------------------------
ament_target_dependencies(${PROJECT_NAME}_client
  rclcpp
  sensor_msgs
)

#---------------------------------------------------------------------------------------
# Install
#---------------------------------------------------------------------------------------
install(TARGETS
  ${PROJECT_NAME}_node
  ${PROJECT_NAME}_client
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch params startup config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
