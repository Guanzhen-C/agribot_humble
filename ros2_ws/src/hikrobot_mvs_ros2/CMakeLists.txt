cmake_minimum_required(VERSION 3.8)
project(hikrobot_mvs_ros2)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)

set(MVS_ROOT "/opt/MVS" CACHE PATH "Hikrobot MVS SDK installation directory")
if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64)$")
  set(MVS_LIBRARY_DIR "${MVS_ROOT}/lib/aarch64")
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64)$")
  set(MVS_LIBRARY_DIR "${MVS_ROOT}/lib/64")
else()
  message(FATAL_ERROR "Unsupported MVS architecture: ${CMAKE_SYSTEM_PROCESSOR}")
endif()

find_path(MVS_INCLUDE_DIR
  NAMES MvCameraControl.h
  PATHS "${MVS_ROOT}/include"
  NO_DEFAULT_PATH
)
find_library(MVS_CAMERA_CONTROL_LIBRARY
  NAMES MvCameraControl
  PATHS "${MVS_LIBRARY_DIR}"
  NO_DEFAULT_PATH
)

if(NOT MVS_INCLUDE_DIR OR NOT MVS_CAMERA_CONTROL_LIBRARY)
  message(FATAL_ERROR
    "Hikrobot MVS SDK not found under ${MVS_ROOT}. "
    "Run scripts/install_mvs_usb_sdk.sh with the vendor SDK archive first."
  )
endif()

add_executable(hikrobot_mvs_camera_node src/hikrobot_mvs_camera_node.cpp)
target_include_directories(hikrobot_mvs_camera_node PRIVATE "${MVS_INCLUDE_DIR}")
target_link_libraries(hikrobot_mvs_camera_node
  "${MVS_CAMERA_CONTROL_LIBRARY}"
  pthread
)
ament_target_dependencies(hikrobot_mvs_camera_node
  camera_info_manager
  rclcpp
  sensor_msgs
  std_msgs
)

install(TARGETS hikrobot_mvs_camera_node
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)
install(PROGRAMS scripts/install_mvs_usb_sdk.sh
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
