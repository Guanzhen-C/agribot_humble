cmake_minimum_required(VERSION 3.8)
project(vikit_common)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

add_compile_options(
  -Wall
  -Wno-unknown-pragmas
  -Wno-unused-but-set-variable
  -Wno-unused-variable
)

if(CMAKE_SYSTEM_PROCESSOR MATCHES "^(x86_64|amd64|AMD64|i.86)$")
  add_compile_options(-mmmx -msse -msse2 -msse3 -mssse3)
elseif(CMAKE_SYSTEM_PROCESSOR MATCHES "^(aarch64|arm64|ARM64)$")
  add_compile_options(-march=armv8-a)
endif()

find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(Sophus REQUIRED)
find_package(fmt REQUIRED)

set(SOURCES
  src/atan_camera.cpp
  src/equidistant_camera.cpp
  src/homography.cpp
  src/img_align.cpp
  src/math_utils.cpp
  src/omni_camera.cpp
  src/performance_monitor.cpp
  src/pinhole_camera.cpp
  src/polynomial_camera.cpp
  src/robust_cost.cpp
  src/user_input_thread.cpp
  src/vision.cpp
)

add_library(${PROJECT_NAME} SHARED ${SOURCES})
target_include_directories(${PROJECT_NAME}
  PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
)
target_link_libraries(${PROJECT_NAME}
  PUBLIC
    Eigen3::Eigen
    ${OpenCV_LIBS}
    Sophus::Sophus
    fmt::fmt
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 OpenCV Sophus fmt)
ament_package()
