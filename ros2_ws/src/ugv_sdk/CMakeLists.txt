cmake_minimum_required(VERSION 3.8)
project(ugv_sdk VERSION 0.8.0)

find_package(ament_cmake REQUIRED)

find_program(CCACHE_PROGRAM ccache)
if(CCACHE_PROGRAM)
    message(STATUS "Found ccache")
    set_property(GLOBAL PROPERTY RULE_LAUNCH_COMPILE "${CCACHE_PROGRAM}")
endif()

## Project Options
option(BUILD_TESTING "Build tests" OFF)
option(STATIC_CHECK "Run static check" OFF)
# set(CMAKE_BUILD_TYPE Release)
# set(CMAKE_BUILD_TYPE Debug)

set(BUILD_WITHOUT_ROS ON)

## Generate symbols for IDE indexer
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(STATIC_CHECK)
    find_program(CPPCHECK cppcheck)
    if(CPPCHECK)
        message(STATUS "Found cppcheck")
        set(CMAKE_CXX_CPPCHECK cppcheck;--std=c++11;--enable=all)
    endif() 
endif()

## Additional cmake module path
set(USER_CMAKE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
list(APPEND CMAKE_MODULE_PATH "${USER_CMAKE_PATH}/modules")
list(APPEND CMAKE_PREFIX_PATH "/usr/lib/${CMAKE_SYSTEM_PROCESSOR}-linux-gnu/cmake")

## Set compiler to use c++ 11 features
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Chosse build type
set(default_build_type "Release")
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to '${default_build_type}' as none was specified.")
  set(CMAKE_BUILD_TYPE "${default_build_type}" CACHE
      STRING "Choose the type of build." FORCE)
  # Set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

if(BUILD_WITHOUT_ROS)
    # check whether to build tests
    if(PROJECT_NAME STREQUAL CMAKE_PROJECT_NAME AND BUILD_TESTING)
        enable_testing()
        include(GoogleTest)
        set(BUILD_TESTS ON)
        message(STATUS "Tests will be built")
    else()
        set(BUILD_TESTS OFF)
        message(STATUS "Tests will not be built")
    endif()

    ## Use GNUInstallDirs to install libraries into correct locations on all platforms.
    include(GNUInstallDirs)

    ## Put all binary files into /bin and libraries into /lib
    set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR})
    set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_BINARY_DIR}/${CMAKE_INSTALL_BINDIR})
endif()

## Build libraries
find_package(Threads REQUIRED)

# add sdk main library
add_library(${PROJECT_NAME}
    ########################
    ## async_port from weston robot
    src/async_port/async_serial.cpp
    src/async_port/async_can.cpp
    ########################
    ## utilities
    src/utilities/protocol_detector.cpp
    ########################
    ## public interface to access robot
    src/mobile_robot/scout_robot.cpp
    src/mobile_robot/hunter_robot.cpp
    src/mobile_robot/bunker_robot.cpp
    src/mobile_robot/ranger_robot.cpp
    src/mobile_robot/titan_robot.cpp
    src/mobile_robot/tracer_robot.cpp
    ########################
    ## protocol v2 support
    src/protocol_v2/agilex_msg_parser_v2.c
    src/protocol_v2/protocol_v2_parser.cpp
    ########################
    ## legacy protocol v1 support (transparent to user)
    src/protocol_v1/agilex_msg_parser_v1.c
)
target_link_libraries(${PROJECT_NAME} PUBLIC Threads::Threads)
target_compile_definitions(${PROJECT_NAME} PUBLIC ASIO_ENABLE_OLD_SERVICES)
target_include_directories(${PROJECT_NAME} PUBLIC
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)
    
set(BUILD_TESTS OFF)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(DIRECTORY include/ugv_sdk
  DESTINATION include
)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_package()
