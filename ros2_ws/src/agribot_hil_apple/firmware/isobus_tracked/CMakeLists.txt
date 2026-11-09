cmake_minimum_required(VERSION 3.16)
set(HEAP_SIZE 0x28000)
set(STACK_SIZE 0x10000)
find_package(hpm-sdk REQUIRED HINTS $ENV{HPM_SDK_BASE})
project(hil_isobus_tracked LANGUAGES C CXX)
set(CMAKE_CXX_STANDARD 11)
# The SDK forces its compiler and skips CMake's cross-compiler feature probe.
set(CMAKE_CXX_COMPILE_FEATURES cxx_std_11 cxx_std_14 cxx_std_17)
set(CMAKE_CXX_STANDARD_COMPUTED_DEFAULT 17)
set(AGISOSTACK_ROOT "$ENV{HOME}/.cache/agribot/isobus/AgIsoStack-plus-plus" CACHE PATH "Pinned AgIsoStack++")
execute_process(COMMAND git -C "${AGISOSTACK_ROOT}" rev-parse HEAD
    OUTPUT_VARIABLE STACK_COMMIT OUTPUT_STRIP_TRAILING_WHITESPACE)
if(NOT STACK_COMMIT STREQUAL "02f3fe910de5fdc73e2014394c2e80900490b9cc")
    message(FATAL_ERROR "Wrong ISOBUS stack revision")
endif()
set(CAN_STACK_DISABLE_THREADS ON CACHE BOOL "Bare metal" FORCE)
set(DISABLE_CAN_STACK_LOGGER ON CACHE BOOL "No stack console logging" FORCE)
set(DISABLE_ISOBUS_DATA_DICTIONARY ON CACHE BOOL "Private PGN prototype" FORCE)
set(DISABLE_BUSLOAD_MONITORING ON CACHE BOOL "Bound memory use" FORCE)
set(BUILD_TESTING OFF CACHE BOOL "No upstream tests" FORCE)
set(BUILD_EXAMPLES OFF CACHE BOOL "No upstream examples" FORCE)
add_subdirectory("${AGISOSTACK_ROOT}" "${CMAKE_BINARY_DIR}/agisostack" EXCLUDE_FROM_ALL)
# This gateway does not implement agricultural guidance. Its unused optional
# source also relies on std::roundf, absent from this SDK's newlib C++ headers.
get_target_property(STACK_SOURCES Isobus SOURCES)
list(FILTER STACK_SOURCES EXCLUDE REGEX "isobus_guidance_interface\\.cpp$")
set_property(TARGET Isobus PROPERTY SOURCES "${STACK_SOURCES}")
# SDK flags include the target ABI and its board/system include directories.
target_link_libraries(Isobus PRIVATE $<BUILD_INTERFACE:hpm_sdk_lib_itf>)
target_link_libraries(Utility PRIVATE $<BUILD_INTERFACE:hpm_sdk_lib_itf>)
sdk_inc(../../isobus)
sdk_inc("${AGISOSTACK_ROOT}/isobus/include" "${AGISOSTACK_ROOT}/utility/include")
sdk_compile_definitions(-DBOARD_SHOW_CLOCK=0 -DBOARD_SHOW_BANNER=0
    -DCAN_STACK_DISABLE_THREADS -DDISABLE_CAN_STACK_LOGGER -DDISABLE_ISOBUS_DATA_DICTIONARY
    -DDISABLE_BUSLOAD_MONITORING)
sdk_app_src(main.cpp ../../isobus/endpoint.cpp ../../isobus/work_protocol.c)
target_link_libraries(${APP_ELF_NAME} Isobus Utility)
generate_ide_projects()
