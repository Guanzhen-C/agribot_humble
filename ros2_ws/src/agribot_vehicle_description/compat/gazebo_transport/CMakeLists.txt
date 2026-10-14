find_package(PkgConfig REQUIRED)
pkg_check_modules(TRANSPORT_GAZEBO QUIET gazebo)
if(NOT TRANSPORT_GAZEBO_VERSION STREQUAL "11.10.2")
  message(STATUS "Gazebo transport compatibility fix is only for Gazebo 11.10.2")
  return()
endif()
find_package(TBB REQUIRED)
if(TBB_VERSION VERSION_LESS "2021")
  return()
endif()
find_package(gazebo REQUIRED)
find_package(Boost REQUIRED COMPONENTS thread system filesystem)
include(FetchContent)
FetchContent_Declare(agribot_gazebo_upstream
  URL https://codeload.github.com/gazebosim/gazebo-classic/tar.gz/refs/tags/gazebo11_11.10.2
  URL_HASH SHA256=51976f29e375463e3bbf1343b7532b038728f13389e94e78b2fa8271911b2dcb)
FetchContent_GetProperties(agribot_gazebo_upstream)
if(NOT agribot_gazebo_upstream_POPULATED)
  FetchContent_Populate(agribot_gazebo_upstream)
endif()

# Preserve the installed Connection layout and all upstream transport methods.
# Drain on the IO arena that queues reads, before its worker is destroyed.
file(READ "${agribot_gazebo_upstream_SOURCE_DIR}/gazebo/transport/Connection.cc" connection_source)
set(before "  if (iomanager)\n  {\n    iomanager->DecCount();")
set(after "  if (iomanager)\n  {\n#if TBB_VERSION_MAJOR >= 2021\n    std::promise<void> drained;\n    auto completed = drained.get_future();\n    iomanager->GetIO().dispatch([this, &drained]()\n    {\n      this->taskGroup.Wait();\n      drained.set_value();\n    });\n    completed.get();\n#endif\n    iomanager->DecCount();")
string(FIND "${connection_source}" "${before}" patch_offset)
if(patch_offset LESS 0)
  message(FATAL_ERROR "Pinned Gazebo Connection destructor does not match the shutdown patch")
endif()
string(REPLACE "${before}" "${after}" connection_source "${connection_source}")
string(REPLACE "#include <stdio.h>" "#include <stdio.h>\n#include <future>"
  connection_source "${connection_source}")
string(APPEND connection_source
  "\n// AGRIBOT modification: drain pending read tasks on their IO arena before IOManager teardown.\n")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/Connection.cc" "${connection_source}")

function(replace_checked variable before after)
  string(FIND "${${variable}}" "${before}" offset)
  if(offset LESS 0)
    message(FATAL_ERROR "Pinned Gazebo IOManager does not match the lifetime patch")
  endif()
  string(REPLACE "${before}" "${after}" updated "${${variable}}")
  set(${variable} "${updated}" PARENT_SCOPE)
endfunction()
file(READ "${agribot_gazebo_upstream_SOURCE_DIR}/gazebo/transport/IOManager.cc" io_source)
replace_checked(io_source "#include <atomic>" "#include <atomic>\n#include <memory>")
replace_checked(io_source "boost::asio::io_service *io_service = nullptr;"
  "std::shared_ptr<boost::asio::io_service> io_service;")
replace_checked(io_source "this->dataPtr->io_service = new boost::asio::io_service;"
  "this->dataPtr->io_service = std::make_shared<boost::asio::io_service>();")
replace_checked(io_source
  "this->dataPtr->thread = new boost::thread(boost::bind(\n      &boost::asio::io_service::run, this->dataPtr->io_service));"
  "this->dataPtr->thread = new boost::thread(\n      [service = this->dataPtr->io_service]() { service->run(); });")
replace_checked(io_source
  "delete this->dataPtr->io_service;\n  this->dataPtr->io_service = nullptr;"
  "this->dataPtr->io_service.reset();")
replace_checked(io_source "this->dataPtr->thread->join();"
  "if (this->dataPtr->thread->get_id() == boost::this_thread::get_id())\n      this->dataPtr->thread->detach();\n    else\n      this->dataPtr->thread->join();")
# When the last connection dies in an IO callback, joining this same thread
# would throw. Its captured shared_ptr keeps io_service alive until run exits.
string(APPEND io_source
  "\n// AGRIBOT modification: retain IO service ownership until run returns and avoid joining the current thread.\n")
file(WRITE "${CMAKE_CURRENT_BINARY_DIR}/IOManager.cc" "${io_source}")

set(transport_sources CallbackHelper ConnectionManager Node
  Publication PublicationTransport Publisher Subscriber SubscriptionTransport
  TopicManager TransportIface)
list(TRANSFORM transport_sources PREPEND "${agribot_gazebo_upstream_SOURCE_DIR}/gazebo/transport/")
list(TRANSFORM transport_sources APPEND ".cc")
add_library(agribot_gazebo_transport_compat SHARED
  "${CMAKE_CURRENT_BINARY_DIR}/Connection.cc"
  "${CMAKE_CURRENT_BINARY_DIR}/IOManager.cc" ${transport_sources})
target_compile_features(agribot_gazebo_transport_compat PRIVATE cxx_std_17)
if(NOT CMAKE_BUILD_TYPE)
  target_compile_options(agribot_gazebo_transport_compat PRIVATE -O2)
endif()
target_include_directories(agribot_gazebo_transport_compat BEFORE PRIVATE
  "${CMAKE_CURRENT_SOURCE_DIR}/include" ${GAZEBO_INCLUDE_DIRS})
target_compile_definitions(agribot_gazebo_transport_compat PRIVATE
  BUILDING_DLL_GZ_TRANSPORT GAZEBO_DEFAULT_MASTER_HOST="localhost"
  GAZEBO_DEFAULT_MASTER_PORT=11345)
target_link_libraries(agribot_gazebo_transport_compat PRIVATE gazebo_common gazebo_msgs
  TBB::tbb Boost::thread Boost::system Boost::filesystem)
set_target_properties(agribot_gazebo_transport_compat PROPERTIES
  OUTPUT_NAME gazebo_transport VERSION 11.10.2 SOVERSION 11
  LIBRARY_OUTPUT_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}/lib")
install(TARGETS agribot_gazebo_transport_compat
  LIBRARY DESTINATION lib/agribot_gazebo_transport_compat)
# ament's symlink installer otherwise installs only the fully versioned file.
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/lib/libgazebo_transport.so.11"
  DESTINATION lib/agribot_gazebo_transport_compat)
install(FILES
  "${agribot_gazebo_upstream_SOURCE_DIR}/LICENSE"
  "${agribot_gazebo_upstream_SOURCE_DIR}/COPYING"
  DESTINATION share/${PROJECT_NAME}/licenses/gazebo_transport)
