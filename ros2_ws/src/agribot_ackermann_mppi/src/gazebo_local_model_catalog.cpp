#include <gazebo/common/FuelModelDatabase.hh>

// Loaded only into our offline gzclient. Gazebo 11 detaches the Fuel catalog
// request threads, which can outlive libcurl/OpenSSL during process shutdown.
// All simulation assets are local; skip catalog discovery, not model loading.
std::vector<ignition::fuel_tools::ServerConfig>
gazebo::common::FuelModelDatabase::Servers() const
{
  return {};
}
