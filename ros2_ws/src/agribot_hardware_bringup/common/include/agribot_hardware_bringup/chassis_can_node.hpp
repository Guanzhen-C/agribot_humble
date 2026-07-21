#ifndef AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_NODE_HPP_
#define AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_NODE_HPP_

#include "agribot_hardware_bringup/chassis_adapter.hpp"

namespace agribot_hardware_bringup
{

int runChassisCanNode(
  int argc,
  char ** argv,
  const char * node_name,
  ChassisAdapterFactory adapter_factory);

}  // namespace agribot_hardware_bringup

#endif  // AGRIBOT_HARDWARE_BRINGUP__CHASSIS_CAN_NODE_HPP_
