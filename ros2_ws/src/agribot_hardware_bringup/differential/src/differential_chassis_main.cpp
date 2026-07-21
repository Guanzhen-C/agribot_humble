#include "agribot_hardware_bringup/chassis_can_node.hpp"

int main(int argc, char ** argv)
{
  return agribot_hardware_bringup::runChassisCanNode(
    argc, argv, "differential_chassis_can",
    agribot_hardware_bringup::makeDifferentialChassisAdapter);
}
