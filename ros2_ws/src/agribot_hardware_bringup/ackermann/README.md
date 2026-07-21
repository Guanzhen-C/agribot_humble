# Ackermann physical vehicle

This directory contains only the Ackermann chassis implementation:

- `src/` and `include/`: `0x515/0x535/0x536/0x537` reference CAN protocol and adapter
- `config/chassis_can.yaml`: wheelbase, steering limits, IDs and safety timing
- `launch/`: NavSat and FAST-LIO physical-vehicle entry points
- `test/`: Ackermann protocol and kinematics tests

Build and run the NavSat variant after the controller firmware has been checked
against the reference protocol:

```bash
colcon build --packages-select agribot_hardware_bringup --symlink-install
source install/setup.bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  enable_can_output:=true can_interface:=can0 \
  allow_unverified_ackermann_protocol:=true
```

Use `ackermann_mppi_fastlio.launch.py` for FAST-LIO. Do not enable physical CAN
output until the controller supplier confirms all four frames.
