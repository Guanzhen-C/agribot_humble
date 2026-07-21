# Differential physical vehicle

This directory contains only the differential chassis implementation:

- `src/` and `include/`: `0x514/0x532/0x533/0x534` CAN protocol and adapter
- `config/chassis_can.yaml`: physical chassis dimensions, limits and safety timing
- `config/nav2_dwb_*.yaml`: DWB navigation parameters
- `launch/`: NavSat and FAST-LIO physical-vehicle entry points
- `test/`: differential protocol and kinematics tests

Build and run the NavSat variant:

```bash
colcon build --packages-select agribot_hardware_bringup --symlink-install
source install/setup.bash
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py \
  enable_can_output:=true can_interface:=can0
```

Use `differential_dwb_fastlio.launch.py` for FAST-LIO.
