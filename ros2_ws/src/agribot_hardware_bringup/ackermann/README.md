# Ackermann physical vehicle

This directory contains the WHEELTEC C50C Ackermann chassis implementation:

- `src/` and `include/`: `0x181` command codec and `0x101/0x102/0x103` telemetry adapter
- `config/chassis_can.yaml`: wheelbase, steering limits, IDs and safety timing
- `launch/`: NavSat and FAST-LIO physical-vehicle entry points
- `test/`: captured-frame protocol and kinematics tests

The RDK X5 configures `can0` for standard CAN at 1 Mbit/s. The driver transmits
at 20 Hz, requires valid feedback before permitting motion, sends an all-zero
command after a command timeout, and sends a stop burst during ROS shutdown.

Build and run the NavSat variant:

```bash
colcon build --packages-select agribot_hardware_bringup --symlink-install
source install/setup.bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml \
  enable_can_output:=true can_interface:=can0
```

Use `ackermann_mppi_fastlio.launch.py` for FAST-LIO. Do not run the standalone
`rdk_car_gui` controller while the ROS chassis node owns `0x181`.

The three feedback frames contain chassis velocity, a chassis IMU, and battery
voltage. They do not contain a documented autonomous-mode, emergency-stop,
steering-position, motor-fault, or wheel-RPM field. Those safety signals cannot
be inferred by this driver and require separate protocol support if available.
