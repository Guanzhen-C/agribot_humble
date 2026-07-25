# Ackermann physical vehicle

This directory contains the WHEELTEC C50C Ackermann chassis implementation:

- `src/` and `include/`: shared Ackermann kinematics plus CAN and USART3 codecs
- `config/chassis_can.yaml`: wheelbase, steering limits, IDs and safety timing
- `config/chassis_serial.yaml`: USART3 port, 115200 baud and safety timing
- `config/nav2_params_ackermann_fastlio_local.yaml`: mapless rolling costmaps
- `launch/`: NavSat, static-map FAST-LIO and local FAST-LIO entry points
- `test/`: captured-frame protocol and kinematics tests

Both chassis transports run at 20 Hz, require valid feedback before permitting
motion, send an all-zero command after a command timeout, and send a stop burst
during ROS shutdown. The serial transport is the default for the Ackermann
launch files; select `chassis_driver:=ackermann_can` to use SocketCAN instead.

Build and run the NavSat variant:

```bash
colcon build --packages-select agribot_hardware_bringup --symlink-install
source install/setup.bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_serial \
  serial_port:=/dev/wheeltec_controller
```

Use `ackermann_mppi_fastlio.launch.py` for FAST-LIO. Do not run the standalone
serial GUI while the ROS serial chassis node owns `/dev/wheeltec_controller`.
For short-range FAST-LIO navigation without a saved map, use:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio_local.launch.py \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_serial
```

The local mode uses `odom`-frame rolling costmaps and C16 `/scan` obstacles,
and caps MPPI at `0.30 m/s`. Goals must remain inside the rolling window.
The original CAN backend remains available:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio.launch.py \
  map:=/absolute/path/to/real_map.yaml \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_can \
  can_interface:=can0
```

USART3 uses 115200 baud. Commands are 11-byte `0x7b ... XOR 0x7d` packets and
telemetry is the 24-byte WHEELTEC packet also carried by the three CAN feedback
frames. The serial node publishes `/wheel/odometry`, `/scout_status`,
`/hardware/chassis_imu`, `/hardware/battery_voltage`,
`/hardware/chassis_e_stop`, and `/diagnostics`.

The telemetry contains chassis velocity, a chassis IMU, and battery voltage. It
does not contain a documented autonomous-mode, physical emergency-stop,
steering-position, motor-fault, or wheel-RPM field. Those states cannot be
inferred by this driver and require separate protocol support if available.
