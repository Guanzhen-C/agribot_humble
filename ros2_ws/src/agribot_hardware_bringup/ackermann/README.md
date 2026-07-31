# Ackermann physical vehicle

This directory contains the WHEELTEC C50C Ackermann chassis implementation:

- `src/` and `include/`: shared Ackermann kinematics plus CAN and USART3 codecs
- `config/chassis_can.yaml`: wheelbase, steering limits, IDs and safety timing
- `config/chassis_serial.yaml`: USART3 port, 115200 baud and safety timing
- `config/nav2_params_ackermann_fastlio_local.yaml`: mapless rolling costmaps
- `config/nav2_params_ackermann_fastlio_mapped.yaml`: mapped navigation limits
- `launch/`: NavSat, local FAST-LIO, 3D mapping and saved-map navigation
- `test/`: captured-frame protocol and kinematics tests

Both chassis transports run at 20 Hz, require valid feedback before permitting
motion, send an all-zero command after a command timeout, and send a stop burst
during ROS shutdown. The Ackermann CAN launch files default to the ZQWL-CANFD
USB CDC bridge on channel 0 at 1 Mbit/s. Native SocketCAN and the chassis
USART3 connection remain available as explicit fallbacks.
Every ZQWL open performs one STOP, waits 100 ms, clears stale input and then
sends START. This resets a stale channel once at startup; motion remains
blocked until fresh `0x101`, `0x102` and `0x103` feedback has been received.
The serial transport also limits commanded steering changes to `0.60 rad/s`;
timeouts and stop commands bypass this limiter and stop immediately.

Build and run the NavSat variant:

```bash
colcon build --packages-select agribot_hardware_bringup --symlink-install
source install/setup.bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_can \
  can_transport:=zqwl_cdc \
  zqwl_port:=/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

For short-range FAST-LIO navigation without a saved map, use:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio_local.launch.py \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_can \
  can_transport:=zqwl_cdc \
  zqwl_port:=/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

The local mode uses `odom`-frame rolling costmaps and the complete C16
`/lidar/points` cloud through STVL, and caps MPPI at `0.30 m/s`. Goals must
remain inside the rolling window.
The serial fallback remains available. Do not run the standalone serial GUI
while the ROS serial chassis node owns the chassis USB serial port:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio_local.launch.py \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_serial \
  serial_port:=/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C2C079857-if00
```

For 3D PCD map construction with FAST-LIO odometry:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_mppi_fastlio_3d_mapping.launch.py \
  map_base:=/home/sunrise/agribot_maps/test_site/map \
  enable_chassis_output:=false
```

The mapping entry point accumulates `/cloud_registered` into a voxelized
three-dimensional map. Mapping defaults to no chassis output so one manual
controller can drive the coverage route. By default, it excludes registered
points in a `1.2 m` wide strip extending from the rear bumper to `4 m` behind
`base_link`, allowing one operator to follow directly behind the vehicle
without being accumulated into the map. Save it before shutdown:

```bash
ros2 service call /pcd_map_builder/save_map std_srvs/srv/Trigger "{}"
```

The save operation writes `.pcd`, `.pgm`, and `.yaml` files in one coordinate
system. Use the saved YAML projection for long-range planning:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_mppi_fastlio_mapped.launch.py \
  map_base:=/home/sunrise/agribot_maps/test_site/map \
  initial_pose:="[0.0, 0.0, 0.0]" \
  enable_chassis_output:=true \
  chassis_driver:=ackermann_can \
  can_transport:=zqwl_cdc \
  zqwl_port:=/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

Set an approximate initial rear-axle map pose on the command line or with
RViz's `2D Pose Estimate` tool before sending a goal. The map supplies planning
only; FAST-LIO remains the pose source and is not continuously corrected by
map matching.

USART3 uses 115200 baud. Commands are 11-byte `0x7b ... XOR 0x7d` packets and
telemetry is the 24-byte WHEELTEC packet also carried by the three CAN feedback
frames. The serial node publishes `/wheel/odometry`, `/scout_status`,
`/hardware/chassis_imu`, `/hardware/battery_voltage`,
`/hardware/chassis_e_stop`, and `/diagnostics`.

The telemetry contains chassis velocity, a chassis IMU, and battery voltage. It
does not contain a documented autonomous-mode, physical emergency-stop,
steering-position, motor-fault, or wheel-RPM field. Those states cannot be
inferred by this driver and require separate protocol support if available.

For a native CAN adapter, override the CAN transport:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio_local.launch.py \
  can_transport:=socketcan \
  can_interface:=can0
```
