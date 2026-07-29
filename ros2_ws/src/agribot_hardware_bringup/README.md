# Agribot hardware bringup

This package is the physical-vehicle entry point for sensors, localization,
Nav2, command safety, and SocketCAN chassis control. It supports these tested
navigation selections:

| Vehicle | Controller | Localization | Launch file |
| --- | --- | --- | --- |
| Differential | DWB | NavSat/KF-GINS | `differential_dwb_navsat.launch.py` |
| Differential | DWB | FAST-LIO | `differential_dwb_fastlio.launch.py` |
| Ackermann | MPPI | NavSat/KF-GINS | `ackermann_mppi_navsat.launch.py` |
| Ackermann | MPPI | FAST-LIO | `ackermann_mppi_fastlio.launch.py` |
| Ackermann | MPPI | FAST-LIO local rolling map | `ackermann_mppi_fastlio_local.launch.py` |

Vehicle-specific physical code is kept in separate source trees:

```text
can/           SocketCAN transport, ROS topics, diagnostics and frame utilities
differential/  differential protocol, adapter, executable, config, launch and tests
ackermann/     Ackermann protocol, MPPI config, behavior trees, launch and tests
localization/  NavSat/KF-GINS node and localization bridge scripts
```

All project-owned runtime code and resources used by these four entry points
are contained in this package. The remaining package dependencies are ROS 2
system components or third-party device/algorithm packages: Nav2, RViz,
FAST-LIO, `hipnuc_imu`, `lslidar_driver`, and their message packages.
The CAN status interface also uses the third-party `scout_msgs` package.

No simulation map is bundled with the physical-vehicle package. Pass the
absolute path to a map recorded on the target vehicle with `map:=/path/map.yaml`
when launching a static-map entry point. The FAST-LIO local entry point does
not require a saved map.

The installed executables are `differential_chassis_can_node` and
`ackermann_chassis_can_node`; there is no mixed vehicle executable.

The dedicated differential launch files default to CAN output disabled. The
Nav2 command path is:

```text
/nav2/cmd_vel -> chassis driver
```

## Differential CAN protocol

The differential driver implements the chassis portion of the supplied
`三合一协议.xlsx`, with behavior from the proven
`noah_chassis_mutil_function_car.cpp` retained where the workbook is silent.
The workbook is authoritative for signal positions, Intel byte order, rolling
counter, and XOR checksum.

| Direction | CAN ID | Content |
| --- | --- | --- |
| TX | `0x514` | Left/right motor percentage and headlight command |
| RX | `0x532` | Mode, emergency stop, motion state, battery, communication faults |
| RX | `0x533` | Left motor faults, RPM, voltage, current, temperature |
| RX | `0x534` | Right motor faults, RPM, voltage, current, temperature |

All frames are standard 11-bit, 8-byte CAN frames. Byte 6 low nibble is the
rolling counter and byte 7 is XOR of bytes 0 through 6. Invalid checksums and
unchanged replay counters are rejected. Counter jumps are accepted as dropped
frames but recorded in `/diagnostics`.

The old working C++ writes `0x03` to command byte 0 while braking, although the
workbook does not define that byte. This compatibility behavior is enabled by
`legacy_brake_byte: true`. The old C++ also used big-endian battery decoding;
the migrated driver corrects that to the workbook's Intel order.

The workbook also describes implement, remote-control, and BMS frames. Those
are outside this chassis adapter. In particular, the old C++ sends an
implement command on `0x582`, while the workbook defines it as `0x580`; that
conflicting implement command is intentionally not transmitted here.

The driver command topic is configurable. The unified navigation launch routes
`geometry_msgs/msg/Twist` from `/nav2/cmd_vel` directly to the selected
driver. The driver publishes:

- `/wheel/odometry`: odometry integrated from left/right motor RPM
- `/scout_status`: common chassis feedback
- `/hardware/chassis_e_stop`: decoded controller emergency-stop state
- `/diagnostics`: freshness, fault, checksum, counter, replay, and I/O status

Important dimensions and drivetrain values are in
`differential/config/chassis_can.yaml`:
`track_width_m`, `wheel_diameter_m`, `reduction_ratio`, and `max_motor_rpm`.
Measure and verify them before physical motion.

## Differential DWB navigation

NavSat/KF-GINS, DWB, sensors, collision monitor, and RViz:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml
```

FAST-LIO, DWB, sensors, collision monitor, and RViz:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_fastlio.launch.py \
  map:=/absolute/path/to/real_map.yaml
```

Both use a maximum linear speed of `0.8 m/s` and maximum angular speed of
`1.4 rad/s`. FAST-LIO consumes `/lidar/points` and `/imu/data` for
localization. In both modes, obstacle avoidance consumes `/scan`, which is the
horizontal projection published by the C16 driver on the real vehicle.

To inspect the complete navigation stack without opening a chassis transport,
leave the default `enable_chassis_output:=false`. The unified real-vehicle
launch accepts `none`, `differential_can`, `ackermann_can`, and
`ackermann_serial` chassis backends. The legacy `enable_can_output` argument
remains as a compatibility alias.

After the controller bitrate, vehicle dimensions, wheel directions, emergency
stop, and lifted-wheel test have been confirmed, enable the supplied
differential controller:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml \
  enable_can_output:=true chassis_driver:=differential_can can_interface:=can0
```

Use `differential_dwb_fastlio.launch.py` for the corresponding FAST-LIO run.

## SocketCAN setup

The supplied workbook does not state the bus bitrate. Obtain it from the
controller configuration before running:

```bash
ros2 run agribot_hardware_bringup configure_can.sh can0 BITRATE
```

The setup script requests `sudo`; no password is stored in this package.
For protocol-only testing, create a virtual CAN interface and run the node:

```bash
sudo modprobe vcan
sudo ip link add dev vcan0 type vcan
sudo ip link set vcan0 up
ros2 run agribot_hardware_bringup differential_chassis_can_node --ros-args \
  --params-file $(ros2 pkg prefix agribot_hardware_bringup)/share/agribot_hardware_bringup/differential/config/chassis_can.yaml \
  -p can_interface:=vcan0
```

The node sends at 10 Hz, matching the proven implementation and the
workbook's recommended 100-300 ms interval. Motion is independently blocked
when the command is older than 0.25 s, required feedback is older than 1.2 s,
the controller is not in autonomous mode, emergency stop is active, or a
reported chassis/motor fault is present. Shutdown sends three brake frames.

## Ackermann C50C protocol

The Ackermann backend supports both the WHEELTEC C50C SocketCAN protocol and its
USART3 transport. SocketCAN was verified on the RDK X5 at 1 Mbit/s:

| Direction | CAN ID | Content |
| --- | --- | --- |
| TX | `0x181` | Longitudinal speed, lateral speed, front steering angle, reserved |
| RX | `0x101` | Bytes 0-7 of the 24-byte chassis telemetry packet |
| RX | `0x102` | Bytes 8-15 of the 24-byte chassis telemetry packet |
| RX | `0x103` | Bytes 16-23 of the 24-byte chassis telemetry packet |

`0x181` contains four signed big-endian 16-bit fields. Bytes 0-1 are forward
speed in `0.001 m/s`, bytes 2-3 are lateral speed and remain zero for Ackermann,
bytes 4-5 are front steering angle in `0.001 rad`, and bytes 6-7 are reserved
zeros. Positive speed is forward and positive steering turns left. A stop is an
all-zero frame.

The RX frames are one logical packet, not three independent status messages.
The assembled packet starts with `0x7b`, ends with `0x7d`, and byte 22 is the
XOR of bytes 0 through 21. It reports X/Y velocity and Z yaw rate at `0.001`
units, raw chassis IMU values, and battery voltage at `0.001 V`. The driver
publishes wheel odometry only after the complete packet passes its BCC check.

The serial backend opens
`/dev/serial/by-id/usb-1a86_USB_Single_Serial_5C2C079857-if00` exclusively at
115200 baud.
Its 11-byte command starts with `0x7b`, contains forward speed, zero lateral
speed and front steering angle as signed big-endian values at `0.001` units,
then an XOR byte and `0x7d`. It receives the same 24-byte telemetry packet
directly from USART3.

MPPI supplies `linear.x` and yaw rate in `angular.z`. The adapter converts yaw
rate to front steering angle using the configured wheelbase and never requests
an in-place rotation. The protocol has no documented autonomous-mode,
emergency-stop, steering-position, motor-fault, or wheel-RPM feedback fields;
the driver does not invent these safety states.

The two complete Ackermann entry points are:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio.launch.py \
  map:=/absolute/path/to/real_map.yaml
```

These Ackermann entry points use the verified `can0` SocketCAN backend by
default. The serial backend remains available through
`chassis_driver:=ackermann_serial`.

For short-range navigation without a saved map, use the FAST-LIO rolling
costmap entry point:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio_local.launch.py
```

This mode plans in a `20 x 20 m` rolling `odom` window, uses `/scan` for both
global and local obstacle layers, and caps MPPI at `0.30 m/s`. It is intended
for immediate local goals. It does not provide persistent global coordinates
or restart-time relocalization.

For a protocol-only virtual-CAN run, use the dedicated executable and config:

```bash
ros2 run agribot_hardware_bringup ackermann_chassis_can_node --ros-args \
  --params-file $(ros2 pkg prefix agribot_hardware_bringup)/share/agribot_hardware_bringup/ackermann/config/chassis_can.yaml \
  -p can_interface:=vcan0
```

## Sensors and localization

- Leishen C16: `/lidar/points` at about 10 Hz and `/scan`
- N300Pro: `/imu/data` at about 100 Hz
- RTK position: `/rtk/fix` and `/rtk/fix_quality`
- RTK heading: `/rtk/heading`, `/rtk/heading_deg`, `/rtk/heading_valid`, and
  `/rtk/heading_solution`

The RTK driver reads `$GNGGA`, `$GNTHS`, and `#UNIHEADINGA`, verifies their
checksums, and converts clockwise-from-north heading to an ENU quaternion.
The NavSat KF-GINS configuration waits for valid RTK heading before
initialization. FAST-LIO publishes through `/fastlio/odometry`.

Configure the dedicated C16 Ethernet route once using NetworkManager, or run:

```bash
ros2 run agribot_hardware_bringup configure_c16_network.sh eno1
```

For NTRIP, keep credentials outside Git:

```bash
export NTRIP_HOST=example.invalid
export NTRIP_MOUNTPOINT=mountpoint
export NTRIP_USERNAME=user
export NTRIP_PASSWORD=password
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml enable_ntrip:=true
```

## Runtime command path and calibration

The real chassis receives `/nav2/cmd_vel` directly from the selected Nav2
controller. With chassis output enabled, selecting a Nav2 Goal in RViz
immediately starts path execution. Global planning, the local costmap, and the
controller remain responsible for obstacle avoidance. Each chassis transport
sends a zero command when the velocity stream becomes stale.

Monitor the chassis transport with:

```bash
ros2 topic echo /scout_status
ros2 topic echo /diagnostics
```

Before field motion, replace zero placeholders in `sensor_mounts.yaml`,
`kf_gins_n300pro.yaml`, `fast_lio_c16.yaml`, and `fastlio_bridge.yaml` with
measured transforms and the RTK antenna lever arm. Register the orchard map to
the ENU or FAST-LIO map frame as appropriate.

## Build and test

On another ROS 2 Humble machine, this is the only project-owned package that
must be copied into the workspace. Install its dependencies with `rosdep`;
FAST-LIO and the physical sensor driver packages must still be available as
third-party ROS packages.

```bash
cd ~/agribot_ws/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select agribot_hardware_bringup
colcon test-result --test-result-base build/agribot_hardware_bringup --verbose
```
