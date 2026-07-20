# Agribot hardware bringup

This package is the physical-vehicle entry point for sensors, localization,
Nav2, command safety, and SocketCAN chassis control. It supports these tested
navigation selections:

| Vehicle | Controller | Localization | Launch file |
| --- | --- | --- | --- |
| Differential | DWB | NavSat/KF-GINS | `differential_dwb_navsat.launch.py` |
| Differential | DWB | FAST-LIO | `differential_dwb_fastlio.launch.py` |
| Ackermann | MPPI | NavSat/KF-GINS | `vehicle_autonomy.launch.py` |
| Ackermann | MPPI | FAST-LIO | `vehicle_autonomy.launch.py` |

The dedicated differential launch files default to CAN output disabled. The
Nav2 command path is:

```text
/nav2/cmd_vel -> collision monitor -> /nav2/cmd_vel_safe
  -> vehicle command gate -> /hardware/cmd_vel -> chassis driver
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

The driver receives `geometry_msgs/msg/Twist` on `/hardware/cmd_vel` and
publishes:

- `/wheel/odometry`: odometry integrated from left/right motor RPM
- `/scout_status`: common chassis feedback used by preflight
- `/hardware/chassis_e_stop`: decoded controller emergency-stop state
- `/diagnostics`: freshness, fault, checksum, counter, replay, and I/O status

Important dimensions and drivetrain values are in `config/chassis_can.yaml`:
`track_width_m`, `wheel_diameter_m`, `reduction_ratio`, and `max_motor_rpm`.
Measure and verify them before physical motion.

## Differential DWB navigation

NavSat/KF-GINS, DWB, sensors, collision monitor, and RViz:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py
```

FAST-LIO, DWB, sensors, collision monitor, and RViz:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_fastlio.launch.py
```

Both use a maximum linear speed of `0.8 m/s` and maximum angular speed of
`1.4 rad/s`. FAST-LIO consumes `/lidar/points` and `/imu/data` for
localization. In both modes, obstacle avoidance consumes `/scan`, which is the
horizontal projection published by the C16 driver on the real vehicle.

To inspect the complete navigation stack without opening SocketCAN, leave the
default `enable_can_output:=false`. To test its output path without hardware:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py \
  enable_can_output:=true chassis_driver:=simulated
```

After the controller bitrate, vehicle dimensions, wheel directions, emergency
stop, and lifted-wheel test have been confirmed, enable the supplied
differential controller:

```bash
ros2 launch agribot_hardware_bringup differential_dwb_navsat.launch.py \
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
ros2 run agribot_hardware_bringup chassis_can_node --ros-args \
  --params-file $(ros2 pkg prefix agribot_hardware_bringup)/share/agribot_hardware_bringup/config/chassis_can.yaml \
  -p can_interface:=vcan0
```

The node sends at 10 Hz, matching the proven implementation and the
workbook's recommended 100-300 ms interval. Motion is independently blocked
when the command is older than 0.25 s, required feedback is older than 1.2 s,
the controller is not in autonomous mode, emergency stop is active, or a
reported chassis/motor fault is present. Shutdown sends three brake frames.

## Ackermann reference protocol

No Ackermann controller specification was supplied. The Ackermann codec is
therefore a clearly isolated reference implementation, not a claim of
compatibility with a physical controller:

- Command ID `0x515`, feedback ID `0x535`
- Byte 0: enable and brake bits
- Bytes 1-2: signed speed, `0.001 m/s`, Intel order
- Bytes 3-4: signed steering angle, `0.001 rad`, Intel order
- Byte 5: headlight bit
- Byte 6: rolling counter; byte 7: XOR checksum

It converts `Twist` yaw rate to steering angle using the configured wheelbase
and never requests an in-place rotation. The node and unified launch both
refuse this backend unless `allow_unverified_ackermann_protocol:=true` is set.
Replace the reference IDs and signal layout after receiving the real
Ackermann controller document; do not enable it on hardware based only on this
example.

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
  enable_ntrip:=true
```

## Safety and calibration

Real CAN output is opt-in. The command gate requires current preflight and
chassis feedback, a clear software and hardware emergency stop, a fresh finite
velocity command, and `/safety/drive_enable:=true`. The CAN node repeats those
checks independently before encoding motion.

Monitor or stop the command path with:

```bash
ros2 topic echo /hardware/preflight_status
ros2 topic echo /hardware/command_output_active
ros2 topic echo /diagnostics
ros2 topic pub --once /safety/e_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /safety/drive_enable std_msgs/msg/Bool '{data: false}'
```

Before field motion, replace zero placeholders in `sensor_mounts.yaml`,
`kf_gins_n300pro.yaml`, `fast_lio_c16.yaml`, and `fastlio_bridge.yaml` with
measured transforms and the RTK antenna lever arm. Register the orchard map to
the ENU or FAST-LIO map frame as appropriate.

## Build and test

```bash
cd ~/agribot_ws/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select agribot_hardware_bringup
colcon test-result --test-result-base build/agribot_hardware_bringup --verbose
```
