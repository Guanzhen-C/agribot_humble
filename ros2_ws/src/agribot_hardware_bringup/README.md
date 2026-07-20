# Agribot hardware bringup

This package is the single runtime entry point for the physical vehicle. It
starts the sensor drivers, one localization pipeline, the static map, Nav2
MPPI, collision monitoring, command preflight, the velocity safety gate and,
when explicitly enabled, the CAN chassis driver.

## Sensor interfaces

- Leishen C16: `/lidar/points` at about 10 Hz and `/scan`
- N300Pro: `/imu/data` at about 100 Hz
- RTK position: `/rtk/fix` and `/rtk/fix_quality`
- RTK dual-antenna heading: `/rtk/heading`, `/rtk/heading_deg`,
  `/rtk/heading_valid` and `/rtk/heading_solution`

The checked-in serial configurations use the stable `/dev/serial/by-id/...`
links observed on the Jetson. The RTK driver reads `$GNGGA`, `$GNTHS` and
`#UNIHEADINGA`, verifies their checksums, and converts clockwise-from-north
heading to an ENU quaternion. The NavSat KF-GINS configuration waits for a
valid RTK heading before initialization and preserves that absolute ENU yaw.

## One launch command

NavSat/KF-GINS plus MPPI, with CAN output disabled for inspection:

```bash
ros2 launch agribot_hardware_bringup vehicle_autonomy.launch.py \
  localization:=navsat
```

FAST-LIO plus MPPI, with CAN output disabled:

```bash
ros2 launch agribot_hardware_bringup vehicle_autonomy.launch.py \
  localization:=fastlio
```

The currently available CAN backend is the AgileX Scout protocol from
`scout_base`. Use the following only if the physical controller is confirmed
to be protocol-compatible:

```bash
ros2 launch agribot_hardware_bringup vehicle_autonomy.launch.py \
  localization:=navsat enable_can_output:=true \
  chassis_driver:=scout can_interface:=can0
```

This is still one launch command from this package. Setting
`enable_can_output:=true` is intentionally explicit. It starts the chassis
driver and arms the command gate, but motion remains blocked until every
required sensor, localization output, CAN interface and `/scout_status`
feedback is current. `/safety/e_stop:=true`, `/safety/drive_enable:=false`, a
stale input, an invalid number, or failed preflight forces zero velocity.
`scout_base` has an independent 0.25 second watchdog and also sends zero on
shutdown.

Monitor the safety state with:

```bash
ros2 topic echo /hardware/preflight_status
ros2 topic echo /hardware/command_output_active
```

Emergency stop and software drive enable are standard Boolean topics:

```bash
ros2 topic pub --once /safety/e_stop std_msgs/msg/Bool '{data: true}'
ros2 topic pub --once /safety/drive_enable std_msgs/msg/Bool '{data: false}'
```

## One-time machine setup

Configure the dedicated LiDAR Ethernet route once using NetworkManager, or
run this after boot:

```bash
ros2 run agribot_hardware_bringup configure_c16_network.sh eno1
```

After confirming the controller bitrate, configure SocketCAN:

```bash
ros2 run agribot_hardware_bringup configure_can.sh can0 500000
```

These commands request `sudo`; do not put a password in launch files. For a
true single-command boot, persist the Ethernet profile and CAN interface in
NetworkManager/systemd on the vehicle computer.

For NTRIP, keep credentials outside Git:

```bash
export NTRIP_HOST=example.invalid
export NTRIP_MOUNTPOINT=mountpoint
export NTRIP_USERNAME=user
export NTRIP_PASSWORD=password
ros2 launch agribot_hardware_bringup vehicle_autonomy.launch.py \
  localization:=navsat enable_ntrip:=true
```

## Required calibration

Before field motion, replace all zero placeholders in `sensor_mounts.yaml`,
`kf_gins_n300pro.yaml`, `fast_lio_c16.yaml` and `fastlio_bridge.yaml` with the
measured vehicle transforms and RTK antenna lever arm. The static orchard map
must also be registered to the ENU/FAST-LIO map frame.

`scout_base` controls a Scout skid-steer base with linear and angular velocity;
it is not a generic Ackermann CAN protocol. If the real chassis is not an
AgileX Scout, its CAN specification (IDs, byte layout, units, checksum,
counter, bitrate and feedback frames) is required before enabling output. Add
that adapter behind `/hardware/cmd_vel`; the sensor, localization, MPPI,
preflight and safety portions do not need to change.
