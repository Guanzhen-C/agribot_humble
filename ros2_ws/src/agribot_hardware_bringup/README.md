# Agribot hardware bringup

This package is the physical-vehicle entry point for sensors, localization,
Nav2, command safety, and CAN/serial chassis control. It provides these
navigation entry points; the differential stack is software-validated but
remains calibration-gated until its physical measurements are supplied:

| Vehicle | Controller | Localization | Launch file |
| --- | --- | --- | --- |
| Differential | MPPI DiffDrive | FAST-LIVO2 + fixed RTK + saved map | `differential_mppi_fastlivo_rtk_mapped.launch.py` |
| Differential | State Lattice planner test | Saved 2D map | `differential_state_lattice_validation.launch.py` |
| Differential | None | FAST-LIO 3D/2D mapping | `differential_3d_mapping.launch.py` |
| Ackermann | MPPI | NavSat/KF-GINS | `ackermann_mppi_navsat.launch.py` |
| Ackermann | MPPI | FAST-LIO local rolling map | `ackermann_mppi_fastlio_local.launch.py` |
| Ackermann | MPPI | FAST-LIO + 3D PCD mapping | `ackermann_mppi_fastlio_3d_mapping.launch.py` |
| Ackermann | MPPI | FAST-LIO + saved-map planning | `ackermann_mppi_fastlio_mapped.launch.py` |
| Ackermann | MPPI | FAST-LIVO2 + fixed RTK + saved map | `ackermann_mppi_fastlivo_rtk_mapped.launch.py` |
| Ackermann | MPPI | Outdoor 0811 verified map profile | `ackermann_outdoor_0811_experiment.launch.py` |
| Ackermann | None | Raw C16 + IMU + RTK + RGB-D collection | `ackermann_sensor_data_collection.launch.py` |

Vehicle-specific physical code is kept in separate source trees:

```text
can/           SocketCAN/ZQWL CDC transports, ROS topics, diagnostics and frame utilities
differential/  differential protocol, adapter, executable, config, launch and tests
ackermann/     Ackermann protocol, MPPI config, behavior trees, launch and tests
localization/  NavSat/KF-GINS node and localization bridge scripts
```

All project-owned runtime code and resources used by these entry points
are contained in this package. The remaining package dependencies are ROS 2
system components or third-party device/algorithm packages: Nav2, RViz,
FAST-LIO, `hipnuc_imu`, `lslidar_driver`, PCL, STVL, and their message packages.
The CAN status interface also uses the third-party `scout_msgs` package.

No simulation map is bundled with the physical-vehicle package. Mapped
navigation receives a real map base path whose `.pcd`, `.yaml`, and image files
were generated together. The differential outdoor RTK entry additionally
requires the matching `_georeference.yaml` file.

The installed chassis executables remain vehicle-specific; there is no mixed
vehicle executable.

When enabled, Ackermann launches publish `urdf/ackermann_vehicle.urdf` for
RViz. Its multi-material GLB meshes are converted from the measured vehicle
STEP assembly and retain the assembly colors and detailed suspension geometry.
The body and four original wheel assemblies are separate visual meshes. The
URDF converts it to ROS axes (`+X` forward, `+Y` left, `+Z` up) and centers it
at the rear axle to match `base_link`. It is visual-only; Nav2 continues to use
the configured footprint for collision checking.

Physical Ackermann launches preserve the original pose/TF-only RViz display by
default. Pass `use_detailed_vehicle_model:=true` to load the STEP-derived body,
wheel and sensor visuals. This option changes visualization only; it does not
change `base_link`, calibrated sensor transforms, Nav2 geometry or control.

`ackermann_joint_state_publisher` uses measured `/wheel/odometry` velocity to
animate the four wheel rotations and infer the left/right Ackermann steering
angles. Its front-wheel joint hierarchy matches the simulation: steering about
`+Z`, followed by rolling about `+Y`. The chassis protocol does not expose a
standalone steering-position field, so the last inferred steering angle is held
while stationary.

The STEP/URDF geometry is the canonical source for physical Ackermann
kinematics: mean wheelbase `0.5265855 m`, front/rear tracks `0.589931 m` and
`0.590517 m`, nominal wheel radius `0.1275 m`, and maximum steering angle
`0.384 rad`. This gives a `1.303241 m` minimum bicycle-model turning radius.
The CAN, serial, MPPI, Smac Hybrid-A*, joint visualization, and Nav2 footprint
configurations use these same values.

The same robot description also displays the installed HiPNUC N300 Pro IMU
and the side-outlet LeiShen C16 V4.0 lidar. Their model centers follow
`config/sensor_mounts.yaml`: `imu_link` is at `(0.1425, 0, 0.143)` m and the
C16 optical-center frame `lidar_link` is at `(0.48, 0, 0.233)` m relative to
the rear-axle-centered `base_link`. After the rigid IMU mount replacement, a
2026-08-05 level-floor calibration set the IMU RPY to
`(0.000572424, -0.009139547, -0.000002616)` rad and the C16 RPY to
`(-0.007648487, -0.001835661, 0.000007020)` rad. FAST-LIO, LIO-SAM,
the odometry bridge, sensor TFs and the KF-GINS wrapper use the corresponding
rotations consistently. The left RTK master antenna measurement
    point is at `(0.1425, 0.2952585, 0.78476)` m. The right secondary antenna is
    at `(0.1425, -0.2952585, 0.78476)` m, giving a `0.590517 m` lateral baseline
    and an IMU-to-master-antenna lever arm of `(0, 0.2952585, 0.64176)` m. These
configuration vectors use ROS FLU;
the NavSat wrapper converts the antenna lever arm to KF-GINS FRD, initializes
the filter state at the IMU center, and publishes the resulting pose and twist
at the rear-axle-centered `base_link`. The physical-navigation RViz profiles
show the sensor axes; these visuals do not publish or replace the calibrated TFs.
The vehicle mesh removes the STEP assembly's obsolete M10 lidar and the rear
auxiliary assembly. The C16 uses the rear carrier-plate shape at its calibrated
position, with four pillars extended down to the chassis mounting surface.

The dedicated differential launch files default to CAN output disabled. The
Nav2 command path is:

```text
/nav2/cmd_vel -> chassis driver
```

## Differential CAN protocol

The differential driver implements the `伽马底盘` sheet in
`主控到分控数据格式V2.4.xlsx`. The workbook is authoritative for signal
positions, Intel byte order, rolling counter, XOR checksum, and the default
250 kbit/s CAN bitrate.

| Direction | CAN ID | Content |
| --- | --- | --- |
| TX | `0x514` | Independent brakes, signed left/right PWM, headlight, and turn lights |
| RX | `0x532` | Mode, emergency stop, motion, remote connection, lights, and battery voltage |
| RX | `0x533` | Left motor faults/protections, direction, brakes, speed, and current |
| RX | `0x534` | Right motor faults/protections, direction, brakes, speed, and current |

All frames are standard 11-bit, 8-byte CAN frames. Byte 6 low nibble is the
rolling counter and byte 7 is XOR of bytes 0 through 6. Invalid checksums and
unchanged replay counters are rejected. Counter jumps are accepted as dropped
frames but recorded in `/diagnostics`.

Command byte 0 bits 0 and 1 are the workbook-defined left and right brake
commands. Chassis battery voltage is the one-byte physical value in byte 2.
Motor speed is an unsigned 16-bit magnitude in bytes 2-3; byte 1 bit 5 supplies
its direction. Running current is a signed 16-bit value in bytes 4-5. Reserved
bits and bytes are transmitted as zero and ignored on reception.

The driver command topic is configurable. The unified navigation launch routes
`geometry_msgs/msg/Twist` from `/nav2/cmd_vel` directly to the selected
driver. The driver publishes:

- `/wheel/odometry`: odometry integrated from the signed left/right speed feedback
- `/scout_status`: common chassis feedback
- `/hardware/chassis_e_stop`: decoded controller emergency-stop state
- `/diagnostics`: freshness, fault, checksum, counter, replay, and I/O status

Important dimensions and drivetrain values are in
`differential/config/chassis_can.yaml`:
`track_width_m`, `command_full_scale_wheel_speed_mps`, and
`feedback_wheel_speed_mps_per_speed_unit`. Measure and verify them before
physical motion because the workbook does not state the speed field's unit or
the PWM-to-vehicle-speed conversion.

## Differential State Lattice and MPPI navigation

The differential stack uses Smac State Lattice with the Humble 5 cm
differential motion-primitive set and MPPI's `DiffDrive` model. It supports
in-place rotation and reverse motion while checking the complete rectangular
footprint. Both local and global costmaps consume `/lidar/points` through STVL;
no legacy single-ring `/scan`, AMCL, NavFn, or DWB component is used.

The checked-in geometry, wheel parameters, speed limits, and sensor transforms
are provisional placeholders. `differential/config/vehicle_calibration.yaml`
therefore has `calibration_complete: false`, and the full launch refuses to
create the physical chassis node until calibration is marked complete.

Run the full stack in observation-only mode first:

```bash
ros2 launch agribot_hardware_bringup \
  differential_mppi_fastlivo_rtk_mapped.launch.py \
  map_base:=/absolute/path/to/real_map \
  initialization_source:=manual \
  enable_chassis_output:=false rviz:=true
```

After geometry, motor directions, limits, emergency stop, feedback freshness,
and lifted-wheel motion have all been verified, physical motion additionally
requires the exact authorization string:

```bash
ros2 launch agribot_hardware_bringup \
  differential_mppi_fastlivo_rtk_mapped.launch.py \
  map_base:=/absolute/path/to/real_map \
  enable_chassis_output:=true \
  motion_authorization:=ENABLE_DIFFERENTIAL_MOTION
```

See `differential/README.md` for sensor validation, mapping, offline planning,
outdoor RTK initialization, and USB-CAN commands.

## SocketCAN setup

The V2.4 workbook specifies a default bus bitrate of 250 kbit/s. Configure a
native CAN interface with the bitrate required by the selected chassis:

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
when the command is older than 0.25 s, required feedback is older than 0.6 s,
the controller is not in autonomous mode, emergency stop is active, or a
reported chassis/motor fault is present. Shutdown sends three brake frames.

## Ackermann C50C protocol

The Ackermann backend supports the WHEELTEC C50C classic CAN protocol through
either native SocketCAN or the ZQWL-CANFD USB CDC bridge. Its USART3 transport
remains available as a separate fallback. The CAN bus runs at 1 Mbit/s:

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

The NavSat static-map Ackermann entry point is:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  map:=/absolute/path/to/real_map.yaml
```

These Ackermann entry points default to `chassis_driver:=ackermann_can` with
the `zqwl_cdc` transport and this stable USB device path:

```text
/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

The driver configures channel 0 for classic CAN at 1 Mbit/s. Native SocketCAN
remains available through `can_transport:=socketcan can_interface:=can0`, and
the chassis serial fallback through `chassis_driver:=ackermann_serial`.
Every ZQWL open performs one deterministic channel initialization: send STOP,
wait 100 ms, discard stale adapter input, then send the verified START
configuration. This clears a channel state left active or bus-off by an
earlier process. Motion remains inhibited until fresh chassis feedback arrives.

For short-range navigation without a saved map, use the FAST-LIO rolling
costmap entry point:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_fastlio_local.launch.py
```

This mode plans in a `20 x 20 m` rolling `odom` window, uses the complete C16
cloud through STVL for both global and local obstacle layers, and caps MPPI at
`0.30 m/s`. It is intended for immediate local goals. It does not provide
persistent global coordinates or restart-time relocalization.

For voxelized 3D PCD mapping while FAST-LIO supplies odometry:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_mppi_fastlio_3d_mapping.launch.py \
  map_base:=/home/sunrise/agribot_maps/test_site/map \
  rviz:=true \
  enable_chassis_output:=false
```

For offline mapping datasets, collect only the physical sensor streams on the
RDK. This entry starts the C16, IMU, RTK, stereo camera right eye and rosbag
recorder. Install the stable right-camera device rule once after building:

```bash
sudo install -m 0644 \
  /home/sunrise/agribot_ws/ros2_ws/install/agribot_hardware_bringup/share/agribot_hardware_bringup/udev/99-agribot-right-camera.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger --subsystem-match=video4linux
readlink -f /dev/agribot_right_camera
```

The last command must resolve to the `Stereo Vision 1` capture endpoint. The
launch uses this stable name instead of a changeable `/dev/videoN` number.
It does not start FAST-LIO, Nav2, the online PCD builder, RViz or a chassis
driver:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_sensor_data_collection.launch.py \
  start_camera:=true \
  bag_output:=/mnt/agribot_data/bags/test_$(date +%Y%m%d_%H%M%S)
```

The bag includes the full C16 point cloud, lidar timing/device information,
IMU, magnetic field, temperature, every raw RTK serial sentence, parsed RTK
quality fields, headings, the raw 640 x 480 right-eye image and camera-info
messages, and static sensor transforms. It does not record a depth stream.
Calibrate this camera's intrinsics and camera-to-lidar extrinsics before using
the images for FAST-LIVO2. Run FAST-LIO, FAST-LIVO2 or LIO-SAM later on the
Jetson from this bag. Control the vehicle through the existing independent
manual controller while collecting data.

### C16 gPTP time synchronization

The RDK X5 serves its UTC system clock to the C16 over the dedicated `eth0`
link. The system clock is disciplined by RTK RMC/ZDA plus GPIO PPS when those
signals are available; otherwise it continues from the RTC and chrony
holdover. Install and start the two host services once after building:

```bash
sudo apt install chrony ethtool linuxptp
source /opt/ros/humble/setup.bash
source /home/sunrise/agribot_ws/ros2_ws/install/setup.bash
ros2 run agribot_hardware_bringup install_c16_gptp.sh eth0
```

Configure the lidar itself once while `lslidar_driver_node` is running:

```bash
ros2 service call /time_mode lslidar_msgs/srv/TimeMode \
  "{time_mode: 1, ntp_ip: ''}"
```

Restart the lidar driver after that call. The production C16 configuration
uses its internal PTP timestamp. Check both host services and the live cloud:

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/agribot_ws/ros2_ws/install/setup.bash
ros2 run agribot_hardware_bringup check_c16_gptp.sh eth0
```

Do not replace the PHC command's `-O 0` with `-w`. This C16 firmware writes
received PTP seconds directly into packets, while ROS interprets them as UTC;
using the normal TAI offset would make cloud timestamps 37 seconds fast in
2026.

This entry point accumulates FAST-LIO's registered 3D cloud into a filtered
PCD map. The first rear-axle pose is the map origin. Chassis output and Nav2
default to disabled during mapping so one external manual controller can own
the chassis. The default rear exclusion mask removes points between
`x=-4.0..-0.12 m` and `y=-0.60..0.60 m` in the current `base_link` frame so a
following operator is not written into the static map.

Save the map before stopping the launch:

```bash
ros2 service call /pcd_map_builder/save_map std_srvs/srv/Trigger "{}"
```

This creates `map.pcd` plus an aligned `map.pgm` and `map.yaml`. The projection
can also be regenerated from the PCD with a different height band:

```bash
ros2 run agribot_hardware_bringup pcd_to_nav2_map.py \
  /home/sunrise/agribot_maps/test_site/map.pcd \
  /home/sunrise/agribot_maps/test_site/map
```

The base path must contain `.pcd`, `.pgm`, and `.yaml`. Nav2 uses the
two-dimensional projection for planning, while one-shot initial localization
uses the complete three-dimensional PCD. Start mapped navigation with the base
path:

Before sensors are available, the saved YAML can be checked independently with
the planner-only entry point:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_smac_planner_validation.launch.py \
  map:=/home/cgz/agribot_maps/test_site/map.yaml
```

In RViz, set a start with `2D Pose Estimate`, then use `2D Goal Pose` one or
more times to append ordered waypoints. The bridge sends the explicit start and
all waypoints to Nav2 `ComputePathThroughPoses` using the same Smac Hybrid-A*
configuration as the physical Ackermann vehicle. It publishes the waypoint
queue on `/planning_test/waypoints` and the complete result on
`/planning_test/path`. Setting a new start clears the existing waypoint queue.
This launch contains only the map server, planner server, lifecycle manager,
bridge and RViz; it cannot command a chassis and does not start any sensor,
localization, controller or behavior-tree node.

To validate the exact native RViz `Nav Through Poses` workflow without moving a
vehicle, use:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_nav_through_poses_validation.launch.py \
  map:=/home/cgz/agribot_maps/test_site/map.yaml
```

This entry point starts the real Nav2 panel, `NavigateThroughPoses` behavior
tree, and Smac planner. A dry-run `FollowPath` server displays the single
continuous path it receives and returns success without creating a controller,
sensor, chassis node, or velocity publisher.

Start the complete mapped navigation stack with the base path:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_mppi_fastlio_mapped.launch.py \
  map_base:=/home/sunrise/agribot_maps/test_site/map \
  rviz:=true \
  enable_chassis_output:=true
```

Keep the vehicle still during startup and use RViz `2D Pose Estimate` to give an
accurate nearby position and heading. The localizer aggregates five stationary
FAST-LIO body scans, crops an `8 m` PCD submap around the prior, runs PCL NDT at
two resolutions, and finishes with PCL GICP. A result is accepted only when its
overlap, inlier RMSE, tilt, height, and correction from the RViz prior pass the
configured limits. Repetitive corridors do not provide enough geometry to
recover a large longitudinal or 180-degree prior error, so the RViz prior is a
required place and direction constraint, not an optional hint.

On success the node freezes `map -> odom`, publishes
`/localization/ready=true`, and releases the point-cloud subscription and
matching timer. FAST-LIO remains the only high-rate `odom -> base_link` source;
there is no motion-time NDT, GICP, particle filtering, drift correction, or
automatic relocalization. Restart the launch to perform a new initialization.
The saved YAML supplies long-range planning, while live C16 PointCloud2 data
supplies STVL obstacle marking and clearing. Monitor initialization with:

```bash
ros2 topic echo /localization/ready --once
ros2 topic echo /localization/status --once
ros2 topic echo /localization_pose --once
ros2 run tf2_ros tf2_echo map odom
```

The alternative Ackermann NavSat mapped entry keeps the same PCD initial
registration, Nav2 map, Smac planner, MPPI controller and C16 STVL layers, but
uses KF-GINS for continuous `odom -> base_link` instead of FAST-LIO:

```bash
ros2 launch agribot_hardware_bringup ackermann_mppi_navsat.launch.py \
  map:=/home/sunrise/agribot_maps/test_site/map.yaml \
  map_georeference:=/home/sunrise/agribot_maps/test_site/map_georeference.yaml \
  initialization_source:=rtk \
  enable_chassis_output:=true
```

The NavSat entry accepts a position-only georeference yaw as a coarse prior;
NDT/GICP must still accept the C16 scan before chassis motion is released.

The outdoor `map_lio_sam_0811` experiment has a dedicated safe entry point:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_outdoor_0811_experiment.launch.py \
  rviz:=true enable_chassis_output:=false
```

It starts the saved-map FAST-LIVO2/RTK localization chain, Smac Hybrid-A*,
MPPI, C16 STVL obstacle processing and RViz. Initial localization first waits
for the existing fixed-RTK seed. If that seed is unavailable or rejected by
NDT/GICP, the RDK BPU EigenPlaces node supplies map-pose candidates; only after
those candidates fail is manual 2D Pose Estimate accepted. Before any sensor
node starts, the launch verifies the PCD, processed PGM, Nav2 YAML,
georeference and visual index against the SHA-256 values in
`config/outdoor_0811_map_profile.yaml`.

Every coarse source is passed through the same two-stage NDT and GICP localizer.
The visual node never publishes `map -> odom` directly, and the chassis remains
inhibited until `/localization/lidar_ready` and `/fastlivo_rtk/ready` are true.
The visual database must be stored next to the map as
`<map_base>_visual_index.npz` and must contain `base_link` poses.

After the no-motion stage has passed, stop that launch completely and start the
same command with `enable_chassis_output:=true`. The CAN driver still blocks
nonzero commands until `/fastlivo_rtk/ready` is true. The concise stage A and B
commands are kept in the workspace `src/2.txt` command file.

The georeference can be checked without Nav2 or a chassis driver. The dedicated
validation launch displays the two-dimensional map and converts RViz Publish
Point selections from `map` coordinates to WGS84 on
`/georeference_test/clicked_geopoint`. It also accepts the left master antenna
position and clockwise-from-true-north vehicle heading as a standard
`GeoPoseStamped`. It applies the configured antenna lever arm, publishes a
yellow rear-axle seed, and passes that seed to the production two-level NDT and
GICP localizer. The accepted green rear-axle pose and its corresponding master
antenna WGS84 pose are published separately:

```bash
ros2 launch agribot_hardware_bringup \
  ackermann_georeference_validation.launch.py \
  map_base:=/absolute/path/to/map_lio_sam_0811

ros2 run agribot_hardware_bringup publish_geodetic_pose.py \
  39.977825835 116.325862855 180.0
ros2 topic echo /georeference_test/result --once
```

The launch never creates Nav2 controllers or a chassis node. NDT/GICP requires
live or replayed lidar, IMU, and FAST-LIO data; keep the vehicle stationary
until the result is accepted. Map-derived altitude remains diagnostic only
because this georeference was calibrated from horizontal RTK positions.

For a protocol-only virtual-CAN run, use the dedicated executable and config:

```bash
ros2 run agribot_hardware_bringup ackermann_chassis_can_node --ros-args \
  --params-file $(ros2 pkg prefix agribot_hardware_bringup)/share/agribot_hardware_bringup/ackermann/config/chassis_can.yaml \
  -p can_transport:=socketcan \
  -p can_interface:=vcan0
```

## Sensors and localization

- Leishen C16: `/lidar/points` at about 10 Hz
- Voxelized mapping output: `/pcd_map`
- N300Pro: `/imu/data` at about 100 Hz
- N300Pro auxiliary data: `/imu/magnetic_field` and `/imu/temperature`
- Hikrobot right camera: `/camera/rgb/image_raw` at about 10 Hz
- RTK position: `/rtk/fix` and `/rtk/fix_quality`
- RTK heading: `/rtk/heading`, `/rtk/heading_deg`, `/rtk/heading_valid`, and
  `/rtk/heading_solution`
- RTK raw and quality data: `/rtk/raw_sentence`, `/rtk/gga_utc`,
  `/rtk/satellite_count`, `/rtk/hdop`, `/rtk/differential_age`, and
  `/rtk/reference_station_id`

The RTK driver reads `$GNGGA`, `$GNTHS`, and `#UNIHEADINGA`, verifies their
checksums, and converts clockwise-from-north heading to an ENU quaternion.
The NavSat KF-GINS configuration waits for valid RTK heading before
initialization. FAST-LIO publishes through `/fastlio/odometry`.

The N300Pro and Hikrobot drivers map their device counters to the RDK ROS
clock with the shared `agribot_time_sync` affine clock mapper. RTK position
and heading use GNSS measurement time when it agrees with the RDK clock. The
C16 receives the RDK UTC clock over gPTP and publishes its hardware timestamp
while retaining per-point relative time; it no longer needs an RMC serial
input. `sensors.launch.py` starts
`sensor_time_sync_monitor.py`, which reports per-topic rates, timestamp age,
monotonicity, and nearest lidar-to-IMU/camera/RTK timestamp differences on
`/diagnostics`. These checks validate a common software time axis; camera
exposure latency still requires motion-based `img_time_offset` calibration.

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
ros2 launch agribot_hardware_bringup differential_outdoor_experiment.launch.py \
  map_base:=/absolute/path/to/real_map enable_ntrip:=true
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

Before NavSat field motion, verify the dual-antenna heading sign and replace the
RTK antenna-top approximation with the manufacturer's phase-center offset when
available. Register the orchard map to the ENU or FAST-LIO map frame as
appropriate.

## Build and test

On another ROS 2 Humble machine, keep this package together with the
project-owned `agribot_time_sync` package. Install their dependencies with
`rosdep`; FAST-LIO and the physical sensor driver packages must still be
available as third-party ROS packages.

```bash
cd ~/agribot_ws/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
colcon test --packages-select agribot_hardware_bringup
colcon test-result --test-result-base build/agribot_hardware_bringup --verbose
```
