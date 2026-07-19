# Agribot ROS 2 Humble

Source workspace for the Agribot Scout platform, Nav2 autonomy, Gazebo
simulation, GNSS/RTK-ESKF localization, FAST-LIO, KISS-ICP, and LiDAR drivers.

The repository is intended to be cloned and rebuilt on another ROS 2 Humble
machine. Generated `build`, `install`, and `log` directories are deliberately
not versioned.

## Supported Environment

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo Classic 11
- Python 3.10 from Ubuntu, without a Conda overlay
- Tested on NVIDIA Jetson AGX Orin (`aarch64`)

The sources contain no architecture-specific build output. An `x86_64`
Ubuntu 22.04/Humble host can build the same workspace after installing its
native dependencies.

## Clone And Build

```bash
git clone https://github.com/Guanzhen-C/agribot_humble.git
cd agribot_humble/ros2_ws

./install_ros2_humble.sh
./build_ros2_humble.sh
source ./setup_ros2_humble.sh
```

`install_ros2_humble.sh` installs ROS, Nav2, rosdep dependencies, and the build
tools. It requires sudo and network access. Allow at least 5 GB of free disk
space for generated build artifacts.

For a clean rebuild:

```bash
cd ros2_ws
rm -rf build install log
./build_ros2_humble.sh
```

## MPPI NavSat Simulation

This is the primary Ackermann simulation used for the current project:

```bash
cd ros2_ws
source ./setup_ros2_humble.sh
ros2 launch agribot_ackermann_mppi \
  ackermann_waypoint_depth_collect_sim.launch.py \
  gui:=false headless:=true rviz:=true \
  use_static_map:=true localization_mode:=navsat \
  enable_slam_map:=false map_file:=orchard_v2_map6.yaml
```

The launch file resolves maps, models, RViz configs, and parameter files from
the installed package share directories. It does not depend on the clone path.

Other supported localization values include `amcl`, `ground_truth`, and
`fast_lio`. The RPP variant is provided by `agribot_ackermann_rpp`.

## Physical Sensors And MPPI

The tested Jetson sensor interface is:

- Leishen C16 at `192.168.1.200`, UDP `2368/2369`
- N300Pro/HI13 at `115200`, default HI91 binary stream at `100 Hz`
- RTK receiver using NMEA GGA at `115200`

Install the IMU permission rule once, then log out and back in:

```bash
sudo cp install/hipnuc_imu/share/hipnuc_imu/udev/99-agribot-hipnuc.rules \
  /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo usermod -aG dialout "$USER"
```

Configure the dedicated C16 Ethernet route after boot without changing the
Wi-Fi subnet, then preview all connected sensors:

```bash
sudo ros2 run agribot_hardware_bringup configure_c16_network.sh eno1
ros2 launch agribot_hardware_bringup sensors.launch.py rviz:=true
```

The normalized topics are `/lidar/points`, `/scan`, `/imu/data`, and
`/rtk/fix`. Start either physical-vehicle MPPI stack with:

```bash
ros2 launch agribot_ackermann_fastlio_mppi fastlio_mppi.launch.py
ros2 launch agribot_ackermann_navsat_mppi navsat_mppi.launch.py
```

Both launches keep physical command output disabled by default. They publish
the collision-monitored command on `/nav2/cmd_vel_safe`; set
`enable_chassis_output:=true` only after the chassis adapter and emergency stop
have been verified. Before field use, replace the zero sensor transforms and
FAST-LIO extrinsics with measured installation values.

The RTK launch reads optional NTRIP credentials from `NTRIP_HOST`,
`NTRIP_MOUNTPOINT`, `NTRIP_USERNAME`, and `NTRIP_PASSWORD`. Do not store these
values in versioned YAML files.

## Legacy Scout Hardware

Driver only:

```bash
cd ros2_ws
source ./setup_ros2_humble.sh
ros2 launch scout_base base.launch.py port_name:=can0
```

Full autonomy:

```bash
ros2 launch agribot_autonomy orchard_nav2_bringup.launch.py \
  port_name:=can0 \
  map:="$(ros2 pkg prefix agribot_autonomy)/share/agribot_autonomy/maps/orchard_v2_map6.yaml" \
  waypoint_file:="$(ros2 pkg prefix agribot_autonomy)/share/agribot_autonomy/config/orchard_waypoints_inrow.yaml" \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

CAN setup and device permissions remain host-specific. The optional
`recover_board_eth.sh` script accepts its interface, address, board IP, and
user through environment variables.

## Repository Contents

Versioned:

- ROS 2 package sources and package manifests
- Required vendored C/C++ dependencies
- Nav2, localization, sensor, and controller configuration
- Orchard maps, Gazebo worlds, meshes, and referenced textures
- URDF/SDF models, RViz configurations, launch files, and build scripts

Not versioned:

- `build/`, `install/`, `log/`, Python caches, and compiler output
- rosbag, PCD, training datasets, trained policy files, and runtime logs
- Local parameter dumps, machine-specific command notes, and credentials
- Large upstream screenshots, GIFs, PDFs, and manuals not used at runtime

RL datasets and models default to `~/.local/share/agribot/`; pass explicit
launch or training arguments when another storage location is preferred.
