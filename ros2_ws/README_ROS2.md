# Agribot ROS 2 Humble Workspace

## Included ROS2 Packages

- `scout_msgs`: ROS2 message definitions for the Scout base.
- `ugv_sdk`: ament-cmake build of the AgileX UGV SDK.
- `scout_base`: ROS2 Scout CAN driver node.
- `scout_description`: Scout URDF/mesh assets for `robot_state_publisher`.
- `livox_ros_driver2`, `lslidar_driver`, `ydlidar_ros2_driver`: LiDAR drivers.
- `fast_lio`, `kiss_icp`: LiDAR-inertial and scan-matching odometry.
- `gazebo_ros_pkgs` 3.9.0 and `velodyne_simulator` 2.0.3: Humble-compatible simulation dependencies vendored for ARM64.
- `agribot_autonomy`: waypoint runner, initial pose sender, ground-truth helpers, Nav2 bringup launch.
- `agribot_rl_nav`: ROS2 ports of the RL/navigation helper nodes.

## Build From A Fresh Clone

```bash
cd agribot_humble/ros2_ws
./install_ros2_humble.sh
./build_ros2_humble.sh
source ./setup_ros2_humble.sh
```

The build and setup scripts remove Conda paths before invoking Humble's system
Python 3.10. This also keeps `env python3` ROS executables on the supported
system interpreter at runtime.
Navigation2 is supplied by the system Humble installation; no bundled
Navigation2 source tree is built in this workspace.

## Hardware Bringup

```bash
cd agribot_humble/ros2_ws
source ./setup_ros2_humble.sh
ros2 launch agribot_autonomy orchard_nav2_bringup.launch.py \
  port_name:=can0 \
  map:="$(ros2 pkg prefix agribot_autonomy)/share/agribot_autonomy/maps/orchard_v2_map6.yaml" \
  waypoint_file:="$(ros2 pkg prefix agribot_autonomy)/share/agribot_autonomy/config/orchard_waypoints_inrow.yaml" \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

## Driver Only

```bash
cd agribot_humble/ros2_ws
source ./setup_ros2_humble.sh
ros2 launch scout_base base.launch.py port_name:=can0
```

See the repository-level `README.md` for the tested MPPI/NavSat simulation
command, portability notes, and the exact set of versioned runtime assets.
