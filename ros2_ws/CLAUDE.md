# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Overview

Agricultural robot (Agribot) built on **ROS 2 Humble** with an AgileX Scout UGV base. The robot performs autonomous waypoint-following navigation through orchards using Nav2, with support for LiDAR SLAM (FAST-LIO, KISS-ICP), AMCL localization, and GNSS/RTK-ESKF localization.

## Build

```bash
cd agribot_humble/ros2_ws
./build_ros2_humble.sh
source ./setup_ros2_humble.sh
```

`build_ros2_humble.sh` and `setup_ros2_humble.sh` remove Conda paths and force
Ubuntu's Python 3.10 for build and runtime commands.
Only source the workspace setup after a successful build.

## Coordinate Frame Architecture

Standard REP-105: `map → odom → base_link`.

- **Hardware**: `scout_base_node` (CAN driver) publishes `odom → base_link` TF and `/odom` topic. AMCL or a localization bridge publishes `map → odom`.
- **Simulation**: `map → odom` is set by a `static_transform_publisher` at the initial pose, and Gazebo publishes `odom → base_link` via the scout base node in simulated mode.

Key transform bridges:
- `ground_truth_localization.py` — computes `map → odom` from Gazebo `/base_pose_ground_truth` and publishes it as a TF plus `/amcl_pose`
- `fastlio_odom_bridge.py` — converts FAST-LIO odometry from LiDAR frame (`camera_init → body`) to `odom → base_link`
- `navsat_pose_bridge.py` / `navsat_to_local_odom.py` — GNSS/RTK localization bridging

## Packages

### Core Autonomy

**`agribot_autonomy`** — Main autonomy package. Python nodes for waypoint running, initial pose, localization bridges, and bringup launches.

- `snake_waypoint_runner.py` — Core coverage node. Loads waypoints from YAML, builds interpolated path segments, and dispatches goals to Nav2 via `NavigateToPose` or `FollowPath` actions. Supports retry logic, proximity-based advance, and waypoint coordinate transforms.
- `initial_pose_sender.py` — Publishes `PoseWithCovarianceStamped` to `/initialpose` on startup with configurable delay, count, and interval.
- `ground_truth_localization.py` — Simulation-only: reads `/base_pose_ground_truth` + current `odom → base_link` TF, computes `map → odom`, broadcasts it and publishes `/amcl_pose`.
- `fastlio_odom_bridge.py` — Transforms FAST-LIO odometry from the LiDAR body frame to `base_link`, accounting for IMU mounting orientation.
- `kiss_localization.py` — Uses KISS-ICP scan matching to produce `map → odom`.
- `pointcloud_frame_relay.py` — Relays pointcloud messages from one frame ID to another.

**`agribot_rl_nav`** — Reinforcement learning navigation (TorchScript models).

- `depth_rl_policy_node.py` — Loads a TorchScript model and outputs `cmd_vel` from depth images, goal pose, global plan, and velocity history. Supports both single-frame and sequence-history models.
- `depth_rl_data_collector.py` — Collects synchronized (depth, RGB, scan, goal, plan, velocity, action) samples as `.npz` shards for offline training.
- `navsat_*` nodes — GNSS/RTK localization, NavSat transform publishing, and truth-error monitoring.
- `odometry_tf_broadcaster.py` — Broadcasts `odom → base_link` from `/odom` topic for use with NavSat localization.

### Scout Platform

- **`scout_base`** (`scout_base_node.cpp`) — C++ CAN driver. Publishes `/odom` and optional `odom → base_link` TF. Supports `simulated_robot` mode (integrates cmd_vel into odometry).
- **`scout_msgs`** — ROS messages for Scout base communication.
- **`ugv_sdk`** — Low-level AgileX UGV communication protocol (ament_cmake wrapper).
- **`scout_description`** — URDF and mesh files for `robot_state_publisher`.
- **`scout_control`**, **`scout_gazebo`**, **`scout_viz`**, **`scout_bringup`** — Control configs, Gazebo simulation worlds, RViz configs, and bringup utilities.

### Navigation

**`scout_navigation`** — Nav2 configuration package with launch files for AMCL, Cartographer, RTAB-Map, SLAM Toolbox, gmapping, and exploration. Includes orchard map files and costmap/planner YAML configs.

**`noah_msgs`** — Large set of custom ROS messages and services for agricultural machinery (machine control, GNSS, obstacle detection, camera/photo planning, tank control, linkage tracking, path recording). Used by the broader Noah robot system.

### SLAM and Sensors

- **`FAST_LIO_ROS2`** — FAST-LIO2 LiDAR-inertial odometry (C++). Produces `Odometry` in the LiDAR frame.
- **`kiss-icp`** — KISS-ICP scan matching for mapless lidar odometry.
- **`KF-GINS`** — Kalman Filter GNSS/INS integration.
- **`livox_ros_driver2`** — Livox LiDAR driver.
- **`lslidar_driver`** / **`lslidar_msgs`** — LSLiDAR driver stack.
- **`ydlidar_ros2_driver`** / **`YDLidar-SDK`** — YDLidar driver stack.

## Navigation Modes

The system supports several localization/navigation configurations, selected via launch files and param files:

| Mode | Localization | Params File |
|------|-------------|-------------|
| AMCL + pre-built map | AMCL | `nav2_params.yaml` |
| Mapless (KISS-ICP) | KISS-ICP scan matching | `nav2_params_mapless.yaml` |
| Mapless (FAST-LIO static) | FAST-LIO with static TF | `nav2_params_fastlio_static.yaml` |
| GNSS/RTK + ESKF | NavSat + robot_localization EKF | `nav2_params_navsat_static.yaml` |

## Key Config Files

- `agribot_autonomy/config/nav2_params.yaml` — AMCL + Nav2 parameters for map-based navigation
- `agribot_autonomy/config/nav2_params_mapless.yaml` — Mapless navigation (KISS-ICP)
- `agribot_autonomy/config/nav2_params_fastlio_static.yaml` — FAST-LIO with static odom→base_link TF
- `agribot_rl_nav/config/nav2_params_navsat_static.yaml` — NavSat/RTK-GNSS navigation
- `agribot_rl_nav/config/navsat_kf_gins_map.yaml` — ESKF IMU/GNSS fusion parameters
- `agribot_autonomy/config/nav2_params.yaml` — Nav2 controller and planner configuration

## Running

### Hardware Bringup (full autonomy)
```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch agribot_autonomy orchard_nav2_bringup.launch.py \
  port_name:=can0 \
  map:=/path/to/map.yaml \
  waypoint_file:=/path/to/waypoints.yaml \
  initial_pose_x:=0.0 initial_pose_y:=0.0 initial_pose_yaw:=0.0
```

### Hardware Bringup (driver only)
```bash
ros2 launch scout_base base.launch.py port_name:=can0
```

### Simulation Launch Examples
```bash
ros2 launch agribot_autonomy fast_lio_sim.launch.py   # FAST-LIO sim
ros2 launch agribot_autonomy kiss_icp_sim.launch.py   # KISS-ICP sim
```

## Waypoint File Format

YAML files with a `waypoints` list. Each waypoint has `x`, `y`, and optional `yaw` (radians). The `snake_waypoint_runner` interpolates between consecutive key waypoints. Waypoints can optionally be transformed from a source coordinate frame using `waypoint_transform_enabled`, `waypoint_source_origin_*` parameters.

## Code Patterns

- All Python ROS nodes follow the pattern: `rclpy.init()`, instantiate `Node` subclass, `rclpy.spin(node)`, `node.destroy_node()`, `rclpy.shutdown()`
- Parameters are declared via `self.declare_parameter("name", default).value`
- Action clients use `send_goal_async()` + done callbacks for non-blocking goal dispatch
- TF lookups use `tf2_ros.Buffer` + `TransformListener` with `rclpy.time.Time()` for latest available transform
- Shared math helpers (`normalize_angle`, `quaternion_to_yaw`) are duplicated across scripts rather than extracted into a common module
