# agribot_autonomy

This package provides waypoint-based orchard coverage helpers on top of the
ROS 2 Scout/Nav2 stack.

## What It Does

- Uses waypoint files as the human-defined global route and leaves path execution to Nav2.
- Publishes either ground-truth-based localization or an initial pose for AMCL workflows.
- Sends a fixed orchard waypoint sequence to the Nav2 `navigate_to_pose` action server.
- Supports both static-map AMCL coverage and ground-truth coverage with a rolling global costmap.
- When `use_ground_truth_localization:=false`, Nav2 map server and AMCL are started automatically.

## Launch

Start the full orchard simulation and waypoint coverage flow:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_autonomy orchard_waypoint_coverage_sim.launch.py
```

To run only the navigation side against an already running world:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_autonomy orchard_waypoint_coverage.launch.py
```

To keep the previous direct Nav2 bringup entrypoint:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_autonomy orchard_nav2_bringup.launch.py
```

### Ackermann MPPI Localization Modes

The Ackermann MPPI simulation accepts `navsat`, `fast_lio`, or `fast_livo2`
through the same launch argument. To run FAST-LIVO2 with Gazebo and RViz:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 launch agribot_ackermann_mppi \
  ackermann_waypoint_depth_collect_sim.launch.py \
  localization_mode:=fast_livo2 gui:=true rviz:=true
```

FAST-LIVO2 consumes `/points`, `/imu/data_corrected`, and
`/camera/color/image_raw`. Its base-frame odometry is published on
`/fastlivo/odometry` and is connected to the same Nav2 MPPI stack used by the
other localization modes.

## Main Files

- `launch/orchard_waypoint_coverage.launch.py`: coverage flow with either AMCL or ground-truth localization
- `launch/orchard_waypoint_coverage_sim.launch.py`: orchard world + coverage launch + RViz
- `launch/orchard_nav2_bringup.launch.py`: direct Nav2 bringup entrypoint retained for the simpler pure-Nav2 workflow
- `launch/fast_livo2_sim.launch.py`: FAST-LIVO2, IMU-frame correction, and base-frame odometry bridge
- `scripts/snake_waypoint_runner.py`: sequentially sends Nav2 `navigate_to_pose` goals from YAML
- `config/orchard_waypoints_default_start.yaml`: full orchard snake route from the default spawn point
- `config/orchard_waypoints_inrow.yaml`: shorter in-row waypoint sequence
