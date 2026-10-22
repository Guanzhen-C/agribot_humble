# K2 Scanned Campus Simulation

This package reuses the existing private K2 LCC2 decoder output. Raw archives,
decoded Gaussians, derived meshes and maps stay outside Git.

The physics and CPU-ray lidar use all original K2 mesh triangles. Appearance
uses all original highest-resolution Gaussians plus environment, through the
reference EWA renderer. No mesh appearance proxy, color baking or extra Gaussian
downsampling is used. GUI and server render status are recorded separately;
server camera images must be checked, not inferred from the GUI screenshot.

Preparation preserves the source files and refuses to overwrite an output:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 run agribot_k2_sim prepare_k2_scene \
  --reference /home/cgz/3dgs/gazebo-k2-automation-20261022 \
  --archive /home/cgz/Downloads/其域k2数据/K2数据自动化所_render2.zip \
  --output ~/.local/share/agribot/k2_automation_gaussian_20261022
```

Launch in a domain and Gazebo master separate from real vehicles and other scenes:

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=86
export ROS_LOCALHOST_ONLY=1
export GAZEBO_MASTER_URI=http://127.0.0.1:11375
export IGN_PARTITION=agribot_k2_20261022
export GZ_PARTITION=$IGN_PARTITION
ros2 launch agribot_k2_sim k2_ackermann.launch.py gui:=true rviz:=true
```

The default is stationary until a goal is selected in RViz. Set
`run_waypoints:=true` to opt into the prepared short route. This runs the existing
configured Ackermann vehicle, FAST-LIVO2 + simulated RTK, Smac Hybrid-A* and MPPI.
The one-time settled spawn anchor is the existing simulation initialization, not
an NDT/GICP relocalization test. Simulated GNSS is not the K2 capture's GNSS.

The running scene can be inspected without publishing motion commands:

```bash
ros2 run agribot_k2_sim check_k2_sensors --seconds 12 --output /tmp/k2_sensor_check
```

The same domain/localhost environment above is required. A nonzero `--distance`
for `verify_running_k2` explicitly requests a short simulated motion test; do not
run that tool while a user is selecting goals in RViz.

Current checks: full Gaussians render in both the GUI and native Gazebo RGB
camera; original-mesh CPU lidar, IMU and simulated GNSS publish valid streams
with available mounting TFs. Camera timing jitter and bounded FAST-LIVO2 frame
drops remain; this is not a full-campus navigation or precise synchronization
acceptance. The earlier textured-mesh smoke result does not validate the current
full-Gaussian scene. The later 3 m command in a running scene was aborted and
must not be reported as a passed navigation test.
