# agribot_bev

ROS 2 Humble node for a symmetric front/left/rear/right stereographic camera
rig. It uses the known camera geometry to precompute four ground-plane lookup
tables, blends synchronized images, and publishes:

- `/bev/image` (`sensor_msgs/Image`, BGR8)
- `/bev/valid_mask` (`sensor_msgs/Image`, MONO8)
- `/bev/ground_grid` (`nav_msgs/OccupancyGrid`, 10 cm/cell by default)

The grid is local to `base_footprint`. Cells outside the observed range are
unknown. With point-cloud fusion disabled, valid cells are geometric ground
observations, not a learned traversability classification. With fusion enabled,
fresh points between the configured obstacle height limits are marked occupied;
if the cloud becomes stale the grid is published unknown rather than falsely
cleared.

For the current Gazebo rig:

```bash
ros2 launch agribot_bev surround_bev.launch.py \
  fuse_pointcloud:=true pointcloud_topic:=/points
```

For the real robot, replace the stereographic projection parameters with the
measured fisheye calibration before using the grid for navigation.
