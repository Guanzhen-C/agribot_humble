# Livox ROS Driver 2 for ROS 2 Humble

This package provides the Livox ROS Driver 2 node and custom messages for this
ROS 2 Humble workspace. It supports HAP, MID-360, MID-360s, and mixed HAP / MID-360
configurations.

The package builds the pinned Livox-SDK2 v1.3.1 source under `3rdparty/` as part
of the colcon build. Do not install a second SDK copy under `/usr/local`.

## Build

From the workspace root:

```bash
./build_ros2_humble.sh --packages-up-to livox_ros_driver2
source install/setup.bash
```

## Configure the network

Edit the JSON file for the connected device before starting the driver:

- `config/HAP_config.json`
- `config/MID360_config.json`
- `config/MID360s_config.json`
- `config/mixed_HAP_MID360_config.json`

Set each LiDAR IP address, the host IP address, and the command, point-cloud,
IMU, and log ports to match the physical network. The host interface must have
an address on the same subnet.

## Run

Message-only launch files:

```bash
ros2 launch livox_ros_driver2 msg_HAP_launch.py
ros2 launch livox_ros_driver2 msg_MID360_launch.py
ros2 launch livox_ros_driver2 msg_MID360s_launch.py
```

RViz launch files:

```bash
ros2 launch livox_ros_driver2 rviz_HAP_launch.py
ros2 launch livox_ros_driver2 rviz_MID360_launch.py
ros2 launch livox_ros_driver2 rviz_MID360s_launch.py
ros2 launch livox_ros_driver2 rviz_mixed.py
```

The driver publishes shared topics by default:

- `/livox/lidar`: `sensor_msgs/msg/PointCloud2` when `xfer_format` is `0`, or
  `livox_ros_driver2/msg/CustomMsg` when it is `1`
- `/livox/imu`: `sensor_msgs/msg/Imu`

Set `multi_topic` to `1` in a launch file to publish separate topics for each
connected LiDAR. `publish_freq` accepts values from 0.5 Hz through 100 Hz.

## Point-cloud formats

- `xfer_format: 0`: Livox `PointCloud2` fields (`x`, `y`, `z`, `intensity`,
  `tag`, `line`, and per-point `timestamp`)
- `xfer_format: 1`: `livox_ros_driver2/msg/CustomMsg`
The driver publishes live device data to ROS 2 topics. Record those topics with
the standard Humble command when needed:

```bash
ros2 bag record /livox/lidar /livox/imu
```
