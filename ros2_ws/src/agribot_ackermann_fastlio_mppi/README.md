# Ackermann FAST-LIO MPPI

Physical-vehicle launch for C16 + N300Pro + FAST-LIO + Nav2 MPPI. It does not
start Gazebo or simulated sensor bridges.

```bash
ros2 launch agribot_ackermann_fastlio_mppi fastlio_mppi.launch.py
```

FAST-LIO consumes `/lidar/points` and `/imu/data`, and publishes full 6-DoF
odometry as `/fastlio/odometry`. The TF chain is `map -> odom -> base_link`.
The default `map -> odom` transform is identity and can be changed with launch
arguments when aligning an existing map.

The zero LiDAR-to-IMU and base-to-IMU transforms are calibration placeholders.
They must be replaced with measured values before evaluating localization.

`enable_chassis_output` defaults to `false`; MPPI and collision monitoring can
be observed without sending motion to the physical chassis. The chassis adapter
and emergency-stop path remain the final hardware-specific integration step.
