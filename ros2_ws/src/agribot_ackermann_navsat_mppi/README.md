# Ackermann NavSat MPPI

Physical-vehicle launch for N300Pro IMU + RTK + KF-GINS + Nav2 MPPI. It does
not start Gazebo or any simulated sensor bridge.

```bash
ros2 launch agribot_ackermann_navsat_mppi navsat_mppi.launch.py
```

Required live interfaces are `/imu/data`, `/rtk/fix`, `/scan`, and the C16
point cloud. KF-GINS publishes the full 6-DoF pose on
`/odometry/filtered_navsat`; the TF chain is `map -> odom -> base_link`.

`enable_chassis_output` defaults to `false`. With that setting MPPI can be
observed on `/nav2/cmd_vel_safe`, but no command reaches `/cmd_vel`. Enable it
only after the physical chassis adapter, steering convention, emergency stop,
wheelbase, footprint, turning radius, and sensor extrinsics have been verified.
