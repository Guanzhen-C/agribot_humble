# Agribot hardware bringup

This package normalizes the physical sensor interfaces used by both navigation
stacks:

- Leishen C16: `/lidar/points` at about 10 Hz and `/scan`
- N300Pro: `/imu/data` at 100 Hz
- RTK receiver: `/rtk/fix` (`sensor_msgs/NavSatFix`)

Configure the dedicated LiDAR Ethernet link after each boot:

```bash
sudo ros2 run agribot_hardware_bringup configure_c16_network.sh eno1
```

Then start the connected sensors:

```bash
ros2 launch agribot_hardware_bringup sensors.launch.py rviz:=true
```

RTK is disabled by default until it is connected. Enable it with
`start_rtk:=true`. For an external NTRIP caster, keep credentials outside Git:

```bash
export NTRIP_HOST=example.invalid
export NTRIP_MOUNTPOINT=mountpoint
export NTRIP_USERNAME=user
export NTRIP_PASSWORD=password
ros2 launch agribot_hardware_bringup sensors.launch.py \
  start_rtk:=true enable_ntrip:=true
```

Before vehicle motion, replace the zero placeholders in `sensor_mounts.yaml`
with measured `base_link` to sensor transforms. The C16-to-IMU transform in the
FAST-LIO package must describe the same physical installation.
