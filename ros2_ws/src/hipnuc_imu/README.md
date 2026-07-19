# hipnuc_imu

ROS 2 Humble driver for the N300Pro default HI91 stream (`115200`, `100 Hz`).
It validates every frame with CRC16, publishes SI units, and converts the vendor
RFU body frame to ROS FLU while keeping the vendor ENU world frame.

Topics:

- `/imu/data` (`sensor_msgs/Imu`), full orientation, angular velocity and acceleration
- `/imu/magnetic_field` (`sensor_msgs/MagneticField`)
- `/imu/temperature` (`sensor_msgs/Temperature`)

The installed udev rule is device-specific. Install it once and reload udev:

```bash
sudo cp install/hipnuc_imu/share/hipnuc_imu/udev/99-agribot-hipnuc.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
```

The user running ROS must be in `dialout`. A new login is required after
`sudo usermod -aG dialout "$USER"`.
