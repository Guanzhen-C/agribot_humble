# Hikrobot MVS ROS 2 driver

用于海康机器人 `MV-CU013-A0UC` USB3 工业相机。驱动直接调用 MVS SDK，SDK缓存固定为少量最新帧，避免视觉算法处理不及时导致内存持续增长。

## 依赖

MVS SDK属于海康机器人专有软件，不放入Git仓库。把对应架构的官方SDK压缩包放到设备后执行：

```bash
src/hikrobot_mvs_ros2/scripts/install_mvs_usb_sdk.sh /path/to/mvs-sdk-aarch64.tar.gz
```

该脚本只安装SDK、动态库路径和USB权限，不安装或启用厂商网络服务。

## 无镜头取流检查

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch hikrobot_mvs_ros2 mv_cu013_a0uc.launch.py trigger_enable:=false
ros2 topic hz /camera/rgb/image_raw
```

无镜头时只能验证连接和取流，不能标定或评价画面质量。安装并固定镜头后，必须重新标定相机内参、相机到雷达的外参和图像时间偏移，不能复用旧深度相机参数。

驱动默认使用相机的64位设备计数器。当前 `MV-CU013-A0UC` 已实测为
`100 MHz`，驱动将其在线映射到MVS SDK记录的RDK接收时间；`/diagnostics`
中的 `hikrobot_mvs/time_sync` 会报告预热、时钟漂移、接收抖动和复位次数。
`timestamp_offset_sec` 保持为零，曝光相对雷达的固定偏移统一由后续
FAST-LIVO2 `img_time_offset` 标定，避免两处重复补偿。

## 镜头安装后的收尾顺序

1. 固定镜头、锁紧光圈和焦距，在车辆主要工作距离内确认整幅画面清晰。
2. 保持 `1280x1024` 原始分辨率完成单目标定。现有棋盘若仍为 `11x8` 个内角点、方格边长 `10 mm`，可执行：

   ```bash
   ros2 run camera_calibration cameracalibrator \
     --size 11x8 --square 0.010 --no-service-check \
     image:=/camera/rgb/image_raw camera:=/camera/rgb
   ```

3. 将真实 `fx/fy/cx/cy/d0..d3` 写入 `agribot_hardware_bringup/config/fastlivo_hikrobot_mv_cu013.yaml`。
4. 采集包含平移和三轴转动的数据，用 FAST-Calib 求相机到 C16 的旋转、平移和图像时间偏移；分别更新阿克曼或差速车的 FAST-LIVO2 外参配置。
5. 只有重投影、点云着色和时间对齐验收通过后，才把对应车型 `hikrobot_camera_calibration_status.yaml` 的四个状态改为 `true`。

完整 FAST-LIVO2 启动入口会检查这些状态。未标定时会明确退出，但相机单独取流和原始数据采集仍可运行。
