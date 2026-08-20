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
