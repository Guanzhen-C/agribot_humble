# 差速真车配置

该目录是差速车独立的真机运行边界，不复用阿克曼车的几何标定值作为正式参数。

## 算法与数据流

- 定位：FAST-LIVO2 局部里程计、固定解 RTK 因子图融合、NDT/GICP 初始重定位。
- 规划：Nav2 Smac State Lattice，使用 ROS 2 Humble 官方 5 cm 差速运动基元。
- 控制：Nav2 MPPI，运动模型为 `DiffDrive`，允许原地旋转和倒车。
- 避障：C16 `/lidar/points` 直接进入 STVL，不生成或订阅 `/scan`。
- 底盘：严格使用 `主控到分控数据格式V2.4.xlsx` 的“伽马底盘”协议，
  `0x514` 控制，`0x532/0x533/0x534` 反馈，默认 `250 kbit/s`，支持
  ZQWL USB-CAN 和 SocketCAN。

Nav2 输出路径为：

```text
/nav2/cmd_vel -> differential_chassis_can_node -> 0x514
```

`0x532` 的电池电压是 Byte2 单字节实际值；`0x533/0x534` 的速度是
16 位无符号幅值，方向由 Reverse 位给出，运行电流是 16 位有符号整数。
不存在旧实现中的通信故障字节、电机电压或电机温度字段。

底盘反馈生成 `/wheel/odometry` 和 `/scout_status`。导航定位使用
`/fastlivo_rtk/odometry`，轮速里程计只用于底盘状态和验收，不替代激光惯性定位。

## 坐标和标定

`base_link` 定义在四个轮毂中心组成矩形的几何中心，ROS 坐标为 `+X` 向前、
`+Y` 向左、`+Z` 向上。现有临时尺寸由阿克曼车参数重新居中得到，不是真实差速车标定值。
以下文件必须以同一套实测参数更新：

- `config/vehicle_calibration.yaml`：车体前后左右尺寸、安全余量、轮距、轮径和速度上限。
- `config/sensor_mounts.yaml`：IMU、C16、相机和 RTK 主天线相对 `base_link` 的外参。
- `config/fastlio_bridge.yaml`、`config/fastlivo_bridge.yaml`：IMU/body 外参。
- `config/pcd_initial_localization.yaml`：C16 外参。
- `config/rtk_map_initializer.yaml`、`config/fastlivo_rtk_fusion.yaml`：RTK 杆臂和 IMU 姿态外参。
- `config/chassis_can.yaml`：左右控制方向、百分比到轮组速度的实测比例、底盘限速及通信安全参数。

仓库内现有差速数值只是结构占位值，`calibration_complete: false`。完整启动文件默认
`enable_chassis_output:=false`；即使手动打开输出，标定未完成或缺少显式授权字符串时也会拒绝创建底盘节点。

## 验证顺序

1. 只验收传感器和 TF：

```bash
ros2 launch agribot_hardware_bringup differential_sensor_validation.launch.py \
  rviz:=true
```

2. 使用 FAST-LIO 建三维 PCD 和二维 Nav2 地图，始终不创建底盘节点：

```bash
ros2 launch agribot_hardware_bringup differential_3d_mapping.launch.py \
  map_base:=/absolute/path/to/map_name \
  rviz:=true record_bag:=false
```

3. 不连接传感器和底盘，单独验证 State Lattice 路径：

```bash
ros2 launch agribot_hardware_bringup \
  differential_state_lattice_validation.launch.py \
  map:=/absolute/path/to/map_name.yaml rviz:=true
```

4. 使用已建地图运行定位、规划、控制和避障，但保持底盘输出关闭：

```bash
ros2 launch agribot_hardware_bringup \
  differential_mppi_fastlivo_rtk_mapped.launch.py \
  map_base:=/absolute/path/to/map_name \
  initialization_source:=manual \
  enable_chassis_output:=false rviz:=true
```

室外有有效地理配准时，使用 `differential_outdoor_experiment.launch.py`，它固定采用
RTK 粗定位和 NDT/GICP 精配准。

完成所有实测、架空轮测试、急停测试和反馈验收后，将标定文件中的
`calibration_complete` 改为 `true`，再显式授权运动：

```bash
ros2 launch agribot_hardware_bringup \
  differential_mppi_fastlivo_rtk_mapped.launch.py \
  map_base:=/absolute/path/to/map_name \
  enable_chassis_output:=true \
  motion_authorization:=ENABLE_DIFFERENTIAL_MOTION \
  can_transport:=zqwl_cdc \
  zqwl_port:=/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

任何反馈超时、定位就绪超时、急停、非自动模式、底盘故障或命令超时都会触发零速制动。
