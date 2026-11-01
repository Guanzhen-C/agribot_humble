# 差速真车配置

该目录是差速车独立的真机运行边界，不复用阿克曼车的几何标定值作为正式参数。

## 算法与数据流

- 定位：FAST-LIVO2 局部里程计、固定解 RTK 因子图融合、NDT/GICP 初始重定位。
- 规划：Nav2 Smac State Lattice，使用 ROS 2 Humble 官方 5 cm 差速运动基元。
- 控制：Nav2 MPPI，运动模型为 `DiffDrive`，允许原地旋转和倒车。
- 避障：C16 `/lidar/points` 直接进入 STVL，不生成或订阅 `/scan`。
- 底盘：严格使用 2026-08-26 更新的 `三合一协议.xlsx` 第 2-50 行，
  `0x514` 控制，`0x532/0x533/0x534` 反馈。实车 CAN 波特率为
  `250 kbit/s`，支持 ZQWL USB-CAN 和 SocketCAN。

Nav2 输出路径为：

```text
/nav2/cmd_vel -> differential_chassis_can_node -> 0x514
```

`0x514` 的 Byte1/Byte2 是左右电机有符号 PWM 百分比，Byte3 bit0 是大灯；
该协议没有独立刹车位，安全停止使用左右 PWM 同时归零。`0x532` 的电池电压
按 Byte2-3 Intel 无符号值和 `0.1 V` 分辨率解析；Byte4 bit0-bit3 分别是遥控器、
无人化、电机驱动和 BMS 通信故障。`0x533/0x534` 的 Byte0 bit0-bit7 分别是
OVP、UVP、温度、OCP、过载、霍尔、堵转和其他故障，Byte1-2 是 Intel 有符号
电机转速，Byte3 是电机电压（`0x4b` 即 `75 V`），Byte4 是有符号电流，
Byte5 减 40 后是温度。所有帧使用 Byte6 低4位计数和 Byte7 XOR；实车反馈
计数器会在相邻发送帧之间继续推进，因此驱动拒绝重复帧但不强制相邻同 ID 的
计数值逐次加一。

同一 Excel 后半部分的作业设备、遥控器和 BMS 帧属于各自设备节点，本导航底盘
节点不会冒充这些设备发送报文。

底盘反馈生成 `/wheel/odometry` 和 `/scout_status`。导航定位使用
`/fastlivo_rtk/odometry`，轮速里程计只用于底盘状态和验收，不替代激光惯性定位。

## 坐标和标定

`base_link` 定义在左右履带中心线之间、履带接地长度纵向中心的地面投影处，
ROS 坐标为 `+X` 向前、`+Y` 向左、`+Z` 向上。2026-08-26 已录入以下实测值：

- 车体前后各 `0.60 m`、左右各 `0.38 m`，履带几何中心距 `0.60 m`。
- IMU `(0, 0, 0.64) m`，外壳轴与车辆 FLU 对齐。
- C16 `(0.47, 0, 0.91) m`，出线口对应扫描 `0°`/点云 `+Y` 并朝车后，
  因此点云 `+X` 朝车左，`base_link -> lidar_link` 偏航为 `pi/2`。
  `sensor_mounts.yaml` 中的 `lidar.pps_target_angle_deg: 180.0` 会在启动雷达驱动前
  自动核对：一致时不写入，不一致时备份设备包、修改目标角并回读确认。
  只调整 PPS 锁定角，不修改 gPTP 时钟源、网络配置或点云首点时间戳；
  时钟源不是 PTP、端口被占用或回读失败会明确报错。纯回放和禁用雷达时跳过。
- 相机光心 `(0.4735, 0, 0.96) m`，镜头朝前，图像正立。
- RTK 天线位置于 2026-10-28 更新：相对 IMU 的左主天线坐标为
  `(0.44, 0.32, 0.20) m`，右副天线为 `(0.44, -0.32, 0.20) m`，基线长 `0.64 m`。
  由于 IMU 在 `base_link` 下为 `(0, 0, 0.64) m` 且轴向一致，两天线的
  `base_link` 坐标分别为 `(0.44, 0.32, 0.84)`、`(0.44, -0.32, 0.84) m`；
  主副天线方向未变，接收机航向补偿仍为 `-90°`。同步更新定位、初始化、
  离线建图杆臂及天线过滤位置。历史数据需使用采集时的外参快照，不能套用新外参。

以下文件由同一套实测参数约束：

- `config/vehicle_calibration.yaml`：车体前后左右尺寸、安全余量、轮距、轮径和速度上限。
- `config/sensor_mounts.yaml`：IMU、C16、相机和 RTK 主天线相对 `base_link` 的外参。
- `config/fastlio_bridge.yaml`、`config/fastlivo_bridge.yaml`：IMU/body 外参。
- `config/pcd_initial_localization.yaml`：C16 外参。
- `config/rtk_map_initializer.yaml`、`config/fastlivo_rtk_fusion.yaml`：RTK 杆臂和 IMU 姿态外参。
- `config/chassis_can.yaml`：左右控制方向、百分比到轮组速度的实测比例、底盘限速及通信安全参数。

平移和名义轴向已不再使用阿克曼占位值，当前雷达-相机手工外参、履带轮距及速度
控制曲线已被接受用于低速调试运行，后续仍可通过靶标和运动实验继续精修。因此
`calibration_complete: true`，但完整启动文件仍默认 `enable_chassis_output:=false`；
缺少显式授权字符串时仍会拒绝启用底盘控制输出。

## 2026-10-30 速度参数候选

提速前代码保存在Git标签`20261030`。单套MPPI参数调整为：最大线速度0.50 m/s，
最大角速度0.90 rad/s，角加速度上限0.50 rad/s²；底盘对应限速同步，单侧履带
命令上限1600（实测0.78 m/s）。以0.60 m履带中心距计算，最大组合指令的外侧
履带速度0.77 m/s，不触发履带速度缩放。1600是履带上限，不是整车目标占空比。

角速度采样0.08、线速度采样0.12、路径代价权重、5 Hz控制、反向限速0.15 m/s、
安全轮廓与恢复行为速度保持原值；不增加直行/地头状态切换。桌面GUI默认800不变。
这是待真车验证的候选参数，配置一致性测试不等于直线精度和制动距离已验证。

保存路线会校验完整导航配置文件的指纹。上述参数变更会使旧路线的配置绑定失效，
部署时须重新验证并更新绑定；不能直接跳过校验，也不改变原有104个经停点。

## CAN互斥与只读反馈

桌面差速车GUI和农机控制台运行栈互斥使用同一USB-CAN。使用控制台前先退出
桌面GUI；使用桌面GUI前先在控制台停止运行栈，不允许同时抢占串口。

完整导航入口默认 `enable_chassis_feedback:=true`。观察阶段虽然关闭底盘输出，
仍启动CAN节点的 `read_only` 模式，接收并发布 `/scout_status` 和底盘诊断。
只读模式不订阅速度、不发送CAN控制帧，退出时也不发送CAN停止帧；连接适配器
时仍需发送USB适配器自身的通道配置包，这不是向底盘发送运动指令。
底盘反馈不依赖6750诊断串口，控制台可显示电压、故障状态和左右电机RPM。

如果GUI尚未退出，接收节点只等待适配器并报告占用原因，GUI关闭后自动重试。
没有实际反馈、三条反馈不齐、诊断停止或消息超时，不显示“反馈有效”。
底盘的无人化通信故障可能在纯监听时存在，因为只读模式不会代发控制心跳；
只有显式授权的真车阶段才启动原有CAN控制发送逻辑。

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

当前粗标定被接受后，仍须显式授权运动：

```bash
ros2 launch agribot_hardware_bringup \
  differential_mppi_fastlivo_rtk_mapped.launch.py \
  map_base:=/absolute/path/to/map_name \
  enable_chassis_output:=true \
  motion_authorization:=ENABLE_DIFFERENTIAL_MOTION \
  can_transport:=zqwl_cdc \
  zqwl_port:=/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

任何反馈超时、定位就绪超时、急停、非自动模式、底盘故障或命令超时都会触发左右 PWM 归零。
