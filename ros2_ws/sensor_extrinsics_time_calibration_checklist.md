# 实机传感器外参与时间校准清单

适用对象：
- `IMU`
- `RTK/GNSS`
- `3D LiDAR`

适用链路：
- `navsat / ESKF`
- `FAST-LIO`

## 1. 坐标系约定

统一采用车体坐标系 `base_link`：

- `x`：车头前方
- `y`：车体左侧
- `z`：车体上方

单位约定：

- 平移：`m`
- 角度：记录时可用 `deg`，落参数时转为程序要求单位

## 2. 空间外参清单

| 编号 | 外参名称 | 必须内容 | 作用模块 | 当前代码参数位置 | 备注 |
|---|---|---|---|---|---|
| 1 | `IMU -> base_link` | `x y z`，`roll pitch yaw` | `navsat / ESKF`、`FAST-LIO` | `SCOUT_IMU_XYZ`、`SCOUT_IMU_RPY` | 必须先确认 IMU 厂家轴定义 |
| 2 | `RTK天线 -> IMU` | `x y z` | `navsat / ESKF` | `antlever_m` | 量的是天线相位中心，不是外壳中心 |
| 3 | `LiDAR -> IMU` | `x y z`，旋转矩阵或 `roll pitch yaw` | `FAST-LIO` | `extrinsic_T`、`extrinsic_R` | 这是 `FAST-LIO` 最关键外参 |
| 4 | `LiDAR -> base_link` | `x y z`，`roll pitch yaw` | TF、可视化、点云落车体 | `laser_3d_xyz`、`laser_3d_rpy` | 应与 1、3 自洽 |

## 3. 时间外参清单

| 编号 | 时间关系 | 作用模块 | 当前代码参数位置 | 是否必须关注 | 备注 |
|---|---|---|---|---|---|
| 1 | `LiDAR <-> IMU` | `FAST-LIO` | `time_offset_lidar_to_imu` | 是 | 不同步会直接影响激光惯导 |
| 2 | `RTK <-> IMU` | `navsat / ESKF` | 当前无显式补偿参数 | 是 | 转弯、加减速时尤为敏感 |
| 3 | `RTK位置 <-> RTK航向` | `navsat / ESKF` | 当前依赖消息本身同步 | 建议关注 | 若位置和航向不同步，会造成航向更新异常 |

## 4. 当前代码中的对应位置

### 4.1 IMU 安装外参

文件：
- [src/scout_description/urdf/scout.urdf.xacro](src/scout_description/urdf/scout.urdf.xacro#L151)

当前默认值：

```text
SCOUT_IMU_XYZ = [0.19, 0, 0.149]
SCOUT_IMU_RPY = [0, -1.5708, 3.1416]
```

### 4.2 RTK 天线杆臂

文件：
- [src/agribot_rl_nav/config/navsat_kf_gins_map.yaml](src/agribot_rl_nav/config/navsat_kf_gins_map.yaml#L60)

当前值：

```yaml
antlever_m: [0.0, 0.0, 0.0]
```

### 4.3 LiDAR 到 IMU 外参

文件：
- [src/agribot_autonomy/config/fast_lio_sim_tuned.yaml](src/agribot_autonomy/config/fast_lio_sim_tuned.yaml#L36)

当前仿真值：

```yaml
extrinsic_T: [-0.19, 0.0, 0.429]
extrinsic_R:
  [1.0, 0.0, 0.0,
   0.0, 1.0, 0.0,
   0.0, 0.0, 1.0]
```

说明：
- `FAST-LIO` 官方定义：`extrinsic_T / extrinsic_R` 表示“LiDAR 在 IMU body frame 下的位姿”
- 实机不能直接照搬仿真值

### 4.4 LiDAR 到 base_link 外参

文件：
- [src/scout_description/urdf/scout.urdf.xacro](src/scout_description/urdf/scout.urdf.xacro#L287)

当前仿真入口参数：

```text
laser_3d_xyz
laser_3d_rpy
```

### 4.5 LiDAR 与 IMU 时间偏移

文件：
- [src/agribot_autonomy/config/fast_lio_sim_tuned.yaml](src/agribot_autonomy/config/fast_lio_sim_tuned.yaml#L16)

当前值：

```yaml
time_offset_lidar_to_imu: 0.0
```

## 5. 实机测量建议

### 5.1 IMU -> base_link

要做：
- 明确 `base_link` 原点位置
- 明确 IMU 原点位置
- 明确 IMU 正方向定义
- 量出 `x y z`
- 确认 `roll pitch yaw`

注意：
- 不要凭外壳方向猜 IMU 坐标轴
- 必须查 IMU 手册或驱动说明

### 5.2 RTK天线 -> IMU

要做：
- 找天线相位中心定义
- 量到 IMU 测量中心的 `x y z`

注意：
- 如果只能量到外壳中心，要记录这是近似值

### 5.3 LiDAR -> IMU

要做：
- 找 LiDAR 原点定义
- 明确 LiDAR 原始点云 frame 轴方向
- 量 LiDAR 原点到 IMU 原点的 `x y z`
- 确认两者之间旋转关系

注意：
- 不同 LiDAR 厂家 `x/y/z` 定义可能不同
- 这组外参错误会直接导致 `FAST-LIO` 漂移或歪斜

### 5.4 LiDAR <-> IMU 时间偏移

要做：
- 确认是否硬同步
- 确认是否共用同一时钟
- 确认时间戳是否来自设备时钟还是板端接收时刻

注意：
- 若不同步，静止时不明显，运动时误差会明显放大

### 5.5 RTK <-> IMU 时间偏移

要做：
- 检查 RTK 与 IMU 时间戳源是否一致
- 检查 RTK 位置和航向是否同一时刻

注意：
- 若不同步，常见现象是：
  - 直线还行，转弯误差明显
  - 航向滞后
  - 速度变化时位置抖动

## 6. 上车前优先级

### 第一优先级

- `LiDAR -> IMU`
- `RTK天线 -> IMU`
- `IMU -> base_link`

### 第二优先级

- `LiDAR <-> IMU` 时间偏移
- `RTK <-> IMU` 时间偏移

### 第三优先级

- `LiDAR -> base_link`

说明：
- 如果 1、3 已准确，`LiDAR -> base_link` 可以由它们间接推出
- 但为了 TF、可视化、排障方便，仍建议单独整理

## 7. 建议填写表

### 7.1 空间外参填写

| 项目 | x (m) | y (m) | z (m) | roll (deg) | pitch (deg) | yaw (deg) | 备注 |
|---|---:|---:|---:|---:|---:|---:|---|
| IMU -> base_link |  |  |  |  |  |  |  |
| LiDAR -> base_link |  |  |  |  |  |  |  |
| LiDAR -> IMU |  |  |  |  |  |  |  |

### 7.2 RTK 杆臂填写

| 项目 | x (m) | y (m) | z (m) | 备注 |
|---|---:|---:|---:|---|
| RTK天线 -> IMU |  |  |  |  |

### 7.3 时间外参填写

| 项目 | 偏移量 | 单位 | 备注 |
|---|---:|---|---|
| LiDAR -> IMU |  | s / ms |  |
| RTK -> IMU |  | s / ms |  |
| RTK位置 -> RTK航向 |  | s / ms |  |

## 8. 风险提示

- 仿真参数不能直接拿到实机
- 若坐标系方向定义错，后续所有调参都会白费
- 若时间偏移没处理，运动中误差会远大于静止时观测结果
- 若 `RTK天线 -> IMU` 杆臂没填，ESKF 在转弯场景下会出现系统偏差
