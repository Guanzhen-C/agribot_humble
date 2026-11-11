# 动力换挡与 CVT 硬件在环操作说明

本扩展在 Car4 拖拉机、三点悬挂及圆盘耙台架上增加动力传动控制。X5 计算导航速度需求，HPM6750 执行换挡、调速及离合器控制，PC Gazebo 计算等效发动机、液压离合器、传动系和车辆的物理响应。保留原有悬挂控制和作业效果，不修改阿克曼车、差速履带车的真机 CAN 协议。

这里验证的是控制器硬件与实际通信链路。发动机、变速箱、离合器及车辆均为等效仿真模型，不能用其曲线宣称某一实际拖拉机的换挡品质、油耗、寿命、牵引力或安全认证结果。

## 控制链路

```text
PC 仿真 GNSS / IMU / 雷达
           | 网口 ROS 2
           v
X5 NavSat ESKF + Smac Hybrid-A* + MPPI
           | 高层模式、车速需求；真实 USB-CAN
           v
HPM6750 CAN1：ISOBUS 地址声明及对端 NAME 校验
           | 20 ms 传动控制与悬挂控制
HPM6750 CAN0：阀量、油门、离合器、速比
           | 真实 USB-CAN
           v
PC Gazebo：轴转矩 -> 实际轮胎运动 -> 发动机/压力/车速反馈
           | 原生 CAN 反馈 -> 6750 -> ISOBUS -> X5
           v
只读参数曲线窗口与原始报文记录
```

上游同一条经典 CAN 同时承载本隔离台架的行走需求和 ISOBUS 扩展帧，速率为 250 kbit/s。ROS Domain 为 74，Gazebo master 为 11358。6750 上的传动控制与悬挂控制独立维护指令租约与故障状态。

`0x620` 在新传动场景中只表达车速及前轮转向需求。旧后轮速度电机的 `wheel_torque` 必须为 0；车辆前进由新插件 `libagribot_hil_powertrain_plant.so` 对后轮施加转矩实现。不得直接发布 `/hil_hitch/actuator/*` 来替代真实 CAN 链路。

## 模式和参数

| 模式名称 | 数值 | 操作与含义 |
| --- | --- | --- |
| `off` | 0 | 空挡、分离离合器并制动；用于启动和故障后的显式恢复 |
| `powershift_manual` | 1 | 人工选择 1 至 4 挡，控制器逐级交接离合器 |
| `powershift_auto` | 2 | 根据平滑车速需求自动选择低速作业挡位 |
| `cvt_auto` | 3 | 根据转速与车速需求连续调整总减速比 |
| `cvt_manual` | 4 | 人工设定总减速比，控制器限制速比变化速率 |

| 可下发参数 | 范围及单位 | 本次默认值 |
| --- | --- | --- |
| 车速需求 | 0 至 0.5 m/s，仅前进 | 由 MPPI 输出 |
| 发动机转速请求 | 800 至 2200 rpm，20 rpm 分辨率 | 1600 rpm |
| 手动动力换挡挡位 | 1 至 4 | 自动模式中填 0 |
| 手动 CVT 总减速比 | 200 至 900，0.1 分辨率 | 自动模式中填 0 |
| 换挡交接时间 | 200 至 2000 ms，10 ms 分辨率 | 600 ms |
| CVT 速比变化率 | 10 至 300 比值单位/s | 150/s |

总减速比包含低速作业传动比，不是单独变速箱齿轮对的速比。四个等效作业挡的总减速比分别为 640、480、350、260。自动挡的速度分界为 0.17、0.28、0.40 m/s，连续升降挡至少间隔 700 ms，每次只跨一个相邻挡。交接分为充油和扭矩交接阶段，双离合器压力随阶段变化，不是简单修改显示的挡位数字。

自动 CVT 以转速、轮周长和车速参考计算速比，并增加有界反馈修正。动力换挡模式的转速目标还会根据挡位和车速需求降低到不低于 800 rpm；因此转速请求不是每种运行条件下强制保持的实际转速。车速参考的变化率为 0.4 m/s²。

以下为模型参数而非已标定真机参数：离合器最大压力 2 MPa、液压响应时间常数 0.12 s、CVT 执行机构最大变化率 220/s、传动效率 0.86、总后轮转矩限幅 1750 N·m、基础滚动阻力 180 N。额外牵引负载可通过 `/hil_hitch/powertrain/load_force` 注入 0 至 3000 N；这是受控扰动，不是农具真实入土阻力模型。

## 构建和准备

沿用 README 中的 Ubuntu 22.04、ROS 2 Humble、Gazebo Classic 11、原始 Car4/圆盘耙网格及宽地头场景依赖。不要更改系统发行版。首次准备可使用：

```bash
HITCH_HIL_STATE="$HOME/.local/state/agribot-powertrain-hil-new" \
  bash src/agribot_hil_hitch/scripts/build.sh --powertrain --prepare --firmware
```

输出目录存在时不覆盖。若复用已转换网格，单独调用 `prepare_scene.py --powertrain`，然后 `prepare_runtime.py`，两者的完整参数参见 `--help`。部署到 X5 的 `scenario/` 必须来自当前生成场景的 `rdk/`，不能混用旧车型参数。

本次已验证目录：

```text
PC ~/.local/state/agribot-powertrain-hil-20261111/scenario
PC ~/.local/state/agribot-powertrain-hil-20261111/install
X5 ~/agribot_hitch_hil_ws/scenario
X5 ~/agribot_hitch_hil_ws/build-protocol/hil_hitch_endpoint
```

新功能需要带 `powertrain.c` 的 6750 专用固件及新 X5 协议端点。烧录前撤销授权、停止两端台架，备份原固件和板上镜像，再编程并校验 ELF。不要把这个专用固件用于两台真车。

## 启动与作业

先按 README 准备两端 CAN 与网口 DDS。下列终端需保持运行，日志目录不能已存在：

```bash
# PC
export HITCH_HIL_STATE="$HOME/.local/state/agribot-powertrain-hil-20261111"
export HITCH_SCENE_ROOT="$HITCH_HIL_STATE/scenario"
bash src/agribot_hil_hitch/scripts/run_hil.sh pc can_trace:=/new/path/pc.jsonl

# X5
bash ~/agribot_hitch_hil_ws/src/agribot_hil_hitch/scripts/run_hil.sh rdk \
  powertrain_enabled:=true can_trace:=/new/path/usb.jsonl client_trace:=/new/path/client.jsonl

# PC，另一个终端，继承上面的 HITCH_HIL_STATE
bash src/agribot_hil_hitch/scripts/run_hil.sh pc monitor --record /new/path/curves.csv
```

启动后默认未授权，不自动行走。确认仿真和实体反馈均就绪后，在 X5 另一个终端选择一种传动模式执行五阶段任务：

```bash
bash ~/agribot_hitch_hil_ws/src/agribot_hil_hitch/scripts/run_hil.sh rdk mission \
  --transmission powershift_auto --start --output ~/agribot_hitch_hil_ws/mission-new
```

使用 `--transmission cvt_auto` 可选择 CVT。切换前完成任务、撤销授权并确认停车；从原起点重新比较时重启仿真及 X5 定位节点。任务和检查程序不能同时运行。

任务仍为抬耙接近、降耙第一条作业行、抬耙地头转弯、降耙第二条作业行、抬耙停车。路径由 X5 Smac 规划，MPPI 跟踪。任务检查 X5 的传动开关和 PC 场景是否匹配，拒绝在旧速度驱动场景中执行新传动任务。

原悬挂速度驱动台架仍使用不带 `--powertrain` 的场景、`powertrain_enabled:=false` 和 `--transmission off`。新旧场景不可只改一个参数混用。

## 参数监视与人工试验

曲线窗口只读取 X5 经实际 ISOBUS 收到的数据，显示车速、发动机与输入轴转速、总减速比、离合器压力、挡位、换挡阶段、滑差、负载及制动状态。其反馈采用 20 Hz 最新值快照，不允许监视窗口阻塞控制循环；原始 CAN 日志仍逐帧保存。

人工台架指令话题是 `/hil_hitch/powertrain/command`，消息为 `std_msgs/String`，JSON 必须完整包含以下字段，每 50 ms 左右刷新。仍需显式授权和新鲜的高层车速需求；单独发送这个消息不自动驱动车辆。

```json
{"mode":"cvt_manual","gear":0,"engine_rpm":1600,"ratio":450,"shift_ms":600,"ratio_rate":150}
```

非手动 CVT 模式 `ratio` 必须为 0，非手动动力换挡模式 `gear` 必须为 0。换挡交接时间及 CVT 变化率只允许停车时修改；持续重发相同设置不受此限制。行驶中改变动力换挡/CVT 模式被拒绝，车速需求保持在同一模式内时可正常升降挡或连续调速。

停止任务时取消导航、发送空挡并撤销 `/hil_hitch_rdk_client` 的 `drive_authorized` 和 `work_authorized`。如果任务为临时服务，可先撤销授权，再停止对应 mission 服务，最后停止 PC/X5 预览。关闭 Gazebo 窗口不替代此流程。

## 故障与恢复

- X5 高层传动命令超过 250 ms 未刷新时请求空挡；高层车速超过 200 ms、定位超过 800 ms 未更新时禁止行走。
- 6750 传动命令有效期 500 ms、车速需求有效期 250 ms、传感组合反馈有效期 200 ms。
- 三组关键传感帧需要相同计数器，组包跨度不超过 100 ms；CRC 错误、重复和倒序不刷新租约。
- 传感故障、网络失联或实际速度超过 0.65 m/s 时分离离合器、撤销驱动并制动。
- 故障恢复需要显式空挡和新指令，不恢复缓存的旧非零动作。PC 执行侧还有独立 200 ms 配对报文租约。
- 原始日志中的拒绝应答须与人工非法切换/失联注入区分；不能直接删除失败记录或关闭保护来得到通过结果。

这些是实验台架保护，不是经认证的急停或功能安全系统。

## 验证与证据

```bash
# 协议、控制器及模型单元测试
/usr/bin/ctest --test-dir "$HITCH_HIL_STATE/build-protocol" --output-on-failure
HITCH_TEST_SCENE="$HITCH_SCENE_ROOT" /usr/bin/python3 -m pytest -q src/agribot_hil_hitch/test

# 两端实体台架已运行、没有导航任务时，PC 发起检查
bash src/agribot_hil_hitch/scripts/run_hil.sh pc verify-powertrain \
  --output /new/path/powertrain-check
```

检查涵盖动力换挡一挡、逐级升降挡、行驶模式切换拒绝、自动 CVT、连续速比变化、600 N 负载扰动、手动 CVT、超时制动、显式恢复及悬挂联动，并验证停车设置更新与 CVT 变化率生效。可选 `--native-bridge-pid` 会暂挂指定的本台架 PC 原生桥，用于验证真实 6750 的传感超时保护：分别检查0.5秒内撤销驱动并制动，以及之后5秒内车体停稳，再恢复通信。两种耗时分别记录，不把执行指令归零视为车体瞬时停稳；整个检查保留等效地面和制动物理响应。只能填当前本隔离台架 `pc_bridge.py` 的 PID，不用于导航运行时。

每次输出 `result.json`、Gazebo 实际状态 `samples.jsonl`、高层发送记录 `commands.jsonl`。实体测试须同时保存 X5 的 USB/客户端和 PC 的原生 CAN 日志。软件在环仅使用主机端点及虚拟 CAN，不得标为实体硬件测试。详细字段见 [传动协议](POWERTRAIN_PROTOCOL.md)，最终实体结果见 [测试记录](POWERTRAIN_TEST_REPORT.md)。
