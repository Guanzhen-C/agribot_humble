# Waypoint Localization Handover

差速车室外流程使用两段定位，规划器、控制器、车体参数和传感器驱动不变：

1. 起始使用已有GNSS/IMU ESKF（KF-GINS机械编排），只接收质量4的位置观测。
2. 保存路线进度达到第100个人工经停点后，异步启动FAST-LIVO2＋RTK。
3. 预热期间继续使用NavSat输出，不取消FollowPath，不强制停车。
4. 新定位XY方差均不超过1.0平方米，与同一时刻NavSat位姿的位置差不超过1.0米、三维姿态差不超过20度，
   且健康重叠持续1秒后接管；之后退出NavSat估计器进程组。
5. 接管第一帧与前一帧公共位姿一致，剩余全局修正按最多0.05米/秒、2度/秒逐步释放。

重叠校验以每一帧融合定位的测量时刻查询NavSat历史，不以两次消息间的等待时间
反复清零稳定窗口。两帧间隔/流延迟超过1秒仍重置窗口；接管瞬间的新定位仍须
在0.5秒以内，且保留位姿、协方差和进程健康检查。没有新观测不能靠计时器凑满窗口。
预热FAST-LIVO2子进程使用nice 5和单线程BLAS，优先保留正在工作的NavSat/控制链路资源。

切换点由保存路线的 localization_handover_waypoint 指定：原路线100、第二路线126；
第三路线显式设为null，全程KF-GINS，不启动融合定位。切换点只发异步预热请求，不发送停车指令、不取消或重新下发FollowPath。
后台启动尚未完成时，NavSat公共输出及ready保持由其自身健康状态决定；
接管后仅使用FAST-LIVO2/RTK自身健康状态，不因RTK退出固定解而单独停车。
所有协方差检查返回原生Python布尔值，ROS Bool消息边界再次转换，避免NumPy
布尔类型导致切换节点退出。回归测试覆盖慢启动期间连续输出、接管后的持续就绪
和RTK消失、以及协方差异常后恢复；真正的定位中断或不健康仍会暂停保留路线。

### Indoor Runtime Health

接管前的0.25平方米XY方差上限只用于NavSat短时惯导维持及新定位接管校验，
不继续用作FAST-LIVO2/RTK接管后的运行门限。GNSS消失后，因子图的全局位置边缘
协方差可以增长，这与局部激光视觉里程计是否持续工作不是同一个指标。
接管后仍完整发布真实协方差，不清零、不截断、不换成固定小数值。

接管后的公共ready依据后端进程、1秒就绪心跳、4秒接收/测量新鲜度，以及数据
合法性判断。位姿和速度须为有限值，协方差须为有限、对称、半正定且对角线非负。
新融合位姿相对上一有效测量的3D位移不得超过`0.20 + 1.5 * dt`米，姿态变化不得
超过`radians(10) + 2.0 * dt`弧度（dt以秒计、最多4秒）。这是容纳履带速度、已有全局
修正速度和估计抖动的突变检查，不改变MPPI或底盘限速，也不保证检测缓慢漂移。
非法帧不覆盖上一有效位姿，并即时撤销ready；后续一致的有效数据可恢复。
连续重复的跳变帧不会因等待时间增长而被逐步放行；长时间断流后的大幅重定位
不能靠累积等待时间绕过突变检查，应重新初始化。

因此，RTK退出质量4或全局协方差增大本身不会暂停已接管的定位；后端断流、
异常帧或明显跳变仍会暂停并保留路线。故障状态明确发布`wait_localization`和
具体原因，不再在融合模式下误报`wait_rtk`或仅显示“FAST-LIVO2/RTK active”。

离线回归包含3 Hz、十分钟无RTK、XY方差逐步增长至现场观测的6.04/2.33平方米，
以及断流、后端失效、非法矩阵、位姿跳变及恢复。以下独立ROS链路测试使用合成
消息、独立域和替代估计器进程，不加载传感器、Nav2或底盘，不能代替真车验收：

```bash
source /opt/ros/humble/setup.bash
source install/setup.bash
export PYTHONPATH="$PWD/src/agribot_localization_switch:${PYTHONPATH:-}"
ROS_DOMAIN_ID=89 ROS_LOCALHOST_ONLY=1 python3 \
  src/agribot_localization_switch/tools/validate_indoor_handover.py \
  --duration 70 --output /tmp/indoor_handover_validation.json
```

这是控制流程和坐标连续性保护，不是实际定位精度或真车全流程验收结论。
过渡时会短时同时运行两套估计器，不能等同于长期双运行。未就绪时保持NavSat；
若NavSat也不健康，公共ready撤销，底盘仍会停车，保存路径任务等待恢复。

### Short GNSS Interruptions

首次就绪仍要求新鲜固定解4、有效航向和健康INS输出。初始化后，一帧浮点解、
残缺报文或暂时缺帧只会丢弃该GNSS观测，不立即撤销导航就绪或报错；
KF-GINS继续处理真实IMU样本，正常固定解回来后继续位置更新，不重启估计器。
非固定解、重复/过期时间戳、非法坐标或协方差不能刷新最后有效固定解时刻。

短时惯导维持窗口默认最多4秒（节点参数`navsat_gnss_holdover_sec`，允许0.5至4秒），
同时要求INS输出接收/测量时延不超过0.5秒、XY方差均不大于0.25平方米。
不等待4秒才检查协方差或IMU中断；任一安全条件先失败仍暂停并保留路线。
该策略不是无期限纯惯导，也不能恢复USB链路已丢失的报文内容。
`fixed_active`仍只表示当前新鲜质量4，惯导维持期不能假装仍有固定解。

## Interfaces

- 最终定位保持`/fastlivo_rtk/odometry`、`/fastlivo_rtk/ready`，兼容现有控制台和Nav2。
- `/localization/active_method`：`navsat`、`navsat_warming_fastlivo`、`fastlivo_rtk`。
- `/localization/start`：可选按钮初始化。
- `/localization/switch_to_fastlivo`：幂等的异步预热请求；不是立即接管命令。
- 仅切换管理器发布`map -> odom -> base_link`；两套估计器输出位于私有话题且不发布此TF。

NavSat使用地图已有地理配准，校验PCD指纹，不以第一次GNSS观测重新定义地图原点。
使用差速车当前外参：IMU相对base为(0,0,0.64)米，主天线相对IMU为(0.44,0.32,0.20)米。
GNSS观测按设备测量时间处理，为接收延迟保留0.25秒IMU缓冲。
管理器使用带速度和角速度的惯导状态，最多前推0.35秒以对齐时间，禁止直接给旧位姿换新时间戳。
新FAST-LIVO2里程计通过同时刻NavSat状态提供地图初始化先验，随后由原RTK固定解因子约束。

## Route Semantics

第100点是`anchor_path_indices[99]`，不是第100个稠密采样点。
从第51点开始仍按原编号100触发；从100或之后起步则立即请求预热。
少于100点的路线和手动目标不会自动触发，保持NavSat。
触发以有序路径进度为准，不用全局最近点或纯距离判定，避免相邻田垄和返程轨迹误触发。
一次室外运行栈内只切换一次，不因RTK质量4/5反复切换。结束整个运行栈后重置为NavSat。

## Runtime

移动控制台的差速室外profile使用本包`outdoor_waypoint.launch.py`。
阿克曼与差速室内profile不变。原`differential_outdoor_experiment.launch.py`直接运行仍使用原定位，
只新增了可选`localization_launch_file`扩展接口，其默认值保持不变。
所有子进程均是无传感器驱动、无底盘节点的估计器launch，统一按进程组退出，
停止运行栈时会同时清理预热进程。没有复制新的底盘或传感器驱动。

部署须一起构建`agribot_hardware_bringup`、本包和`agribot_mobile_app`；只更新手机APK不生效。
先在无底盘输出的观察模式验证新NavSat坐标、航向、定位时延、预热资源占用与重叠误差，
再开展短距离真车切换测试。不可仅凭固定解数字4断言定位安全。
