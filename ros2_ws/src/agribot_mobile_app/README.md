# 农机控制台

该包提供面向手机和平板的可安装PWA，以及运行在RDK X5或Jetson上的ROS 2网关。
它不复制RViz，也不直接发布底盘速度；地图、初始位姿、Nav2 Action、采集任务和
离线处理任务都通过固定白名单接口访问。

主要能力：

- 实时二维地图、全局/局部代价地图、真实车体轮廓和位姿。
- 区分显示已行驶轨迹、Smac全局规划和MPPI局部跟踪路径。
- RViz等价的初始位姿、单目标和连续多位姿导航。
- 手动规划与自然语言语义导航并列；语义任务经Neo4j检索得到有序目标地点，
  再交给同一套Nav2 `NavigateThroughPoses`和Smac规划执行。
- 在RDK本地启动和安全停止原始传感器数据采集。
- 在启用了离线处理的Jetson实例上调用标准LIO-SAM/RTK处理流程。
- 浏览地图与数据包，按室内或室外配置启动观察阶段或底盘阶段。
- 在阿克曼车和四轮差速车之间切换，同一操作按钮按车型调用各自的定位、
  Smac、MPPI、CAN和数据采集启动文件。
- 显示C16、IMU、单目相机、RTK、定位、底盘反馈以及传感器授时状态。
- 显示RTK、视觉、手动三级初始定位阶段；自动来源失败前禁止误发手动位姿。
- 新任务启动前自动停止旧任务，并确认旧ROS进程组完全退出。

状态页只通过ROS图查询C16、IMU、单目相机、RTK、定位和底盘话题是否存在发布者，
并订阅轻量的`/diagnostics`授时结果；不订阅点云、图像或其他原始传感器消息，
也不显示未经处理链路校准的频率值。
网页状态以5 Hz刷新导航所需的位姿、路径和地图信息。

底盘反馈另外校验实际 `/scout_status` 消息及CAN诊断的新鲜度，不以发布者存在
代替有效反馈。履带车观察阶段默认只读接收CAN，不订阅或发送底盘运动指令；
无需接6750诊断串口即可显示电压、故障码和左右RPM。桌面差速GUI与控制台
运行栈仍互斥使用同一USB-CAN，切换前先停止另一方；占用或超时会明确提示。

顶部提供“开灯 / 关灯”按钮，复用桌面差速CAN GUI的前灯协议：`0x514`的
第4字节bit0控制灯光，`0x532`第2字节bit2反馈实际状态。灯光通过现有底盘节点的
`/hardware/set_headlight`（`std_srvs/SetBool`）服务切换，不另开CAN进程、
不解锁底盘、不发布速度或导航目标，也不改变已有运动安全门控。
HTTP接口为`POST /api/v1/chassis/headlight`，请求示例：
`{"vehicle_type":"differential","enabled":true}`。底盘接受指令后返回目标开关状态，
按钮据此切换为反向操作；`confirmed`单独表示是否收到对应灯光反馈。
部分底盘不回报灯光状态，缺少该反馈不再当成下发失败，也不阻止关灯。
按钮提示区分最近下发的指令与实际反馈，不将指令接受冒充灯具状态确认。
观察阶段仍严格只读；离线、反馈过期、车型不匹配时禁止控制。
当前阿克曼协议未定义灯光字段，因此不支持灯光开关，界面按钮禁用。

手机采集页固定记录单目相机，不启动NTRIP。所选底盘决定使用阿克曼或差速车的
传感器外参与采集启动文件。结束实验时点击“停止采集并保存”，
网关会向rosbag发送SIGINT并等待索引写盘完成；不要直接关机或强制终止采集进程。

## 扫描轨迹固定路线

地图同基名的`_routes.json`保存离线转换后的固定途经点。在导航页选择差速车型及对应地图，
切换到“经停点”，选择固定路线并点击“载入扫描路线”。载入只替换待执行点位，并立即在
地图上预览，不会发出任何导航或底盘命令。仍需单独点击“执行路线”，沿用现有定位就绪、
导航任务互斥和底盘授权检查。可以逐点删除或清空；切换地图或车型会清除旧的待执行点位。

只读接口为`GET /api/v1/maps/{map_id}/routes`。路线绑定地图地理配准文件摘要、
栅格原点和分辨率，原点或配准改变后拒绝载入；只修改PGM中的障碍像素不影响绑定。
每条路线限制为2至100点，不把原始扫描的数千帧全部交给Nav2。

离线使用`agribot_offline_mapping/scripts/create_capture_route_preset.py`：
先将扫描轨迹从源ENU经ECEF转换到目标地图坐标系，再按有序RDP保留关键点，默认简化
误差0.25米；不会修改PGM、清除障碍或保证扫描人员走过的地方满足整车净空。
它同时输出`_capture_route.csv`和`_routes_preview.png`，并拒绝覆盖已有结果。
每个点均保留朝向；实际最终轨迹仍由Smac生成，预览折线不是已通过碰撞检测的规划轨迹。

## 差速室外初始化操作

当前 `differential_outdoor` 使用 `agribot_localization_switch/outdoor_waypoint.launch.py`：

1. 选择履带差速农机、室外流程和地图，先启动观察阶段。
2. 保持车辆静止，点击“开始KF-GINS初始化”，确认固定解4、有效航向及定位就绪。
3. 保存路线使用 KF-GINS/NavSat 行驶；有序进度达到原编号第100个人工经停点时，
   后台启动 FAST-LIVO2＋RTK。预热期间不取消路线，继续由 KF-GINS 提供定位。
4. 两套定位通过同时刻位姿差异和连续1秒健康检查后接管；首次输出连续，后续修正限速释放。
   接管后关闭 KF-GINS。控制台导航页显示当前定位方法和预热状态。

这里的第100点不是第100个稠密路径采样。从第51点起步仍在原编号100触发；
从100或之后起步立即预热；不足100个经停点的路线、手动目标不自动切换。
预热期间若 KF-GINS 也失去健康定位，仍会暂时停车并保留路线，不允许使用过期位姿盲行。
KF-GINS首次就绪后，单帧异常或非固定解只丢弃该RTK观测；在INS持续更新且XY方差
不超过0.25平方米时，最多允许4秒的RTK中断，由真实IMU预测维持输出。
正常固定解恢复后直接继续滤波，不因单帧异常取消任务；持续失联、INS过期或协方差过大仍暂停。
完整规则、话题和部署要求见 [定位切换说明](../agribot_localization_switch/README.md)。

此流程不启动 NDT/GICP 初始配准，也不伪造点云配准成功；地图地理配准和真实天线杆臂仍必需。
定位成功不等于可通行，碰撞检查、底盘反馈、安全门控均保留。
阿克曼、差速室内流程，以及直接调用原硬件室外 launch 的默认定位方式不变。
HTTP 初始化接口仍为 `POST /api/v1/localization/start`，按钮不下发目标或速度。

## 语义导航

手机通过 VPN 直接请求 `http://172.18.80.26:8090`。服务器使用本地
`qwen3.7-flash`、`text-embedding-v4`和Neo4j混合检索生成有序目标与禁行地点；
手机再通过热点局域网把结果提交给RDK的`/api/v1/semantic/route`。RDK不访问
大模型或Neo4j，也不保存模型与数据库凭据。手机不能访问Neo4j和Ollama端口，服务器
也没有控制底盘的接口。

语义规划只发送地图ID、自然语言任务和当前`map`二维位姿。服务器返回结果后，RDK会
重新校验地图、模型、图谱哈希、目标位姿、A*中心线和避让标记。拓扑A*只负责选择
道路，不作为必须逐点到达的Nav2目标。网关以中心线左右各2米构造自由走廊，再用
1米过渡带升到高代价；路线与避让代价直接加到原始Nav2代价上并在252饱和。
Nav2仍只接收语言模型要求的有序目的地，由Smac在宽走廊内生成满足阿克曼几何的路径。

手动规划和语义导航的界面、草稿与API互相独立，但两者共享同一个Nav2 Action服务器，
因此同一时刻只允许一个导航任务。切换界面不会停止当前任务；新任务必须先明确取消旧任务。
语义服务地址由`config/mobile_gateway.yaml`中的`semantic_service_url`公开给手机网页。
该地址只用于手机发起跨域请求，网关自身不会请求它。

构建前端后再构建ROS包：

```bash
cd src/agribot_mobile_app/web
npm ci
npm run build
cd ../../..
colcon build --packages-select agribot_mobile_app --symlink-install
```

RDK启动：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/agribot_ws/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=0
export ROS_LOCALHOST_ONLY=1
ros2 launch agribot_mobile_app mobile_app.launch.py
```

手机模式默认把网关及其启动的定位、规划、控制和采集节点限制在RDK本机DDS中，
避免弱Wi-Fi、远程RViz或其他ROS 2节点影响传感器融合实时性。HTTP仍监听
`0.0.0.0:8088`，所以手机通过RDK的局域网IP正常访问，不受该限制影响。
通过SSH在RDK上执行`ros2 topic`或`ros2 node`检查手机启动的流程时，也要先设置
`export ROS_LOCALHOST_ONLY=1`。

在RDK上注册开机服务：

```bash
source /opt/ros/humble/setup.bash
source /home/sunrise/agribot_ws/ros2_ws/install/setup.bash
sudo install -m 0644 \
  "$(ros2 pkg prefix agribot_mobile_app)/share/agribot_mobile_app/systemd/agribot-mobile-app.service" \
  /etc/systemd/system/agribot-mobile-app.service
sudo systemctl daemon-reload
sudo systemctl enable --now agribot-mobile-app.service
```

该服务只启动手机网关，不会自动启动导航栈、CAN或底盘输出。只有在手机端明确选择
运行配置并确认后，网关才会在同一本机DDS环境中启动相应流程。
查看状态与日志：

```bash
systemctl status agribot-mobile-app.service --no-pager
journalctl -u agribot-mobile-app.service -f
```

同一局域网内在手机浏览器打开 `http://RDK_IP:8088`，然后使用浏览器的“添加到主屏幕”。
`api_token`非空时，所有会改变车辆状态的请求必须携带该口令。离线处理默认关闭，
只应在已编译`agribot_offline_mapping`且具有足够算力的Jetson实例上开启。

## Android安装包

Android 8.0及以上设备可直接安装原生壳应用：

```text
http://RDK_IP:8088/downloads/agribot-mobile-0.3.6.apk
```

应用默认连接`http://192.168.100.125:8088`。RDK地址变化时，在应用标题栏点击设置图标，
填写新的IP地址即可；配置会保存在本机，连接失败时应用每5秒自动重试。应用只封装现有
网页和白名单API，不绕过网关的运动授权、定位就绪与底盘安全检查。
APK内置完整控制台资源；没有网络或暂时无法连接RDK时仍会正常显示界面，并明确显示
“离线”。后台探测到RDK恢复后会自动切换到实时数据页面。

Android源码位于`android/`。安装Android SDK 35和JDK 17后可执行：

```bash
rsync -a --delete --exclude downloads \
  web/dist/ android/app/src/main/assets/web/
cd android
./gradlew test lint assembleDebug
```

`downloads/`必须排除，避免把旧APK递归打包进新APK。使用安装了 Android SDK 的
x86 构建电脑（原 218 服务器），SDK 路径为`/home/cgz/Android/Sdk`，不在 RDK 上构建。

0.3.5 修复底盘不回报灯光位时的误报超时；成功下发后按钮切换为相反操作。

0.3.4 增加顶部车灯开关和真实CAN灯光反馈；必须同步更新网关和底盘节点，
只更新APK无法启用灯光服务。观察阶段仍保持CAN只读，运动安全门控不变。

0.3.3 更新差速室外 KF-GINS 初始化按钮及第100点 FAST-LIVO2＋RTK 预热/接管状态。
必须同时部署配套网关和 `agribot_localization_switch`，仅安装APK不能切换定位算法。

0.2.8 同步差速室外流程的“使用 NDT/GICP 精配准”选项及独立的“开始初始化”按钮，
同时更新内置离线界面。配套 RDK 网关及定位节点必须更新后才能使用新初始化流程。
保持与 0.2.7 相同的应用包名和签名，可覆盖安装并保留已保存的网关地址。

0.2.9 增加按地图、车型绑定的扫描固定路线载入，更新内置离线界面。
配套网关使用等价查表读取PGM，并对大JSON栅格响应启用HTTP gzip；客户端不接受gzip时
仍返回原来的JSON。占用值、地图分辨率及坐标系不变。这些优化不替代Smac规划或碰撞检查。

0.2.7 同步最新控制台内置界面，按实际 CAN 反馈新鲜度显示底盘状态、左右转速和
只读接收状态；未启用 6750 诊断不影响底盘反馈显示。保留离线显示、网关地址设置和真机配置导入。
电脑浏览器的 Web Serial 诊断不属于 Android WebView 功能；手机仍通过 RDK 网关访问车辆。

正式包签名配置参考`android/signing.properties.example`。签名文件和密码不得提交到Git。

## HPM6750 控制台集成

保留原有算法运行、采集、导航和语义任务 API。全界面采用 FieldGate VCU（原作者刘泽禹）
的桌面布局、玻璃顶栏、底部图标导航、字体、间距、配色和控件样式。总览、导航、采集、
地图、设备、诊断和设置为独立内容页，打开时仍默认进入导航。设备未回复时不显示已连接。
参考背景和车辆示意为本地静态 PNG，不加载 Unity 或高精度三维场景；移动端可以关闭
玻璃效果。浏览器以及 APK 内置离线界面使用相同资源。
没有导入真机配置时仍使用程序默认方案；地图页的导入不启动运行、不授权运动。

6750 桥接固件通过 Unity 的“停止仿真 → 导出真机配置”导出，由操作人员人工烧录。
导入控制台的是部署包中的 `ackermann_physical_runtime.json`，不是 ELF/BIN。
本控制台没有烧录接口，也不把移动端 Web Serial 作为通信条件。

将 6750 UART0 的诊断串口接到 RDK 后，可选启动只读监控：

```bash
ros2 launch agribot_mobile_app mobile_app.launch.py \
  vcu_diagnostic_port:=/dev/serial/by-id/实际6750诊断串口
```

诊断串口必须是 6750 的 UART0，不得填写正在被 CAN 驱动占用的 USB-CAN 串口。
默认此参数为空，不打开任何串口；配置后独立线程每秒查询身份和状态，校验序号及 CRC。
断连即清空有效状态。界面展示真实电压、帧年龄、转发/丢弃计数；无设备报告的电量百分比
和功能状态显示“未提供”。阿克曼目标转角与角速度分别显示，履带目标为左右百分比。
传感器安装表只读展示所选运行车辆的配置文件，不向车辆写入新的外参。

Jetson本地已有数据包时，使用带离线处理能力的配置启动第二个实例：

```bash
ros2 launch agribot_mobile_app mobile_app.launch.py \
  params_file:=/home/cgz/agribot_ws/ros2_ws/install/agribot_mobile_app/share/agribot_mobile_app/config/mobile_gateway_jetson.yaml
```

该实例监听`8089`，调用的就是`agribot_offline_mapping`标准流程。RDK实例不会在X5上
运行LIO-SAM；原始数据仍先保存在RDK固态硬盘，再通过现有复制流程放入Jetson数据目录。
手机端的“开始采集”只录制bag，不会在RDK上生成最终2D/3D地图。

## 可选真机配置导入

Unity 停止受管仿真后，可导出 `ackermann_physical_runtime.json`。
在控制台的“地图 → 运行配置”中点击“导入真机配置”，选择该文件。
只支持已验证的阿克曼 STVL / FAST-LIVO2+RTK / NDT-GICP / Smac Hybrid-A* / MPPI 方案；
算法和参数必须与车端已安装版本一致。不导入时，原运行配置及命令完全不变。

导入不安装算法包、不导入地图、不启动车辆；设备端口、地图选择与运动确认继续由原流程负责。
运行或采集期间不能更换配置。“恢复默认”会删除保存的导入选择，不改变原程序参数。
配置持久化在 `~/.local/state/agribot-mobile-app/physical-config/`，可通过
`physical_config_root` 指定路径；跨版本失效会显示错误并使用程序默认配置。
Android 文件选择器支持需要更新到 0.2.3 APK，旧 APK 不具备原生文件选择入口。

## 已保存路径模式（差速车，0.3.2）

在“导航 → 手动规划 → 经停点”通过“手动添加路径 / 预先设定路径”切换。
手动模式只显示地图选点列表和执行路线按钮；预设模式只显示保存路径、起点编号、
预览和执行按钮。旧的固定路线和“载入扫描路线”界面已移除。
预设模式中地图只允许浏览，不会误加手动点；导航期间禁止切换模式。
“预览后续路径”只显示该起点之后的密集路径，不发运动指令；
同时标记每个人工经停点的原始编号，例如第51点开始显示51～104。
密集路径采样点不编号；数字错位时用引线指向对应路径位置，不遮挡地图工具。
“执行保存路径”仍要求运行地图、车型、融合定位和实时局部代价地图正确。
车辆离所选起点超过0.35米或方向差超过0.4弧度时，先用Smac规划接入段。
进入路线后直接使用FollowPath/MPPI，不调用周期性全程ComputePathThroughPoses。
原有手动经停点、语义导航及其行为树不改变。

前方1米内的整车轮廓持续受阻2秒，或12秒没有沿路径前进时，取消并确认原跟踪结束，
再尝试接回原路径前方4/7/10米处。只规划接入/绕行段，不重新规划完整104点。
无法绕行时停止任务；未知栅格不再触发保存路径监测的受阻判定。
移除网关中定位1秒、局部地图3秒过期即终止保存路径任务的规则，年龄只保留诊断。
原有底盘限速、CAN互斥、融合定位就绪和底盘安全门控保留。
差速融合定位允许最长4秒的里程计接收间隔；超出后撤销就绪状态。
运行中的保存路径任务遇到定位未就绪时，取消并确认旧MPPI跟踪结束，停车保留路线和有序进度，
不直接置为失败，也不消耗绕行次数。融合重新就绪且定位、局部地图的新数据连续稳定1秒后，
从保留的路线进度继续；如需要，先用Smac规划接回段。等待期间不计入12秒无进展时间。
定位一直未恢复时保持停车等待，用户可随时取消；取消后不会自动续跑。
绕行只允许最多6次尝试周期，失败后需要人工检查并重新启动。
按路线进度进入最后1米且到达配置中的终点位置容差后，取消跟踪并确认停止，
不再强求末端朝向、不再触发额外规划；中间点仅为起点索引，不是逐点停车目标。

地图旁的`<map>_saved_paths.json`包含密集路径、人工点索引和SHA256绑定。
PGM、地图YAML、地理配准、Nav2参数或差速运动基元变化后拒绝加载，需重新验证导出。
全局代价地图完整发布降为0.2Hz，仅用于大范围显示；其内部更新仍为2Hz，
不降低Smac规划使用的地图更新频率。近距离受阻检查使用8×8米局部代价地图，
其内部更新5Hz、完整发布2Hz（160×160格），静止时也持续刷新。
远处4/7/10米绕行接回点可以在局部窗口以外，必须由Smac检查全局可达性，
成功后才交给MPPI；不会把窗口外区域当作可直接通行的直线。
网关为定位、底盘、导航应答、各地图和后台探测分配独立回调组，定位和底盘队列仅保留最新一条。
手机的历史轨迹默认从已接收的融合里程计抽样生成，最多保留600个显示点，
不再反复反序列化增长到数千点的完整ROS轨迹；原始ROS轨迹和算法输出不变。
需要接收独立历史话题时可显式设置`trajectory_from_pose:=false`，低算力平台不建议开启。
定位1秒、局部地图3秒门限仅用于诊断，不再作为保存路径任务的终止条件。
仍拒绝尚未收到输入、错误坐标系、非有限或异常未来时间戳；启动新任务仍要求融合定位就绪。
等待恢复时使用数据新鲜度判定是否可以继续，不把旧的就绪标志当成定位已经恢复。
`/api/v1/state`的`navigation_inputs`提供接收年龄、测量年龄、最大间隔和超时次数；
任务失败日志会指出具体哪一路过期。此项仍需实际X5运行和真车行驶验收。

使用`tools/save_computed_path.py`从已验证的规划CSV与人工点JSON创建文件；
此工具拒绝覆盖已有文件，不修改人工点或地图。发布手机APK时必须同步部署Python后端、
差速Nav2 YAML和此路径文件；仅安装APK不会更新车端算法。

`tools/test_saved_path_navigation.py`要求ROS_DOMAIN_ID=81、ROS_LOCALHOST_ONLY=1。
配合真实Nav2 planner_server/controller_server及差速运行参数进行理想运动学测试，
测试场景由SAVED_PATH_TEST_SCENARIO选择obstacle/cancel/stale/wall/entry。
这不是Gazebo物理、FAST-LIVO2定位或实车验收，不应据此声称X5实时性能已通过。

一键验证命令（测试完成后Ctrl+C退出Nav2节点）：

```bash
export ROS_DOMAIN_ID=81 ROS_LOCALHOST_ONLY=1
ros2 launch agribot_mobile_app saved_path_validation.launch.py scenario:=obstacle
```
