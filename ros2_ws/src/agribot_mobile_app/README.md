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

手机采集页固定记录单目相机，不启动NTRIP。所选底盘决定使用阿克曼或差速车的
传感器外参与采集启动文件。结束实验时点击“停止采集并保存”，
网关会向rosbag发送SIGINT并等待索引写盘完成；不要直接关机或强制终止采集进程。

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
http://RDK_IP:8088/downloads/agribot-mobile-0.2.5.apk
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

`downloads/`必须排除，避免把旧APK递归打包进新APK。当前Jetson不保存Android SDK；
构建使用`192.168.100.218`上的`/home/cgz/Android/Sdk`。

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
