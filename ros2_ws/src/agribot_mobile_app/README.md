# 农机控制台

该包提供面向手机和平板的可安装PWA，以及运行在RDK X5或Jetson上的ROS 2网关。
它不复制RViz，也不直接发布底盘速度；地图、初始位姿、Nav2 Action、采集任务和
离线处理任务都通过固定白名单接口访问。

主要能力：

- 实时二维地图、全局/局部代价地图、真实车体轮廓和位姿。
- 区分显示已行驶轨迹、Smac全局规划和MPPI局部跟踪路径。
- RViz等价的初始位姿、单目标和连续多位姿导航。
- 在RDK本地启动和安全停止原始传感器数据采集。
- 在启用了离线处理的Jetson实例上调用标准LIO-SAM/RTK处理流程。
- 浏览地图与数据包，按室内或室外配置启动观察阶段或底盘阶段。
- 显示C16、IMU、相机、RTK、定位和底盘反馈状态。
- 新任务启动前自动停止旧任务，并确认旧ROS进程组完全退出。

状态页只通过ROS图查询C16、IMU、相机、RTK、定位和底盘话题是否存在发布者，
不订阅点云、图像或其他原始传感器消息，也不显示未经处理链路校准的频率值。
网页状态以5 Hz刷新导航所需的位姿、路径和地图信息。

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
http://RDK_IP:8088/downloads/agribot-mobile-0.1.0.apk
```

应用默认连接`http://192.168.100.125:8088`。RDK地址变化时，在应用标题栏点击设置图标，
填写新的IP地址即可；配置会保存在本机，连接失败时应用每5秒自动重试。应用只封装现有
网页和白名单API，不绕过网关的运动授权、定位就绪与底盘安全检查。
APK内置完整控制台资源；没有网络或暂时无法连接RDK时仍会正常显示界面，并明确显示
“离线”。后台探测到RDK恢复后会自动切换到实时数据页面。

Android源码位于`android/`。安装Android SDK 35和JDK 17后可执行：

```bash
cd android
./gradlew test lint assembleDebug
```

正式包签名配置参考`android/signing.properties.example`。签名文件和密码不得提交到Git。

Jetson本地已有数据包时，使用带离线处理能力的配置启动第二个实例：

```bash
ros2 launch agribot_mobile_app mobile_app.launch.py \
  params_file:=/home/cgz/agribot_ws/ros2_ws/install/agribot_mobile_app/share/agribot_mobile_app/config/mobile_gateway_jetson.yaml
```

该实例监听`8089`，调用的就是`agribot_offline_mapping`标准流程。RDK实例不会在X5上
运行LIO-SAM；原始数据仍先保存在RDK固态硬盘，再通过现有复制流程放入Jetson数据目录。
手机端的“开始采集”只录制bag，不会在RDK上生成最终2D/3D地图。
