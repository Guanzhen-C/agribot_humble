# Agribot mobile app

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
ros2 launch agribot_mobile_app mobile_app.launch.py
```

同一局域网内在手机浏览器打开 `http://RDK_IP:8088`，然后使用浏览器的“添加到主屏幕”。
`api_token`非空时，所有会改变车辆状态的请求必须携带该口令。离线处理默认关闭，
只应在已编译`agribot_offline_mapping`且具有足够算力的Jetson实例上开启。

Jetson本地已有数据包时，使用带离线处理能力的配置启动第二个实例：

```bash
ros2 launch agribot_mobile_app mobile_app.launch.py \
  params_file:=/home/cgz/agribot_ws/ros2_ws/install/agribot_mobile_app/share/agribot_mobile_app/config/mobile_gateway_jetson.yaml
```

该实例监听`8089`，调用的就是`agribot_offline_mapping`标准流程。RDK实例不会在X5上
运行LIO-SAM；原始数据仍先保存在RDK固态硬盘，再通过现有复制流程放入Jetson数据目录。
手机端的“开始采集”只录制bag，不会在RDK上生成最终2D/3D地图。
