# WHEELTEC 大型阿克曼触控 GUI

该 GUI 直接使用智嵌物联 ZQWL-CANFD USB 适配器的 CDC 协议，不依赖
`can0`、ROS 或第三方 Python 包。默认设备为：

```text
/dev/serial/by-id/usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00
```

操作规则：

- 默认锁定，收到 C50C 的 `0x101`、`0x102`、`0x103` 遥测后才能启用。
- 触摸或鼠标按住方向按钮运动，松开停止。
- `WASD` 或方向键控制，空格立即停止并锁定，`F11` 切换全屏。
- 窗口失焦、遥测中断及退出都会停止并锁定。
- 同一时间只能由 GUI 或 ROS 底盘节点中的一个占用 USB 适配器。

安装后启动：

```bash
source /opt/ros/humble/setup.bash
source ~/agribot_ws/ros2_ws/install/setup.bash
ros2 pkg prefix agribot_hardware_bringup
~/agribot_ws/ros2_ws/install/agribot_hardware_bringup/lib/agribot_hardware_bringup/start_wheeltec_car_gui.sh
```

不接硬件的协议和界面自检：

```bash
python3 wheeltec_car_gui.py --self-test
DISPLAY=:0 XAUTHORITY=/home/sunrise/.Xauthority \
  python3 wheeltec_car_gui.py --ui-self-test
```

只发送零速度并验证三条反馈帧：

```bash
python3 wheeltec_car_gui.py --link-test
```
