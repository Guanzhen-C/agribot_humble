# Unity、6750 和农机控制台部署交接

## 不变的边界

真机定位、感知、规划、控制算法和原有运行命令不变。默认运行仍使用已验证方案，
控制台的配置导入是可选的，不创建任务、不启用运动、不烧录固件。
Unity 负责车辆配置和仿真；6750 负责原生 CAN 桥接；控制台负责运行现有 ROS 栈及显示状态。

当前交接包只接受已验证的阿克曼真机配置，不将任意仿真算法或参数自动下发真车。
履带车的 6750 固件已提供，履带真机配置导出仍需另行按验证方案扩展。

## 导出

在 Unity 点击“停止仿真”，确认停止完成后点击“导出真机配置”。原 JSON 输出保留，
同目录新增 `ackermann_physical_runtime_deployment/`：

```text
ackermann_physical_runtime.json
firmware/hpm6750_ackermann.bin
firmware/hpm6750_ackermann.elf
firmware/manifest.json
manifest.json
人工烧录与部署说明.md
```

6750 仓库单独维护，默认位置 `ros2_ws/hpm6750evk2`。按照其 `docs/native_can_bridge.md`
编译固件后，启动导出。若固件放在其他位置，设置 `AGRIBOT_HPM_FIRMWARE_ROOT`。
SDK、工具链、固件构建产物和 Unity 导出文件不是 ROS 算法源码，不纳入主项目 Git。
固件缺失、SHA-256 不符、板卡/版本/协议不匹配时拒绝导出，不声称完成部署。

可独立执行：

```bash
PYTHONPATH=src/agribot_vehicle_config_tools /usr/bin/python3 \
  -m agribot_vehicle_config_tools.cli export-physical \
  --workspace-src src \
  --output "$HOME/unity_exports/ackermann_physical_runtime.json" \
  --firmware-root hpm6750evk2/build/releases/ackermann
```

只需 JSON 的已有调用仍兼容：省略 `--firmware-root` 即仅导出 JSON。
Unity 的导出按钮始终要求固件齐全，不静默退化为 JSON-only。

## 导入与烧录

先按人工说明烧录 6750，核对身份，完成台架验收。控制台地图页点击导入，选择 JSON。
控制台仅接受与安装版本一致的验证方案，修改任意参数/算法将被拒绝。
正在导航或采集时不能更换配置，恢复默认也不影响原有启动命令。

6750 的 CAN1 接 RDK，CAN0 接执行控制器。RDK 原本使用的 USB-CAN 可接 CAN1，不需换控制算法。
诊断 UART0 可另外接入 RDK；只读查询不会占用 USB-CAN 运动串口，也不会产生 CAN 运动指令。

## 验证结果与现场验收

已具备主机协议测试、真实 ROS 编码器与转发比对、官方 SDK 交叉编译、导出/导入回归测试、
控制台桌面/手机浏览器测试及 Unity 可执行程序构建。
这些不等于实板验收。现场仍需人工烧录、核对线序/终端电阻、验证反馈稳定性、零速、
急停/遥控接管以及断链停车；本次实现不会自主让车辆移动。

## Car1-Car5 离线草案

Car1-Car5 在 Unity 完成当前车辆配置后可点击“导出真机配置”，生成
`~/unity_exports/unity_carN_physical_draft/`。该目录包含当前车辆 JSON、
ROS_FLU 安装位姿、导航几何、运动学、FAST-LIVO2 外参和 SHA-256 清单，
`manifest.json` 明确标记 `calibrationVerified=false` 与 `deployable=false`。
它不包含底盘协议或固件，不能导入当前农机控制台或直接用于真车运动。
Car0 仍使用上文的已验证导出流程。
