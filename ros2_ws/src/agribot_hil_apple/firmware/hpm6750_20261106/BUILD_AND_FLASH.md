# VCU 固件（HPM6750EVK2）构建、烧录与配置说明

本包是 2026-11-06 在 **HPM6750EVK2** 上编译、烧录并验证过的 VCU 固件源码。

| 项目 | 值 |
| --- | --- |
| 仓库 | `hpm6750evk2` |
| 提交 | `c022f47` |
| 目标板 | HPM6750EVK2（HPM6750IVM，Core0） |
| 构建类型 | `flash_xip`（QSPI NOR，XIP 运行） |
| 运行时 | `VCU_RUNTIME_ENABLE=1`（原生 CAN 桥接可用） |
| 产物 | `prebuilt/demo.elf`、`prebuilt/demo.bin` |

## 1. 目录

```
src/           固件源码（VCFG 配置、A/B 存储、CAN 桥接、FGCP/FGMP）
tests/         主机侧单元测试（平台无关编解码/桥接）
scripts/       串口测试脚本（macOS/Linux，POSIX termios）
prebuilt/      本次实板验证过的 demo.elf / demo.bin
CMakeLists.txt HPM SDK 应用工程
README.md      固件行为、协议与历史验证记录
```

## 2. 依赖

- HPM SDK **1.12.1**
- GNU RISC-V 工具链 `rv32imac_zicsr_zifencei_multilib_b_ext-win`
- CMake ≥ 3.13、Ninja
- 烧录：OpenOCD 0.12（HPM SDK 自带版本即可）+ CMSIS-DAP 探针

## 3. 构建（Windows PowerShell）

```powershell
$env:HPM_SDK_BASE="C:/hpm_sdk_env/hpm_sdk"
$env:GNURISCV_TOOLCHAIN_PATH="C:/hpm_sdk_env/toolchains/rv32imac_zicsr_zifencei_multilib_b_ext-win"
$env:PATH="C:\hpm_sdk_env\tools\cmake\bin;" +
          "C:\hpm_sdk_env\toolchains\rv32imac_zicsr_zifencei_multilib_b_ext-win\bin;" +
          "C:\hpm_sdk_env\tools\python3;C:\hpm_sdk_env\tools\ninja;" + $env:PATH

cmake -S . -B build -GNinja `
  -DBOARD=hpm6750evk2 `
  -DCMAKE_BUILD_TYPE=debug `
  -DHPM_BUILD_TYPE=flash_xip `
  -DVCU_RUNTIME_ENABLE=1

cmake --build build
```

产物在 `build/output/demo.elf`。

> `-DVCU_RUNTIME_ENABLE=1` 才会编入 CAN 收发；不传时是“纯配置固件”，只处理 VCFG、不做桥接。
> 只做配置选择时可用 `-DHPM_BUILD_TYPE=ram` 下载到 RAM 调试。

## 4. 烧录（OpenOCD + CMSIS-DAP）

```powershell
openocd -s <sdk>/boards/openocd `
  -f probes/cmsis_dap.cfg -c "adapter speed 1000" `
  -f soc/hpm6750-single-core.cfg `
  -f boards/hpm6750evk2.cfg `
  -c "init" -c "halt" `
  -c "program <elf> verify" `
  -c "reset run" -c "shutdown"
```

**HPM6750EVK2 + FireDAP 的关键点**：P4 第 10 脚（TRST）必须悬空。

- TRST 接着时，探针会把 TAP 按在复位态，现象是“IDCODE 能读、DMI 超时”；
- 拔掉第 10 脚后 `Examined RISC-V core` 正常；
- 换成 J-Link 时 TRST 可以正常接（J-Link 会正确驱动）。

## 5. VCFG/1 配置帧

UART0，115200 8N1，24 字节定长：

| 偏移 | 内容 |
| --- | --- |
| 0 | `"VCFG"` |
| 4 | 版本 `1` |
| 5 | 操作码：`0x10` 写选择 / `0x90` 写 ACK / `0x20` 状态查询 / `0xA0` 状态 / `0x21` 身份查询 / `0xA1` 身份 |
| 6–7 | 负载长度 `8`（小端） |
| 8–11 | 序号（小端 u32） |
| 12–13 | machine_id |
| 14–15 | protocol_id |
| 16–19 | compute_interface_id（u32，低 8 位有效） |
| 20–23 | CRC32（前 20 字节） |

组合：

| 模式 | machine | protocol | compute |
| --- | --- | --- | --- |
| 阿克曼转向 | `0x0012` | `0x0202` | `1`=CAN / `2`=SocketCAN |
| 三合一履带 | `0x0001` | `0x0101` | `1` / `2` |
| 差速 / 四驱 / 横移 / 全向 | `0x0010` / `0x0011` / `0x0013` / `0x0014` | `0x0201` | 任意 |
| 串口算力 | 任意 | 对应协议 | `0`（UART13） |

写入成功返回 `0x90` ACK：`status=0`、活动槽 A/B、代数、machine、protocol。
相同配置重复写入幂等，不增加代数。

## 6. 本次实板验证

- `program ... verify` → **Programming Finished / Verified OK**
- 身份：`version=1.0.0`、`hardware_ready=1`、`native_bridge=1`、`machine=0x0012`、`protocol=0x0202`
- 状态：schema 2、`compute=1`（CAN1，1 Mbps 原生桥接）
- 无 CAN 流量时 `fault=0x10`（反馈超时）属预期；`cmd_fwd/dropped/fb_fwd` 均为 0

## 7. 注意事项

- 配置存于 QSPI **最后两个扇区**（`0x80FFE000` / `0x80FFF000`），重刷应用区不会丢。
- EVK2 的 **CAN1 没有板载收发器**，必须外接 3.3V CAN 收发器（`PE31/J11[7]` TX、`PE30/J11[10]` RX）；CAN0 用板载收发器接农机。
- 原生桥接只转发白名单经典 CAN 帧（阿克曼：算力 `0x181` → 农机；农机 `0x101~0x103` → 算力），不保持、不重发。
- 运行镜像收到新选择时，若算力端 1 秒内仍有指令，会返回 `VCU_NOT_STANDBY`；先停发再写。
