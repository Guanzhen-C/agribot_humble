"""Offline deployment bundle. This module never accesses hardware."""
from __future__ import annotations
import hashlib
import json
import os
from pathlib import Path
import tempfile
from .physical_config import PhysicalConfigError, write_document

INSTRUCTIONS = """# 人工烧录与部署说明

仅适用于 HPM6750EVK2、阿克曼 CAN 配置，固件版本 1.0.0。
CAN1 接 RDK CAN 适配器，CAN0 接执行控制器；两侧均为经典 CAN、1 Mbps。
反馈 0x101/0x102/0x103，控制 0x181；0x181 第 4、5 字节是前轮转角，不是角速度。

1. 断开运动输出、切换人工控制，保持车辆静止，检查供电、共地和终端电阻。
2. 按 manifest.json 核对 SHA-256。由人用适配 HPM6750 的工具烧录
   firmware/hpm6750_ackermann.elf（Flash XIP）。使用 BIN 必须依据 ELF 装载地址，
   不得按 RAM 地址烧录，不得默认全片擦除；保留末尾两个配置扇区。
3. 本固件固定使用阿克曼配置，旧持久化选择不会覆盖该配置，原记录不会被擦除。
   换车必须烧录对应固件，不能仅改界面标签。运行镜像拒绝写配置请求。
4. 重启后用 UART0 115200 8N1 查询身份（VCFG 0x21）和状态（0x20），
   核对版本 1.0.0、machine_id=18、protocol_id=514 和硬件就绪。
5. 架空台架测试零速、三条反馈、遥控接管、断链停车、双向字节一致。
   软件测试和交叉编译不等于实板验收，不得跳过台架验证。
6. 农机控制台导入 ackermann_physical_runtime.json；未导入仍使用原有默认算法。
   控制台不烧录固件。定位、规划、反馈验证后，才由人启用运动。

桥接器只转发新到达的白名单帧，不保持或重发运动指令，不改变原 RDK 与控制器的安全逻辑。
"""

def verified_firmware(root: Path) -> tuple[dict, dict[str, bytes]]:
    try:
        manifest = json.loads((root / "manifest.json").read_text(encoding="utf-8"))
    except (OSError, ValueError) as error:
        raise PhysicalConfigError(f"缺少有效 6750 固件清单: {error}") from error
    expected = {"format": "agribot.hpm6750-firmware", "version": 1,
        "firmware_version": "1.0.0", "board": "hpm6750evk2", "profile": "ackermann",
        "machine_id": 18, "protocol_id": 514, "transport": "classic_can",
        "bitrate": 1000000, "build_type": "flash_xip", "flashing": "manual"}
    if not isinstance(manifest, dict) or any(manifest.get(k) != v for k, v in expected.items()):
        raise PhysicalConfigError("6750 固件的板卡、版本或协议与阿克曼方案不匹配")
    names = {"hpm6750_ackermann.bin", "hpm6750_ackermann.elf"}
    if not isinstance(manifest.get("files"), dict) or set(manifest["files"]) != names:
        raise PhysicalConfigError("固件清单必须包含 BIN 和 ELF")
    files = {}
    for name in sorted(names):
        path = root / name
        if not path.is_file() or not 0 < path.stat().st_size <= 16 * 1024 * 1024:
            raise PhysicalConfigError(f"固件缺失或尺寸异常: {name}")
        data = path.read_bytes()
        record = manifest["files"][name]
        if not isinstance(record, dict) or record.get("size") != len(data) or record.get("sha256") != hashlib.sha256(data).hexdigest():
            raise PhysicalConfigError(f"固件校验失败: {name}")
        if name.endswith(".elf") and (data[:5] != b"\x7fELF\x01" or data[5] != 1 or data[18:20] != b"\xf3\x00"):
            raise PhysicalConfigError("固件 ELF 不是 32 位小端 RISC-V 镜像")
        files[name] = data
    return manifest, files

def export_deployment(output: Path, document: dict, firmware_root: Path) -> Path:
    manifest, firmware = verified_firmware(firmware_root)
    output = output.resolve()
    output.parent.mkdir(parents=True, exist_ok=True)
    bundle = output.with_name(output.stem + "_deployment")
    with tempfile.TemporaryDirectory(prefix=".deployment-", dir=output.parent) as temp:
        staged = Path(temp) / bundle.name
        (staged / "firmware").mkdir(parents=True)
        write_document(staged / "ackermann_physical_runtime.json", document)
        for name, data in firmware.items():
            (staged / "firmware" / name).write_bytes(data)
        write_document(staged / "firmware/manifest.json", manifest)
        (staged / "人工烧录与部署说明.md").write_text(INSTRUCTIONS, encoding="utf-8")
        files = {str(p.relative_to(staged)): hashlib.sha256(p.read_bytes()).hexdigest()
            for p in staged.rglob("*") if p.is_file()}
        write_document(staged / "manifest.json", {"format": "agribot.physical-deployment",
            "version": 1, "preset": document["preset"], "configuration_sha256": document["sha256"],
            "firmware": manifest, "files": files, "flashing": "manual"})
        previous = Path(temp) / "previous"
        if bundle.exists():
            if not bundle.is_dir() or bundle.is_symlink():
                raise PhysicalConfigError("部署目录不是普通目录，拒绝覆盖")
            os.replace(bundle, previous)
        try:
            os.replace(staged, bundle)
        except OSError:
            if previous.exists():
                os.replace(previous, bundle)
            raise
    write_document(output, document)
    return bundle
