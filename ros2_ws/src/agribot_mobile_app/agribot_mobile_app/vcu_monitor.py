"""Read-only HPM6750 diagnostics, isolated from ROS and motion transport."""
from __future__ import annotations
import struct
import threading
import time
import zlib

def query(opcode: int, sequence: int) -> bytes:
    body = struct.pack("<4sBBHI8s", b"VCFG", 1, opcode, 8, sequence, b"\0" * 8)
    return body + struct.pack("<I", zlib.crc32(body))

def checked_frame(data: bytes, opcode: int, sequence: int, length: int) -> bytes:
    if len(data) != length or data[:4] != b"VCFG" or data[4:6] != bytes((1, opcode)):
        raise ValueError("6750回复格式错误")
    payload_length, reply_sequence = struct.unpack_from("<HI", data, 6)
    if payload_length != length - 16 or reply_sequence != sequence:
        raise ValueError("6750回复长度或序号错误")
    if struct.unpack_from("<I", data, length-4)[0] != zlib.crc32(data[:-4]):
        raise ValueError("6750回复CRC错误")
    return data[12:-4]

def decode_identity(data: bytes, sequence: int) -> dict:
    p = checked_frame(data, 0xa1, sequence, 24)
    major, minor, patch, flags, machine, protocol = struct.unpack("<BBBBHH", p)
    if flags & ~3 or (machine, protocol) not in ((18, 514), (1, 257)):
        raise ValueError("6750原生桥接配置不支持")
    return {"firmware_version": f"{major}.{minor}.{patch}", "machine_id": machine,
        "protocol_id": protocol, "profile": "ackermann" if machine == 18 else "tracked",
        "native_bridge": bool(flags & 2), "hardware_ready": bool(flags & 1)}

def decode_telemetry(data: bytes, sequence: int) -> dict:
    p = checked_frame(data, 0xa0, sequence, 80)
    if p[0] != 2:
        raise ValueError("6750需使用原生桥接状态协议v2")
    flags = p[1]
    fault, machine, protocol = struct.unpack_from("<HHH", p, 2)
    valid = struct.unpack_from("<H", p, 10)[0]
    if flags & ~15 or valid & ~15 or p[8] not in (1, 2) or (machine, protocol) not in ((18, 514), (1, 257)):
        raise ValueError("6750状态值错误")
    ages = struct.unpack_from("<II", p, 16)
    voltage, speed, yaw, target0, target1 = struct.unpack_from("<iiiii", p, 24)
    return {"machine_id": machine, "protocol_id": protocol, "configured": bool(flags & 1),
        "command_fresh": bool(flags & 2), "feedback_fresh": bool(flags & 4),
        "hardware_ready": bool(flags & 8), "fault": fault,
        "uptime_ms": struct.unpack_from("<I", p, 12)[0],
        "command_age_ms": None if ages[0] == 0xffffffff else ages[0],
        "feedback_age_ms": None if ages[1] == 0xffffffff else ages[1],
        "battery_voltage": voltage / 1000 if valid & 1 else None,
        "linear_velocity": speed / 1000 if valid & 2 else None,
        "angular_velocity": yaw / 1000 if valid & 2 else None,
        "target_speed": target0 / 1000 if flags & 2 and machine == 18 else None,
        "target_steering_angle": target1 / 1000 if flags & 2 and machine == 18 else None,
        "target_wheel_percent": [target0, target1] if flags & 2 and machine == 1 else None,
        "left_rpm": struct.unpack_from("<h", p, 44)[0] if valid & 4 else None,
        "right_rpm": struct.unpack_from("<h", p, 46)[0] if valid & 8 else None,
        "work_mode": None if p[48] == 255 else p[48],
        "drive_disabled": None if p[49] == 255 else bool(p[49]),
        "command_forwarded": struct.unpack_from("<I", p, 52)[0],
        "dropped_frames": struct.unpack_from("<I", p, 56)[0],
        "feedback_forwarded": struct.unpack_from("<I", p, 60)[0]}

class VcuMonitor:
    def __init__(self, port: str, changed=lambda: None):
        self.port = port
        self._changed = changed
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread = None
        self._state = {"configured": bool(port), "online": False, "port": port,
            "identity": None, "telemetry": None, "error": "等待身份回复" if port else "未配置6750诊断串口"}

    def _publish(self, **values):
        with self._lock:
            self._state.update(values)
        self._changed()

    def snapshot(self):
        with self._lock:
            return dict(self._state)

    def start(self):
        if self.port and self._thread is None:
            self._thread = threading.Thread(target=self._run, name="vcu-monitor", daemon=True)
            self._thread.start()

    def close(self):
        self._stop.set()
        if self._thread:
            self._thread.join(timeout=2)

    def _read(self, port, opcode, sequence, size):
        port.write(query(opcode, sequence))
        buffer = bytearray()
        deadline = time.monotonic() + 0.6
        while not self._stop.is_set() and time.monotonic() < deadline:
            buffer.extend(port.read(min(128, max(1, port.in_waiting))))
            start = buffer.find(b"VCFG")
            if start < 0:
                buffer[:] = buffer[-3:]
            elif start:
                del buffer[:start]
            if len(buffer) >= size:
                frame = bytes(buffer[:size])
                checked_frame(frame, opcode | 0x80, sequence, size)
                return frame
        raise TimeoutError("6750回复超时")

    def _run(self):
        sequence = 0
        while not self._stop.is_set():
            try:
                import serial
                with serial.Serial(self.port, 115200, timeout=0.05, write_timeout=0.3, exclusive=True) as port:
                    while not self._stop.is_set():
                        sequence = (sequence + 1) & 0xffffffff
                        identity = decode_identity(self._read(port, 0x21, sequence, 24), sequence)
                        sequence = (sequence + 1) & 0xffffffff
                        telemetry = decode_telemetry(self._read(port, 0x20, sequence, 80), sequence)
                        if any(identity[k] != telemetry[k] for k in ("machine_id", "protocol_id")):
                            raise ValueError("6750身份与状态配置不一致")
                        self._publish(online=True, identity=identity, telemetry=telemetry, error="", received_at=time.time())
                        self._stop.wait(1)
            except Exception as error:
                self._publish(online=False, identity=None, telemetry=None, error=str(error))
                self._stop.wait(2)
