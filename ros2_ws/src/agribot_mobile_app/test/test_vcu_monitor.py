import struct
import zlib
import pytest
from agribot_mobile_app.vcu_monitor import query, decode_identity, decode_telemetry, VcuMonitor

def frame(opcode, payload, sequence=42):
    body = struct.pack("<4sBBHI", b"VCFG", 1, opcode, len(payload), sequence) + payload
    return body + struct.pack("<I", zlib.crc32(body))

def telemetry(machine=18, protocol=514):
    p = bytearray(64)
    p[0:2] = bytes((2, 15))
    struct.pack_into("<HHH", p, 2, 0, machine, protocol)
    p[8] = 1
    struct.pack_into("<H", p, 10, 15)
    struct.pack_into("<iiiii", p, 24, 75000, -300, 120, -300, 384)
    struct.pack_into("<hh", p, 44, -200, 200)
    p[48] = 255
    p[49] = 1
    struct.pack_into("<III", p, 52, 20, 2, 60)
    return frame(0xa0, p)

def test_queries_and_identity_are_crc_and_sequence_checked():
    request = query(0x21, 42)
    assert len(request) == 24 and request[5] == 0x21
    assert struct.unpack_from("<I", request, 20)[0] == zlib.crc32(request[:20])
    reply = frame(0xa1, struct.pack("<BBBBHH", 1, 0, 0, 3, 18, 514))
    assert decode_identity(reply, 42)["profile"] == "ackermann"
    with pytest.raises(ValueError):
        decode_identity(reply, 41)
    with pytest.raises(ValueError):
        decode_identity(reply[:-1] + bytes((reply[-1] ^ 1,)), 42)

def test_native_angle_is_not_yaw_and_diff_percent_is_not_speed():
    ack = decode_telemetry(telemetry(), 42)
    assert ack["target_steering_angle"] == .384
    assert ack["target_speed"] == -.3
    assert ack["battery_voltage"] == 75
    assert ack["left_rpm"] == -200 and ack["dropped_frames"] == 2
    diff = decode_telemetry(telemetry(1, 257), 42)
    assert diff["target_speed"] is None and diff["target_steering_angle"] is None
    assert diff["target_wheel_percent"] == [-300, 384]

def test_unknown_device_and_missing_port_never_show_online():
    monitor = VcuMonitor("")
    monitor.start()
    assert monitor.snapshot()["online"] is False and monitor._thread is None
    monitor.close()
    with pytest.raises(ValueError):
        decode_identity(frame(0xa1, struct.pack("<BBBBHH", 1, 0, 0, 3, 99, 514)), 42)

def test_absent_and_stale_fields_remain_unknown():
    p = bytearray(64); p[0:2] = bytes((2, 9)); p[8] = 1; p[48:50] = b"\xff\xff"
    struct.pack_into("<HHH", p, 2, 0, 18, 514)
    struct.pack_into("<II", p, 16, 0xffffffff, 0xffffffff)
    result = decode_telemetry(frame(0xa0, p), 42)
    assert result["battery_voltage"] is None and result["left_rpm"] is None
    assert result["command_age_ms"] is None and result["drive_disabled"] is None
