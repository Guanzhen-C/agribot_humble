#!/usr/bin/env python3

import argparse
import datetime
import hashlib
import socket
from pathlib import Path


PACKET_SIZE = 1206
DEVICE_HEADER = bytes.fromhex("a5 ff 00 5a 11 11 55 55")
CONFIG_HEADER = bytes.fromhex("aa 00 ff 11 22 22 aa aa")
PACKET_TAIL = bytes.fromhex("0f f0")
CLOCK_SOURCE_OFFSET = 44
TARGET_ANGLE_OFFSET = 46
ANGLE_ERROR_OFFSET = 48
GPS_STATUS_OFFSET = 92
PPS_STATUS_OFFSET = 93


def parse_angle_error(raw_value):
    valid = (raw_value & 0x8000) == 0
    signed_value = raw_value & 0x7FFF
    if signed_value & 0x4000:
        signed_value -= 0x8000
    return valid, signed_value / 100.0


def parse_device_packet(packet):
    if len(packet) != PACKET_SIZE:
        raise ValueError(f"设备包长度错误：{len(packet)}，期望{PACKET_SIZE}")
    if packet[:8] != DEVICE_HEADER:
        raise ValueError("设备包识别头不正确")
    if packet[-2:] != PACKET_TAIL:
        raise ValueError("设备包包尾不正确")
    clock_source = int.from_bytes(
        packet[CLOCK_SOURCE_OFFSET : CLOCK_SOURCE_OFFSET + 2], "big"
    )
    target_hundredths = int.from_bytes(
        packet[TARGET_ANGLE_OFFSET : TARGET_ANGLE_OFFSET + 2], "big"
    )
    raw_error = int.from_bytes(
        packet[ANGLE_ERROR_OFFSET : ANGLE_ERROR_OFFSET + 2], "big"
    )
    pps_valid, angle_error_deg = parse_angle_error(raw_error)
    return {
        "clock_source": clock_source,
        "target_angle_deg": target_hundredths / 100.0,
        "pps_valid": pps_valid,
        "angle_error_deg": angle_error_deg,
        "gps_status": packet[GPS_STATUS_OFFSET],
        "pps_status": packet[PPS_STATUS_OFFSET],
    }


def build_config_packet(device_packet, target_angle_deg):
    parse_device_packet(device_packet)
    target_hundredths = round(target_angle_deg * 100.0)
    if not 0 <= target_hundredths < 36000:
        raise ValueError("PPS目标角必须在[0, 360)度范围内")
    packet = bytearray(device_packet)
    packet[:8] = CONFIG_HEADER
    packet[TARGET_ANGLE_OFFSET : TARGET_ANGLE_OFFSET + 2] = (
        target_hundredths.to_bytes(2, "big")
    )
    packet[-2:] = PACKET_TAIL
    return bytes(packet)


def receive_device_packet(udp_socket, device_ip, timeout_sec):
    udp_socket.settimeout(timeout_sec)
    while True:
        packet, address = udp_socket.recvfrom(2048)
        if address[0] != device_ip or packet[:8] != DEVICE_HEADER:
            continue
        parse_device_packet(packet)
        return packet


def print_status(status):
    clock_names = {0: "GPS", 1: "PTP"}
    print(f"时钟源：{clock_names.get(status['clock_source'], status['clock_source'])}")
    print(f"PPS目标角：{status['target_angle_deg']:.2f} deg")
    print(f"PPS输入有效：{'是' if status['pps_valid'] else '否'}")
    print(f"PPS角度误差：{status['angle_error_deg']:+.2f} deg")
    print(f"GPS状态原始值：{status['gps_status']}")
    print(f"PPS状态原始值：{status['pps_status']}")


def save_packets(backup_directory, original, configured):
    backup_directory.mkdir(parents=True, exist_ok=True)
    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    original_path = backup_directory / f"device_packet_before_{timestamp}.bin"
    configured_path = backup_directory / f"config_packet_{timestamp}.bin"
    original_path.write_bytes(original)
    configured_path.write_bytes(configured)
    print(f"原设备包备份：{original_path}")
    print(f"配置包备份：{configured_path}")
    print(f"原设备包SHA256：{hashlib.sha256(original).hexdigest()}")


def main():
    parser = argparse.ArgumentParser(
        description="读取或配置镭神C16的PPS对齐水平角度"
    )
    parser.add_argument("--device-ip", default="192.168.1.200")
    parser.add_argument("--local-ip", default="192.168.1.102")
    parser.add_argument("--listen-port", type=int, default=2369)
    parser.add_argument("--command-port", type=int, default=2368)
    parser.add_argument("--timeout-sec", type=float, default=3.0)
    parser.add_argument("--samples", type=int, default=5)
    parser.add_argument("--target-angle-deg", type=float)
    parser.add_argument("--apply", action="store_true")
    parser.add_argument(
        "--backup-directory",
        type=Path,
        default=Path.home() / ".local/state/agribot/c16",
    )
    arguments = parser.parse_args()
    if arguments.samples <= 0:
        parser.error("--samples必须大于0")
    if arguments.apply and arguments.target_angle_deg is None:
        parser.error("--apply必须同时提供--target-angle-deg")

    udp_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    udp_socket.bind((arguments.local_ip, arguments.listen_port))
    try:
        original = receive_device_packet(
            udp_socket, arguments.device_ip, arguments.timeout_sec
        )
        print("当前C16状态：")
        print_status(parse_device_packet(original))

        if arguments.target_angle_deg is not None:
            configured = build_config_packet(original, arguments.target_angle_deg)
            print(f"请求目标角：{arguments.target_angle_deg:.2f} deg")
            if not arguments.apply:
                print("只读预览，未发送配置；增加--apply才会修改雷达。")
                return
            save_packets(arguments.backup_directory, original, configured)
            udp_socket.sendto(
                configured, (arguments.device_ip, arguments.command_port)
            )
            expected = round(arguments.target_angle_deg * 100.0) / 100.0
            verified = False
            for _ in range(max(arguments.samples, 5)):
                packet = receive_device_packet(
                    udp_socket, arguments.device_ip, arguments.timeout_sec
                )
                status = parse_device_packet(packet)
                if abs(status["target_angle_deg"] - expected) < 0.005:
                    verified = True
                    print("配置回读成功：")
                    print_status(status)
                    break
            if not verified:
                raise RuntimeError("配置已发送，但设备包未回读到新的目标角")
            return

        for _ in range(arguments.samples - 1):
            packet = receive_device_packet(
                udp_socket, arguments.device_ip, arguments.timeout_sec
            )
            print("---")
            print_status(parse_device_packet(packet))
    finally:
        udp_socket.close()


if __name__ == "__main__":
    main()
