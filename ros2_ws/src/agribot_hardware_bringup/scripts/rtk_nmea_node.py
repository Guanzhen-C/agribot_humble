#!/usr/bin/env python3

import base64
import math
import socket
import threading
import time
from typing import Optional

import rclpy
import serial
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import UInt8


def nmea_checksum_valid(sentence: str) -> bool:
    if not sentence.startswith("$") or "*" not in sentence:
        return False
    body, expected = sentence[1:].rsplit("*", 1)
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    try:
        return checksum == int(expected[:2], 16)
    except ValueError:
        return False


def nmea_coordinate(value: str, hemisphere: str) -> float:
    raw = float(value)
    degrees = math.floor(raw / 100.0)
    coordinate = degrees + (raw - degrees * 100.0) / 60.0
    if hemisphere in ("S", "W"):
        coordinate = -coordinate
    return coordinate


class RtkNmeaNode(Node):
    def __init__(self) -> None:
        super().__init__("rtk_nmea")
        self.serial_port = self.declare_parameter("serial_port", "/dev/ttyUSB0").value
        self.baud_rate = int(self.declare_parameter("baud_rate", 115200).value)
        self.frame_id = self.declare_parameter("frame_id", "rtk_link").value
        self.fix_topic = self.declare_parameter("fix_topic", "/rtk/fix").value
        quality_topic = self.declare_parameter(
            "quality_topic", "/rtk/fix_quality"
        ).value
        self.reconnect_interval = float(
            self.declare_parameter("reconnect_interval_sec", 1.0).value
        )
        self.fixed_std = float(
            self.declare_parameter("fixed_horizontal_std_m", 0.03).value
        )
        self.float_std = float(
            self.declare_parameter("float_horizontal_std_m", 0.30).value
        )
        self.autonomous_std = float(
            self.declare_parameter("autonomous_horizontal_std_m", 2.0).value
        )
        self.vertical_std_scale = float(
            self.declare_parameter("vertical_std_scale", 1.5).value
        )

        self.enable_ntrip = bool(self.declare_parameter("enable_ntrip", False).value)
        self.ntrip_host = self.declare_parameter("ntrip_host", "").value
        self.ntrip_port = int(self.declare_parameter("ntrip_port", 8002).value)
        self.ntrip_mountpoint = self.declare_parameter("ntrip_mountpoint", "").value
        self.ntrip_username = self.declare_parameter("ntrip_username", "").value
        self.ntrip_password = self.declare_parameter("ntrip_password", "").value
        self.ntrip_gga_period = float(
            self.declare_parameter("ntrip_gga_period_sec", 5.0).value
        )

        self.fix_publisher = self.create_publisher(NavSatFix, self.fix_topic, 10)
        self.quality_publisher = self.create_publisher(UInt8, quality_topic, 10)
        self.serial: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.receive_buffer = bytearray()
        self.latest_gga: Optional[bytes] = None
        self.last_open_attempt = 0.0
        self.stop_event = threading.Event()
        self.create_timer(0.01, self.poll_serial)

        self.ntrip_thread = None
        if self.enable_ntrip:
            required = (
                self.ntrip_host,
                self.ntrip_mountpoint,
                self.ntrip_username,
                self.ntrip_password,
            )
            if all(required):
                self.ntrip_thread = threading.Thread(
                    target=self.ntrip_loop, name="ntrip_client", daemon=True
                )
                self.ntrip_thread.start()
            else:
                self.get_logger().error(
                    "NTRIP enabled but host, mountpoint, username or password is empty"
                )

    def destroy_node(self):
        self.stop_event.set()
        self.close_serial()
        return super().destroy_node()

    def open_serial(self) -> None:
        self.last_open_attempt = time.monotonic()
        try:
            device = serial.Serial(
                self.serial_port,
                self.baud_rate,
                timeout=0,
                write_timeout=1.0,
            )
            with self.serial_lock:
                self.serial = device
            self.receive_buffer.clear()
            self.get_logger().info(
                f"RTK connected: port={self.serial_port} baud={self.baud_rate} "
                f"topic={self.fix_topic}"
            )
        except (OSError, serial.SerialException) as exception:
            self.get_logger().error(
                f"Cannot open RTK serial port {self.serial_port}: {exception}"
            )

    def close_serial(self) -> None:
        with self.serial_lock:
            device = self.serial
            self.serial = None
        if device is not None:
            try:
                device.close()
            except serial.SerialException:
                pass

    def poll_serial(self) -> None:
        if self.serial is None:
            if time.monotonic() - self.last_open_attempt >= self.reconnect_interval:
                self.open_serial()
            return

        try:
            with self.serial_lock:
                if self.serial is None:
                    return
                waiting = self.serial.in_waiting
                data = self.serial.read(waiting if waiting > 0 else 1)
        except (OSError, serial.SerialException) as exception:
            self.get_logger().error(f"RTK serial read failed: {exception}")
            self.close_serial()
            return

        if not data:
            return
        self.receive_buffer.extend(data)
        while b"\n" in self.receive_buffer:
            raw_line, _, remainder = self.receive_buffer.partition(b"\n")
            self.receive_buffer = bytearray(remainder)
            line = raw_line.strip().decode("ascii", errors="ignore")
            self.handle_sentence(line)

    def handle_sentence(self, sentence: str) -> None:
        if not nmea_checksum_valid(sentence):
            return
        fields = sentence.split("*")[0].split(",")
        if not fields or not fields[0].endswith("GGA"):
            return
        self.latest_gga = (sentence + "\r\n").encode("ascii")
        if len(fields) < 15:
            return

        try:
            quality = int(fields[6] or 0)
        except ValueError:
            return
        self.quality_publisher.publish(UInt8(data=max(0, min(quality, 255))))
        if quality == 0 or not fields[2] or not fields[4]:
            return

        try:
            latitude = nmea_coordinate(fields[2], fields[3])
            longitude = nmea_coordinate(fields[4], fields[5])
            msl_altitude = float(fields[9])
            geoid_separation = float(fields[11]) if fields[11] else 0.0
            hdop = max(float(fields[8] or 1.0), 0.1)
        except ValueError:
            return

        horizontal_std = self.horizontal_standard_deviation(quality, hdop)
        vertical_std = horizontal_std * self.vertical_std_scale

        fix = NavSatFix()
        fix.header.stamp = self.get_clock().now().to_msg()
        fix.header.frame_id = self.frame_id
        fix.status.status = (
            NavSatStatus.STATUS_GBAS_FIX
            if quality in (2, 4, 5)
            else NavSatStatus.STATUS_FIX
        )
        fix.status.service = (
            NavSatStatus.SERVICE_GPS
            | NavSatStatus.SERVICE_GLONASS
            | NavSatStatus.SERVICE_COMPASS
            | NavSatStatus.SERVICE_GALILEO
        )
        fix.latitude = latitude
        fix.longitude = longitude
        fix.altitude = msl_altitude + geoid_separation
        fix.position_covariance[0] = horizontal_std * horizontal_std
        fix.position_covariance[4] = horizontal_std * horizontal_std
        fix.position_covariance[8] = vertical_std * vertical_std
        fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN
        self.fix_publisher.publish(fix)

    def horizontal_standard_deviation(self, quality: int, hdop: float) -> float:
        if quality == 4:
            return self.fixed_std * hdop
        if quality == 5:
            return self.float_std * hdop
        return self.autonomous_std * hdop

    def ntrip_loop(self) -> None:
        while not self.stop_event.is_set():
            try:
                self.run_ntrip_session()
            except (OSError, RuntimeError) as exception:
                self.get_logger().error(f"NTRIP connection failed: {exception}")
            self.stop_event.wait(2.0)

    def run_ntrip_session(self) -> None:
        credentials = base64.b64encode(
            f"{self.ntrip_username}:{self.ntrip_password}".encode("utf-8")
        ).decode("ascii")
        mountpoint = self.ntrip_mountpoint.lstrip("/")
        request = (
            f"GET /{mountpoint} HTTP/1.0\r\n"
            "User-Agent: NTRIP agribot_ros2/0.1\r\n"
            "Accept: */*\r\n"
            f"Authorization: Basic {credentials}\r\n"
            "Connection: close\r\n\r\n"
        ).encode("ascii")

        with socket.create_connection(
            (self.ntrip_host, self.ntrip_port), timeout=5.0
        ) as connection:
            connection.settimeout(1.0)
            connection.sendall(request)
            correction_data = self.read_ntrip_header(connection)
            self.get_logger().info(
                f"NTRIP connected: {self.ntrip_host}:{self.ntrip_port}/{mountpoint}"
            )
            if correction_data:
                self.write_corrections(correction_data)

            last_gga_time = 0.0
            while not self.stop_event.is_set():
                now = time.monotonic()
                if self.latest_gga and now - last_gga_time >= self.ntrip_gga_period:
                    connection.sendall(self.latest_gga)
                    last_gga_time = now
                try:
                    correction_data = connection.recv(4096)
                except socket.timeout:
                    continue
                if not correction_data:
                    raise RuntimeError("caster closed the connection")
                self.write_corrections(correction_data)

    @staticmethod
    def read_ntrip_header(connection: socket.socket) -> bytes:
        response = bytearray()
        while len(response) < 16384:
            response.extend(connection.recv(4096))
            first_line_end = response.find(b"\r\n")
            if first_line_end < 0:
                continue
            first_line = bytes(response[:first_line_end])
            if b"200" not in first_line:
                raise RuntimeError(first_line.decode("ascii", errors="replace"))
            if first_line.startswith(b"ICY"):
                return bytes(response[first_line_end + 2:])
            header_end = response.find(b"\r\n\r\n")
            if header_end >= 0:
                return bytes(response[header_end + 4:])
        raise RuntimeError("NTRIP response header is too large")

    def write_corrections(self, data: bytes) -> None:
        write_failed = False
        try:
            with self.serial_lock:
                if self.serial is not None:
                    self.serial.write(data)
        except (OSError, serial.SerialException) as exception:
            self.get_logger().error(f"Cannot write RTCM corrections: {exception}")
            write_failed = True
        if write_failed:
            self.close_serial()


def main() -> None:
    rclpy.init()
    node = RtkNmeaNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
