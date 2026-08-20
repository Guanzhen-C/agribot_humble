#!/usr/bin/env python3

import base64
import math
import socket
import threading
import time
from dataclasses import dataclass
from datetime import date, datetime, timedelta, timezone
from typing import Optional

import rclpy
import serial
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from geometry_msgs.msg import PoseWithCovarianceStamped, QuaternionStamped
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Bool, Float64, String, UInt8


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


@dataclass(frozen=True)
class GgaMetadata:
    utc_time: str
    quality: int
    satellite_count: int
    hdop: Optional[float]
    differential_age_sec: Optional[float]
    reference_station_id: str


GPS_EPOCH_UNIX_SEC = 315964800.0


def parse_utc_time_of_day(value: str) -> Optional[tuple[int, int, float]]:
    if len(value) < 6:
        return None
    try:
        hour = int(value[0:2])
        minute = int(value[2:4])
        second = float(value[4:])
    except ValueError:
        return None
    if not 0 <= hour < 24 or not 0 <= minute < 60 or not 0.0 <= second < 61.0:
        return None
    return hour, minute, second


def combine_utc_date_and_time(utc_date: date, value: str) -> Optional[float]:
    parsed = parse_utc_time_of_day(value)
    if parsed is None:
        return None
    hour, minute, second = parsed
    instant = datetime(
        utc_date.year,
        utc_date.month,
        utc_date.day,
        hour,
        minute,
        0,
        tzinfo=timezone.utc,
    ) + timedelta(seconds=second)
    return instant.timestamp()


def parse_rmc_datetime(sentence: str) -> Optional[float]:
    if not nmea_checksum_valid(sentence):
        return None
    fields = sentence.split("*", 1)[0].split(",")
    if len(fields) < 10 or not fields[0].endswith("RMC"):
        return None
    if not fields[1] or not fields[9]:
        return None
    raw_date = fields[9]
    if len(raw_date) != 6 or not raw_date.isdigit():
        return None
    day = int(raw_date[0:2])
    month = int(raw_date[2:4])
    year_two_digits = int(raw_date[4:6])
    year = 2000 + year_two_digits if year_two_digits < 80 else 1900 + year_two_digits
    try:
        utc_date = date(year, month, day)
    except ValueError:
        return None
    return combine_utc_date_and_time(utc_date, fields[1])


def parse_zda_datetime(sentence: str) -> Optional[float]:
    if not nmea_checksum_valid(sentence):
        return None
    fields = sentence.split("*", 1)[0].split(",")
    if len(fields) < 5 or not fields[0].endswith("ZDA"):
        return None
    try:
        utc_date = date(int(fields[4]), int(fields[3]), int(fields[2]))
    except (ValueError, TypeError):
        return None
    return combine_utc_date_and_time(utc_date, fields[1])


def gps_week_milliseconds_to_unix(
    gps_week: int, milliseconds: float, leap_seconds: int = 18
) -> Optional[float]:
    if gps_week < 0 or not math.isfinite(milliseconds) or milliseconds < 0.0:
        return None
    if milliseconds >= 604800000.0:
        return None
    return (
        GPS_EPOCH_UNIX_SEC
        + gps_week * 604800.0
        + milliseconds * 1.0e-3
        - leap_seconds
    )


def parse_gga_metadata(sentence: str) -> Optional[GgaMetadata]:
    """Parse quality fields without discarding a valid no-fix GGA sentence."""
    if not nmea_checksum_valid(sentence):
        return None
    fields = sentence.split("*", 1)[0].split(",")
    if len(fields) < 15 or not fields[0].endswith("GGA"):
        return None
    try:
        quality = int(fields[6] or 0)
        satellite_count = int(fields[7] or 0)
        hdop = float(fields[8]) if fields[8] else None
        differential_age_sec = float(fields[13]) if fields[13] else None
    except ValueError:
        return None
    numeric_values = (
        value for value in (hdop, differential_age_sec) if value is not None
    )
    if quality < 0 or satellite_count < 0 or not all(
        math.isfinite(value) for value in numeric_values
    ):
        return None
    return GgaMetadata(
        utc_time=fields[1],
        quality=quality,
        satellite_count=satellite_count,
        hdop=hdop,
        differential_age_sec=differential_age_sec,
        reference_station_id=fields[14],
    )


def normalize_degrees(angle_deg: float) -> float:
    return angle_deg % 360.0


def gnss_heading_to_enu_yaw(heading_deg: float) -> float:
    """Convert clockwise-from-north GNSS heading to ENU yaw radians."""
    return math.atan2(
        math.sin(math.radians(90.0 - heading_deg)),
        math.cos(math.radians(90.0 - heading_deg)),
    )


def parse_ths_sentence(sentence: str) -> Optional[tuple[Optional[float], bool]]:
    if not nmea_checksum_valid(sentence):
        return None
    fields = sentence.split("*", 1)[0].split(",")
    if len(fields) < 3 or not fields[0].endswith("THS"):
        return None
    valid = fields[2] == "A"
    if not fields[1]:
        return None, valid
    try:
        heading_deg = normalize_degrees(float(fields[1]))
    except ValueError:
        return None, False
    if not math.isfinite(heading_deg):
        return None, False
    return heading_deg, valid


def novatel_crc_valid(sentence: str) -> bool:
    if not sentence.startswith("#") or "*" not in sentence:
        return False
    body, expected = sentence[1:].rsplit("*", 1)
    crc = 0
    for byte in body.encode("ascii", errors="ignore"):
        value = (crc ^ byte) & 0xFF
        for _ in range(8):
            value = (value >> 1) ^ 0xEDB88320 if value & 1 else value >> 1
        crc = ((crc >> 8) & 0x00FFFFFF) ^ value
    try:
        return crc == int(expected[:8], 16)
    except ValueError:
        return False


@dataclass(frozen=True)
class UniHeadingSolution:
    solution_status: str
    position_type: str
    baseline_length_m: float
    heading_deg: float
    pitch_deg: float
    heading_std_deg: float
    pitch_std_deg: float
    measurement_time_unix_sec: Optional[float] = None

    @property
    def valid(self) -> bool:
        return self.solution_status == "SOL_COMPUTED" and self.position_type != "NONE"


def parse_uniheading_sentence(sentence: str) -> Optional[UniHeadingSolution]:
    if not novatel_crc_valid(sentence) or ";" not in sentence:
        return None
    header, payload_with_crc = sentence.split(";", 1)
    header_fields = header.split(",")
    if not header_fields[0].endswith("UNIHEADINGA"):
        return None
    fields = payload_with_crc.split("*", 1)[0].split(",")
    if len(fields) < 8:
        return None
    try:
        numeric = [float(fields[index]) for index in (2, 3, 4, 6, 7)]
    except ValueError:
        return None
    if not all(math.isfinite(value) for value in numeric):
        return None
    measurement_time = None
    if len(header_fields) >= 9:
        try:
            gps_week = int(header_fields[4])
            gps_milliseconds = float(header_fields[5])
            leap_seconds = int(header_fields[8])
            measurement_time = gps_week_milliseconds_to_unix(
                gps_week, gps_milliseconds, leap_seconds
            )
        except ValueError:
            measurement_time = None
    return UniHeadingSolution(
        solution_status=fields[0],
        position_type=fields[1],
        baseline_length_m=numeric[0],
        heading_deg=normalize_degrees(numeric[1]),
        pitch_deg=numeric[2],
        heading_std_deg=numeric[3],
        pitch_std_deg=numeric[4],
        measurement_time_unix_sec=measurement_time,
    )


def heading_standard_deviation_deg(
    solution: UniHeadingSolution,
    fixed_floor_deg: float,
    float_floor_deg: float,
) -> Optional[float]:
    """Return a conservative yaw standard deviation for a valid heading fix."""
    if not solution.valid:
        return None
    if solution.position_type.endswith("_INT"):
        floor_deg = fixed_floor_deg
    elif solution.position_type.endswith("_FLOAT"):
        floor_deg = float_floor_deg
    else:
        return None
    return max(solution.heading_std_deg, floor_deg)


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
        heading_topic = self.declare_parameter(
            "heading_topic", "/rtk/heading"
        ).value
        heading_deg_topic = self.declare_parameter(
            "heading_deg_topic", "/rtk/heading_deg"
        ).value
        heading_valid_topic = self.declare_parameter(
            "heading_valid_topic", "/rtk/heading_valid"
        ).value
        heading_solution_topic = self.declare_parameter(
            "heading_solution_topic", "/rtk/heading_solution"
        ).value
        heading_covariance_topic = self.declare_parameter(
            "heading_covariance_topic", "/rtk/heading_with_covariance"
        ).value
        raw_sentence_topic = self.declare_parameter(
            "raw_sentence_topic", "/rtk/raw_sentence"
        ).value
        gga_utc_topic = self.declare_parameter(
            "gga_utc_topic", "/rtk/gga_utc"
        ).value
        satellite_count_topic = self.declare_parameter(
            "satellite_count_topic", "/rtk/satellite_count"
        ).value
        hdop_topic = self.declare_parameter("hdop_topic", "/rtk/hdop").value
        differential_age_topic = self.declare_parameter(
            "differential_age_topic", "/rtk/differential_age"
        ).value
        reference_station_topic = self.declare_parameter(
            "reference_station_topic", "/rtk/reference_station_id"
        ).value
        self.heading_reference_frame = self.declare_parameter(
            "heading_reference_frame", "map"
        ).value
        self.heading_offset_deg = float(
            self.declare_parameter("heading_offset_deg", 0.0).value
        )
        self.fixed_heading_std_floor_deg = float(
            self.declare_parameter("fixed_heading_std_floor_deg", 1.0).value
        )
        self.float_heading_std_floor_deg = float(
            self.declare_parameter("float_heading_std_floor_deg", 5.0).value
        )
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
        self.use_gnss_measurement_time = bool(
            self.declare_parameter("use_gnss_measurement_time", True).value
        )
        self.gnss_time_offset_sec = float(
            self.declare_parameter("gnss_time_offset_sec", 0.0).value
        )
        self.gnss_time_max_error_sec = float(
            self.declare_parameter("gnss_time_max_error_sec", 5.0).value
        )
        self.heading_pair_tolerance_sec = float(
            self.declare_parameter("heading_pair_tolerance_sec", 0.25).value
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
        self.heading_publisher = self.create_publisher(
            QuaternionStamped, heading_topic, 10
        )
        self.heading_deg_publisher = self.create_publisher(
            Float64, heading_deg_topic, 10
        )
        self.heading_valid_publisher = self.create_publisher(
            Bool, heading_valid_topic, 10
        )
        self.heading_solution_publisher = self.create_publisher(
            String, heading_solution_topic, 10
        )
        self.heading_covariance_publisher = self.create_publisher(
            PoseWithCovarianceStamped, heading_covariance_topic, 10
        )
        self.raw_sentence_publisher = self.create_publisher(
            String, raw_sentence_topic, 100
        )
        self.gga_utc_publisher = self.create_publisher(String, gga_utc_topic, 10)
        self.satellite_count_publisher = self.create_publisher(
            UInt8, satellite_count_topic, 10
        )
        self.hdop_publisher = self.create_publisher(Float64, hdop_topic, 10)
        self.differential_age_publisher = self.create_publisher(
            Float64, differential_age_topic, 10
        )
        self.reference_station_publisher = self.create_publisher(
            String, reference_station_topic, 10
        )
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, "/diagnostics", 10
        )
        self.serial: Optional[serial.Serial] = None
        self.serial_lock = threading.Lock()
        self.receive_buffer = bytearray()
        self.latest_gga: Optional[bytes] = None
        self.last_open_attempt = 0.0
        self.stop_event = threading.Event()
        self.latest_utc_date: Optional[date] = None
        self.latest_fix_measurement_sec: Optional[float] = None
        self.latest_fix_receipt_monotonic: Optional[float] = None
        self.last_time_source = "receipt"
        self.last_time_error_sec = math.nan
        self.gnss_time_reject_count = 0
        self.create_timer(0.01, self.poll_serial)
        self.create_timer(1.0, self.publish_time_diagnostics)

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
        if not sentence:
            return
        self.raw_sentence_publisher.publish(String(data=sentence))
        if sentence.startswith("#"):
            self.handle_uniheading(sentence)
            return
        if not nmea_checksum_valid(sentence):
            return
        fields = sentence.split("*")[0].split(",")
        if not fields:
            return
        if fields[0].endswith("RMC"):
            self.update_absolute_gnss_time(parse_rmc_datetime(sentence))
            return
        if fields[0].endswith("ZDA"):
            self.update_absolute_gnss_time(parse_zda_datetime(sentence))
            return
        if fields[0].endswith("THS"):
            self.handle_ths(sentence)
            return
        if not fields[0].endswith("GGA"):
            return
        self.latest_gga = (sentence + "\r\n").encode("ascii")
        metadata = parse_gga_metadata(sentence)
        if metadata is None:
            return

        quality = metadata.quality
        self.quality_publisher.publish(UInt8(data=max(0, min(quality, 255))))
        self.gga_utc_publisher.publish(String(data=metadata.utc_time))
        self.satellite_count_publisher.publish(
            UInt8(data=max(0, min(metadata.satellite_count, 255)))
        )
        self.hdop_publisher.publish(
            Float64(data=metadata.hdop if metadata.hdop is not None else math.nan)
        )
        self.differential_age_publisher.publish(
            Float64(
                data=(
                    metadata.differential_age_sec
                    if metadata.differential_age_sec is not None
                    else math.nan
                )
            )
        )
        self.reference_station_publisher.publish(
            String(data=metadata.reference_station_id)
        )
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

        receipt_time = self.get_clock().now()
        measurement_time = self.gga_measurement_time(
            metadata.utc_time, receipt_time.nanoseconds * 1.0e-9
        )
        stamp_sec = self.accept_measurement_time(measurement_time, receipt_time)
        self.latest_fix_measurement_sec = stamp_sec
        self.latest_fix_receipt_monotonic = time.monotonic()

        fix = NavSatFix()
        fix.header.stamp = rclpy.time.Time(
            nanoseconds=int(round(stamp_sec * 1.0e9))
        ).to_msg()
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

    def handle_ths(self, sentence: str) -> None:
        parsed = parse_ths_sentence(sentence)
        if parsed is None:
            return
        heading_deg, valid = parsed
        self.heading_valid_publisher.publish(Bool(data=valid))
        if not valid or heading_deg is None:
            return

        vehicle_heading_deg = normalize_degrees(
            heading_deg + self.heading_offset_deg
        )
        yaw = gnss_heading_to_enu_yaw(vehicle_heading_deg)
        receipt_time = self.get_clock().now()
        stamp_sec = receipt_time.nanoseconds * 1.0e-9
        if (
            self.latest_fix_measurement_sec is not None
            and self.latest_fix_receipt_monotonic is not None
            and time.monotonic() - self.latest_fix_receipt_monotonic
            <= self.heading_pair_tolerance_sec
        ):
            stamp_sec = self.latest_fix_measurement_sec
        heading = QuaternionStamped()
        heading.header.stamp = rclpy.time.Time(
            nanoseconds=int(round(stamp_sec * 1.0e9))
        ).to_msg()
        heading.header.frame_id = self.heading_reference_frame
        heading.quaternion.z = math.sin(yaw / 2.0)
        heading.quaternion.w = math.cos(yaw / 2.0)
        self.heading_publisher.publish(heading)
        self.heading_deg_publisher.publish(Float64(data=vehicle_heading_deg))

    def handle_uniheading(self, sentence: str) -> None:
        solution = parse_uniheading_sentence(sentence)
        if solution is None:
            return
        self.heading_solution_publisher.publish(
            String(data=f"{solution.solution_status},{solution.position_type}")
        )
        heading_std_deg = heading_standard_deviation_deg(
            solution,
            self.fixed_heading_std_floor_deg,
            self.float_heading_std_floor_deg,
        )
        if heading_std_deg is None:
            return

        vehicle_heading_deg = normalize_degrees(
            solution.heading_deg + self.heading_offset_deg
        )
        yaw = gnss_heading_to_enu_yaw(vehicle_heading_deg)
        receipt_time = self.get_clock().now()
        stamp_sec = self.accept_measurement_time(
            solution.measurement_time_unix_sec, receipt_time
        )
        self.update_absolute_gnss_time(solution.measurement_time_unix_sec)
        heading = PoseWithCovarianceStamped()
        heading.header.stamp = rclpy.time.Time(
            nanoseconds=int(round(stamp_sec * 1.0e9))
        ).to_msg()
        heading.header.frame_id = self.heading_reference_frame
        heading.pose.pose.orientation.z = math.sin(yaw / 2.0)
        heading.pose.pose.orientation.w = math.cos(yaw / 2.0)
        for index in (0, 7, 14, 21, 28):
            heading.pose.covariance[index] = 1e6
        heading.pose.covariance[35] = math.radians(heading_std_deg) ** 2
        self.heading_covariance_publisher.publish(heading)

    def update_absolute_gnss_time(self, measurement_time: Optional[float]) -> None:
        if measurement_time is None or not math.isfinite(measurement_time):
            return
        self.latest_utc_date = datetime.fromtimestamp(
            measurement_time, tz=timezone.utc
        ).date()

    def gga_measurement_time(
        self, utc_time: str, receipt_time_sec: float
    ) -> Optional[float]:
        receipt_date = datetime.fromtimestamp(
            receipt_time_sec, tz=timezone.utc
        ).date()
        base_date = self.latest_utc_date or receipt_date
        candidates = []
        for day_offset in (-1, 0, 1):
            candidate = combine_utc_date_and_time(
                base_date + timedelta(days=day_offset), utc_time
            )
            if candidate is not None:
                candidates.append(candidate)
        if not candidates:
            return None
        return min(candidates, key=lambda value: abs(value - receipt_time_sec))

    def accept_measurement_time(
        self, measurement_time: Optional[float], receipt_time
    ) -> float:
        receipt_sec = receipt_time.nanoseconds * 1.0e-9
        if not self.use_gnss_measurement_time:
            self.last_time_source = "receipt_disabled"
            self.last_time_error_sec = math.nan
            return receipt_sec
        if measurement_time is None or not math.isfinite(measurement_time):
            self.last_time_source = "receipt_missing_gnss"
            self.last_time_error_sec = math.nan
            return receipt_sec
        corrected_time = measurement_time + self.gnss_time_offset_sec
        self.last_time_error_sec = receipt_sec - corrected_time
        if abs(self.last_time_error_sec) > self.gnss_time_max_error_sec:
            self.gnss_time_reject_count += 1
            self.last_time_source = "receipt_invalid_gnss"
            return receipt_sec
        self.last_time_source = "gnss_measurement"
        return corrected_time

    def publish_time_diagnostics(self) -> None:
        message = DiagnosticArray()
        message.header.stamp = self.get_clock().now().to_msg()
        status = DiagnosticStatus()
        status.name = "rtk_nmea/time_sync"
        status.hardware_id = self.serial_port
        if self.last_time_source == "gnss_measurement":
            status.level = DiagnosticStatus.OK
            status.message = "using GNSS measurement time"
        elif self.last_time_source == "receipt_invalid_gnss":
            status.level = DiagnosticStatus.ERROR
            status.message = "GNSS time differs from RDK clock; using receipt time"
        else:
            status.level = DiagnosticStatus.WARN
            status.message = "using serial receipt time"
        status.values = [
            KeyValue(key="source", value=self.last_time_source),
            KeyValue(
                key="measurement_to_receipt_ms",
                value=(
                    f"{self.last_time_error_sec * 1.0e3:.3f}"
                    if math.isfinite(self.last_time_error_sec)
                    else "nan"
                ),
            ),
            KeyValue(
                key="rejected_measurement_times",
                value=str(self.gnss_time_reject_count),
            ),
            KeyValue(
                key="utc_date_known",
                value="true" if self.latest_utc_date is not None else "false",
            ),
        ]
        message.status = [status]
        self.diagnostics_publisher.publish(message)

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
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
