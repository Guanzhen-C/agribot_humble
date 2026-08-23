#!/usr/bin/env python3

import argparse
import logging
import math
import signal
import socket
import struct
import threading
import time
from datetime import date, datetime, timedelta, timezone
from typing import Optional

import serial


CHRONY_SOCK_MAGIC = 0x534F434B
CHRONY_SOCK_SAMPLE_FORMAT = "@lldiiii"
LOGGER = logging.getLogger("agribot_rtk_time")


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
    raw_date = fields[9]
    if not fields[1] or len(raw_date) != 6 or not raw_date.isdigit():
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
    except (TypeError, ValueError):
        return None
    return combine_utc_date_and_time(utc_date, fields[1])


def parse_nmea_datetime(sentence: str) -> Optional[float]:
    sentence_type = sentence.split(",", 1)[0]
    if sentence_type.endswith("RMC"):
        return parse_rmc_datetime(sentence)
    if sentence_type.endswith("ZDA"):
        return parse_zda_datetime(sentence)
    return None


def pack_chrony_sock_sample(
    measurement_time_sec: float, receipt_time_sec: float
) -> bytes:
    if not math.isfinite(measurement_time_sec) or not math.isfinite(
        receipt_time_sec
    ):
        raise ValueError("chrony sample timestamps must be finite")

    receipt_seconds = math.floor(receipt_time_sec)
    receipt_microseconds = int(round((receipt_time_sec - receipt_seconds) * 1.0e6))
    if receipt_microseconds >= 1_000_000:
        receipt_seconds += 1
        receipt_microseconds -= 1_000_000

    return struct.pack(
        CHRONY_SOCK_SAMPLE_FORMAT,
        receipt_seconds,
        receipt_microseconds,
        measurement_time_sec - receipt_time_sec,
        0,
        0,
        0,
        CHRONY_SOCK_MAGIC,
    )


class ChronySockPublisher:
    def __init__(self, socket_path: str) -> None:
        self.socket_path = socket_path
        self.socket = socket.socket(socket.AF_UNIX, socket.SOCK_DGRAM)

    def publish(self, measurement_time_sec: float, receipt_time_sec: float) -> None:
        self.socket.sendto(
            pack_chrony_sock_sample(measurement_time_sec, receipt_time_sec),
            self.socket_path,
        )

    def close(self) -> None:
        self.socket.close()


class RtkTimeFeeder:
    def __init__(
        self,
        device: str,
        baud_rate: int,
        chrony_socket: str,
        minimum_year: int,
        reconnect_interval_sec: float,
        exit_after_first_sample: bool,
    ) -> None:
        self.device = device
        self.baud_rate = baud_rate
        self.minimum_year = minimum_year
        self.reconnect_interval_sec = reconnect_interval_sec
        self.exit_after_first_sample = exit_after_first_sample
        self.publisher = ChronySockPublisher(chrony_socket)
        self.stop_event = threading.Event()
        self.last_measurement_time: Optional[float] = None
        self.last_warning_monotonic = -math.inf
        self.samples_sent = 0

    def stop(self) -> None:
        self.stop_event.set()

    def close(self) -> None:
        self.publisher.close()

    def run(self) -> int:
        while not self.stop_event.is_set():
            try:
                if self.run_serial_session():
                    return 0
            except (OSError, serial.SerialException) as exception:
                self.log_warning_limited(f"RTK TTL serial unavailable: {exception}")
            self.stop_event.wait(self.reconnect_interval_sec)
        return 0

    def run_serial_session(self) -> bool:
        with serial.Serial(
            port=self.device,
            baudrate=self.baud_rate,
            timeout=1.0,
            write_timeout=1.0,
            exclusive=True,
        ) as serial_port:
            LOGGER.info(
                "Reading RTK absolute time from %s at %d baud",
                self.device,
                self.baud_rate,
            )
            while not self.stop_event.is_set():
                raw_line = serial_port.read_until(b"\n", 1024)
                receipt_time = time.time()
                if not raw_line:
                    continue
                sentence = raw_line.decode("ascii", errors="ignore").strip()
                measurement_time = parse_nmea_datetime(sentence)
                if measurement_time is None:
                    continue
                measurement_year = datetime.fromtimestamp(
                    measurement_time, tz=timezone.utc
                ).year
                if measurement_year < self.minimum_year:
                    self.log_warning_limited(
                        f"Ignoring GNSS year {measurement_year}; minimum is {self.minimum_year}"
                    )
                    continue
                if (
                    self.last_measurement_time is not None
                    and measurement_time <= self.last_measurement_time
                ):
                    continue
                try:
                    self.publisher.publish(measurement_time, receipt_time)
                except OSError as exception:
                    self.log_warning_limited(
                        f"Chrony SOCK source is unavailable: {exception}"
                    )
                    continue

                self.last_measurement_time = measurement_time
                self.samples_sent += 1
                if self.samples_sent == 1:
                    timestamp = datetime.fromtimestamp(
                        measurement_time, tz=timezone.utc
                    ).isoformat()
                    LOGGER.info("First valid RTK time sample sent to chrony: %s", timestamp)
                if self.exit_after_first_sample:
                    return True
        return False

    def log_warning_limited(self, message: str) -> None:
        now = time.monotonic()
        if now - self.last_warning_monotonic >= 10.0:
            LOGGER.warning(message)
            self.last_warning_monotonic = now


def parse_arguments() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Feed RTK RMC/ZDA absolute time from a TTL UART to chrony"
    )
    parser.add_argument("--device", default="/dev/ttyS1")
    parser.add_argument("--baud", type=int, default=115200)
    parser.add_argument("--chrony-socket", default="/run/chrony/rtk.sock")
    parser.add_argument("--minimum-year", type=int, default=2024)
    parser.add_argument("--reconnect-interval", type=float, default=1.0)
    parser.add_argument("--once", action="store_true")
    return parser.parse_args()


def main() -> int:
    arguments = parse_arguments()
    logging.basicConfig(
        level=logging.INFO,
        format="%(asctime)s %(levelname)s %(name)s: %(message)s",
    )
    feeder = RtkTimeFeeder(
        device=arguments.device,
        baud_rate=arguments.baud,
        chrony_socket=arguments.chrony_socket,
        minimum_year=arguments.minimum_year,
        reconnect_interval_sec=arguments.reconnect_interval,
        exit_after_first_sample=arguments.once,
    )

    def request_stop(_signum, _frame) -> None:
        feeder.stop()

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)
    try:
        return feeder.run()
    finally:
        feeder.close()


if __name__ == "__main__":
    raise SystemExit(main())
