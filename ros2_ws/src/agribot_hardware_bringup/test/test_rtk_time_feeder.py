import importlib.util
import struct
from datetime import datetime, timezone
from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]
MODULE_PATH = PACKAGE / "scripts" / "rtk_time_feeder.py"
SPEC = importlib.util.spec_from_file_location("rtk_time_feeder", MODULE_PATH)
RTK_TIME = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(RTK_TIME)


def nmea(body: str) -> str:
    checksum = 0
    for character in body:
        checksum ^= ord(character)
    return f"${body}*{checksum:02X}"


def test_parses_gnss_rmc_and_zda_absolute_time():
    expected = datetime(2026, 8, 23, 10, 11, 12, 250000, tzinfo=timezone.utc)
    rmc = nmea("GNRMC,101112.250,A,0000.0000,N,00000.0000,E,0.0,0.0,230826,,,A")
    zda = nmea("GPZDA,101112.250,23,08,2026,00,00")
    assert RTK_TIME.parse_nmea_datetime(rmc) == expected.timestamp()
    assert RTK_TIME.parse_nmea_datetime(zda) == expected.timestamp()


def test_rejects_invalid_checksum_and_non_time_sentences():
    assert RTK_TIME.parse_nmea_datetime("$GNRMC,101112.0*00") is None
    assert RTK_TIME.parse_nmea_datetime(nmea("GNGGA,101112.0,,,,,0,00,99.9")) is None


def test_chrony_sock_packet_uses_native_protocol_layout():
    packet = RTK_TIME.pack_chrony_sock_sample(1787479872.25, 1787479872.30)
    unpacked = struct.unpack(RTK_TIME.CHRONY_SOCK_SAMPLE_FORMAT, packet)
    assert unpacked[0] == 1787479872
    assert unpacked[1] == 300000
    assert abs(unpacked[2] + 0.05) < 1.0e-7
    assert unpacked[-1] == RTK_TIME.CHRONY_SOCK_MAGIC


def test_system_service_and_ros_fallback_feed_the_same_chrony_source():
    service = (PACKAGE / "systemd/agribot-rtk-time.service").read_text()
    chrony = (PACKAGE / "config/time_sync/rtk-pps.conf").read_text()
    environment = (PACKAGE / "config/time_sync/rtk_time_sync.env").read_text()
    installer = (PACKAGE / "scripts/install_rtk_time_sync.sh").read_text()

    assert "/usr/local/sbin/agribot-rtk-time-feeder" in service
    assert "After=chrony.service dev-ttyS1.device" in service
    assert "RTK_TIME_SERIAL_DEVICE=/dev/ttyS1" in environment
    assert "RTK_TIME_BAUD_RATE=9600" in environment
    assert "SOCK /run/agribot-time/rtk.sock refid RTK" in chrony
    assert "PPS /dev/pps-rtk lock RTK" in chrony
    assert "/etc/chrony/conf.d/rtk-pps.conf" in installer
    assert "/etc/tmpfiles.d/agribot-time.conf" in installer
    assert "/etc/systemd/system/chrony.service.d/rtk-sock.conf" in installer
    assert "chmod 0750 /run/chrony" in installer
    assert "chmod 0751 /run/chrony" not in installer

    ackermann = yaml.safe_load((PACKAGE / "config/rtk_nmea.yaml").read_text())
    differential = yaml.safe_load(
        (PACKAGE / "differential/config/rtk_nmea.yaml").read_text()
    )
    assert ackermann["rtk_nmea"]["ros__parameters"]["enable_chrony_time_feed"]
    differential_parameters = differential["rtk_nmea"]["ros__parameters"]
    assert differential_parameters["enable_chrony_time_feed"]
    assert (
        differential_parameters["chrony_socket_path"]
        == "/run/agribot-time/rtk.sock"
    )
