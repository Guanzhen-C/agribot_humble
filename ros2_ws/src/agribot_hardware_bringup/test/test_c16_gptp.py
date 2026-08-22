from pathlib import Path

import yaml


PACKAGE = Path(__file__).resolve().parents[1]


def _assignments(path: Path) -> dict[str, str]:
    values = {}
    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.split("#", 1)[0].strip()
        if not line or line.startswith("["):
            continue
        key, value = line.split(None, 1)
        values[key] = value.strip()
    return values


def test_c16_uses_lidar_hardware_timestamp():
    config = yaml.safe_load((PACKAGE / "config/c16.yaml").read_text())
    assert config["lslidar_driver_node"]["ros__parameters"]["use_time_service"]


def test_gptp_profile_is_c16_compatible_and_uses_utc_phc():
    values = _assignments(PACKAGE / "config/time_sync/c16_gptp.cfg")
    assert values["network_transport"] == "L2"
    assert values["delay_mechanism"] == "P2P"
    assert values["transportSpecific"] == "0x1"
    assert values["time_stamping"] == "hardware"
    assert values["tx_timestamp_timeout"] == "100"
    assert values["masterOnly"] == "1"
    assert values["asCapable"] == "true"
    assert values["inhibit_delay_req"] == "1"
    assert values["utc_offset"] == "0"


def test_phc_service_does_not_apply_tai_offset():
    service = (PACKAGE / "systemd/agribot-c16-phc.service").read_text()
    assert "phc2sys" in service
    assert "-O 0" in service
    assert "-S 0.5" in service
    assert " -w" not in service


def test_gptp_check_rejects_invalid_system_phc_and_cloud_time():
    script = (PACKAGE / "scripts/check_c16_gptp.sh").read_text()
    assert "minimum_valid_epoch=1704067200" in script
    assert "maximum_phc_offset_sec=0.1" in script
    assert "maximum_cloud_offset_sec=2.0" in script


def test_gptp_service_uses_the_project_profile():
    service = (PACKAGE / "systemd/agribot-c16-gptp.service").read_text()
    assert "ptp4l" in service
    assert "/etc/agribot/c16_gptp.cfg" in service


def test_gptp_installer_restarts_existing_services():
    script = (PACKAGE / "scripts/install_c16_gptp.sh").read_text()
    assert "systemctl restart agribot-c16-phc.service" in script
    assert "systemctl restart agribot-c16-gptp.service" in script
