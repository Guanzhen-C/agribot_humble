import importlib.util
import os
import subprocess
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
LPWM_SCRIPT = PACKAGE_ROOT / "scripts" / "configure_camera_trigger_pwm.sh"
RIGHT_CAMERA_LAUNCH = PACKAGE_ROOT / "launch" / "include" / "right_camera.launch.py"


def load_right_camera_launch():
    spec = importlib.util.spec_from_file_location(
        "right_camera_launch", RIGHT_CAMERA_LAUNCH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RIGHT_CAMERA = load_right_camera_launch()


def make_lpwm_tree(tmp_path):
    chip = tmp_path / "pwmchip2"
    device = chip / "device"
    device.mkdir(parents=True)
    (device / "uevent").write_text(
        "DRIVER=hobot-lpwm\nOF_ALIAS_0=lpwm1\n"
    )
    (device / "lpwm_config_info").write_text(
        "core\tsource\toffset\tperiod\tduty_time\tthreshold\tadjust_step\toccupied\n"
        "0\t0\t0\t0\t0\t0\t0\tNONE\n"
        "1\t0\t0\t0\t0\t0\t0\tNONE\n"
        "2\t0\t0\t0\t0\t0\t0\tNONE\n"
        "3\t0\t0\t0\t0\t0\t0\tNONE\n"
    )
    (chip / "npwm").write_text("4\n")

    register = tmp_path / "pinmux_register"
    register.write_text("0x00F000AA\n")
    fake_devmem = tmp_path / "fake_devmem.sh"
    fake_devmem.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        "if [[ $# -eq 3 ]]; then printf '%s\\n' \"$3\" > \"$AGRIBOT_FAKE_REGISTER\"; fi\n"
        "cat \"$AGRIBOT_FAKE_REGISTER\"\n"
    )
    fake_devmem.chmod(0o755)

    legacy = tmp_path / "legacy_enable"
    legacy.write_text("1\n")
    return device, register, fake_devmem, legacy


def lpwm_environment(tmp_path, register, fake_devmem, legacy, ready_file):
    environment = os.environ.copy()
    environment.update(
        {
            "AGRIBOT_PWM_SYSFS_ROOT": str(tmp_path),
            "AGRIBOT_DEVMEM_COMMAND": str(fake_devmem),
            "AGRIBOT_FAKE_REGISTER": str(register),
            "LPWM_DEVICE": str(tmp_path / "fake-hobot-lpwm1"),
            "LPWM_READY_FILE": str(ready_file),
            "LPWM_LEGACY_PWM_ENABLE_PATH": str(legacy),
        }
    )
    return environment


def run_lpwm_script(environment, action):
    return subprocess.run(
        ["bash", str(LPWM_SCRIPT), action],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def write_ready_file(path, device):
    path.write_text(
        f"device={device}\n"
        "channel_id=4\n"
        "channel=0\n"
        "trigger_source=2\n"
        "trigger_mode=1\n"
        "period_us=100000\n"
        "offset_us=10\n"
        "duty_us=1000\n"
        "threshold_us=0\n"
        "adjust_step=0\n"
        "pps_sequence=42\n"
        f"pid={os.getpid()}\n"
    )


def set_running_config(device):
    (device / "lpwm_config_info").write_text(
        "core\tsource\toffset\tperiod\tduty_time\tthreshold\tadjust_step\toccupied\n"
        "0\t2\t10\t99999\t999\t0\t0\tCAMSYS\n"
        "1\t2\t0\t0\t0\t0\t0\tNONE\n"
        "2\t2\t0\t0\t0\t0\t0\tNONE\n"
        "3\t2\t0\t0\t0\t0\t0\tNONE\n"
    )


def test_lpwm_prepare_selects_timesync2_and_disables_legacy_pwm(tmp_path):
    _, register, fake_devmem, legacy = make_lpwm_tree(tmp_path)
    ready_file = tmp_path / "ready"
    environment = lpwm_environment(
        tmp_path, register, fake_devmem, legacy, ready_file
    )

    result = run_lpwm_script(environment, "prepare")
    assert result.returncode == 0, result.stderr
    assert int(register.read_text().strip(), 0) == 0x00B000AA
    assert legacy.read_text().strip() == "0"
    assert "Pin 33" in result.stdout


def test_lpwm_status_checks_driver_and_live_process(tmp_path):
    device, register, fake_devmem, legacy = make_lpwm_tree(tmp_path)
    ready_file = tmp_path / "ready"
    environment = lpwm_environment(
        tmp_path, register, fake_devmem, legacy, ready_file
    )
    prepared = run_lpwm_script(environment, "prepare")
    assert prepared.returncode == 0, prepared.stderr
    set_running_config(device)
    write_ready_file(ready_file, environment["LPWM_DEVICE"])

    status = run_lpwm_script(environment, "status")
    assert status.returncode == 0, status.stderr
    assert "J14 Pin 18" in status.stdout
    assert "TIME_SYNC2/SGT1" in status.stdout


def trigger_context(lpwm_device_path, ready_path, **overrides):
    values = {
        "camera_driver": "hikrobot_mvs",
        "hikrobot_trigger_enable": "true",
        "hikrobot_trigger_lpwm_device_path": str(lpwm_device_path),
        "hikrobot_trigger_ready_path": str(ready_path),
        "hikrobot_trigger_period_ns": "100000000",
        "hikrobot_trigger_duty_cycle_ns": "1000000",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_camera_launch_accepts_ready_lpwm_hardware_trigger(tmp_path):
    device, _, _, _ = make_lpwm_tree(tmp_path)
    set_running_config(device)
    ready = tmp_path / "ready"
    write_ready_file(ready, "/dev/hobot-lpwm1")
    assert RIGHT_CAMERA._validate_hardware_trigger(
        trigger_context(device, ready)
    ) == []


def test_camera_launch_rejects_missing_lpwm_service_state(tmp_path):
    device, _, _, _ = make_lpwm_tree(tmp_path)
    with pytest.raises(RuntimeError, match="无法读取RDK LPWM状态"):
        RIGHT_CAMERA._validate_hardware_trigger(
            trigger_context(device, tmp_path / "missing-ready")
        )


def test_physical_ackermann_camera_defaults_to_hardware_trigger():
    launch_files = (
        "ackermann_fastlivo_rtk_localization.launch.py",
        "ackermann_mppi_fastlivo_rtk_mapped.launch.py",
        "ackermann_outdoor_0811_experiment.launch.py",
        "ackermann_sensor_data_collection.launch.py",
    )
    for name in launch_files:
        source = (PACKAGE_ROOT / "ackermann" / "launch" / name).read_text()
        assert '"hikrobot_trigger_enable", default_value="true"' in source


def test_physical_differential_camera_defaults_to_hardware_trigger():
    launch_files = (
        "differential_fastlivo_rtk_localization.launch.py",
        "differential_mppi_fastlivo_rtk_mapped.launch.py",
        "differential_outdoor_experiment.launch.py",
        "differential_sensor_data_collection.launch.py",
        "differential_sensor_validation.launch.py",
        "differential_3d_mapping.launch.py",
    )
    for name in launch_files:
        source = (PACKAGE_ROOT / "differential" / "launch" / name).read_text()
        assert '"hikrobot_trigger_enable", default_value="true"' in source
        assert '"lidar_forward_point_offset_sec", default_value="0.05004"' in source


def test_camera_trigger_service_uses_continuous_hardware_pps_retrigger():
    service = (
        PACKAGE_ROOT / "systemd" / "agribot-camera-trigger.service"
    ).read_text()
    assert "Type=simple" in service
    assert "camera-trigger-lpwm" in service
    assert "--trigger-source ${LPWM_TRIGGER_SOURCE}" in service
    assert "RemainAfterExit" not in service
    assert "Restart=on-failure" in service

    installer = (
        PACKAGE_ROOT / "scripts" / "install_camera_trigger_pwm.sh"
    ).read_text()
    assert "systemctl stop agribot-camera-trigger.service" in installer
    assert "agribot-camera-trigger-lpwm" in installer
    assert "agribot-camera-trigger-pps-lock" not in installer

    checker = (
        PACKAGE_ROOT / "scripts" / "check_camera_trigger_pwm.sh"
    ).read_text()
    assert 'LPWM_DEVICE:-/dev/hobot-lpwm1' in checker
    assert "TIME_SYNC2" in checker
    assert "sudo /usr/local/sbin/agribot-camera-trigger-pwm status" in checker

    configurator = LPWM_SCRIPT.read_text()
    assert '[[ -d "/proc/${pid}" ]]' in configurator
    assert 'kill -0 "${pid}"' not in configurator

    environment = (
        PACKAGE_ROOT / "config" / "time_sync" / "camera_trigger_pwm.env"
    ).read_text()
    assert "LPWM_TRIGGER_SOURCE=2" in environment
    assert "LPWM_TIME_SYNC_PINMUX_SHIFT=22" in environment
    assert "LPWM_PERIOD_US=100000" in environment
    assert "LPWM_PPS_DEVICE=/dev/pps-rtk" in environment

    helper = (
        PACKAGE_ROOT / "time_sync" / "src" / "camera_trigger_lpwm.cpp"
    ).read_text()
    assert "kHardwareTriggerMode = 1" in helper
    assert "kLpwmInit == 0x40784c12UL" in helper
    assert "kLpwmClose == 0x40044c13UL" in helper
    assert "RTK PPS序号在超时时间内没有递增" in helper

    camera_launch = RIGHT_CAMERA_LAUNCH.read_text()
    assert 'Path(f"/proc/{process_id}").is_dir()' in camera_launch
