import importlib.util
import os
import subprocess
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
TRIGGER_SCRIPT = PACKAGE_ROOT / "scripts" / "configure_camera_trigger_pwm.sh"
RIGHT_CAMERA_LAUNCH = PACKAGE_ROOT / "launch" / "include" / "right_camera.launch.py"


def load_right_camera_launch():
    spec = importlib.util.spec_from_file_location(
        "right_camera_launch", RIGHT_CAMERA_LAUNCH
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


RIGHT_CAMERA = load_right_camera_launch()


def make_pwm_tree(tmp_path):
    chip = tmp_path / "pwmchip0"
    pwm = chip / "pwm0"
    (chip / "device").mkdir(parents=True)
    pwm.mkdir()
    (chip / "device" / "uevent").write_text("OF_ALIAS_0=pwm3\n")
    (chip / "npwm").write_text("2\n")
    (chip / "export").write_text("")
    (pwm / "period").write_text("0\n")
    (pwm / "duty_cycle").write_text("0\n")
    (pwm / "enable").write_text("0\n")
    (pwm / "polarity").write_text("normal\n")
    return pwm


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
    return device


def make_fake_devmem(tmp_path):
    register = tmp_path / "pinmux_register"
    register.write_text("0x00F000AA\n")
    command = tmp_path / "fake_devmem.sh"
    command.write_text(
        "#!/usr/bin/env bash\n"
        "set -euo pipefail\n"
        'if [[ $# -eq 3 ]]; then printf "%s\\n" "$3" > "$AGRIBOT_FAKE_REGISTER"; fi\n'
        'cat "$AGRIBOT_FAKE_REGISTER"\n'
    )
    command.chmod(0o755)
    return register, command


def run_trigger_script(environment, action):
    return subprocess.run(
        ["bash", str(TRIGGER_SCRIPT), action],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def base_environment(tmp_path, ready_file):
    environment = os.environ.copy()
    environment.update(
        {
            "AGRIBOT_PWM_SYSFS_ROOT": str(tmp_path),
            "CAMERA_TRIGGER_READY_FILE": str(ready_file),
        }
    )
    return environment


def write_pin32_ready(path, pwm):
    path.write_text(
        "backend=pin32_pwm\n"
        f"pwm_enable_path={pwm / 'enable'}\n"
        "period_ns=100000000\n"
        "duty_cycle_ns=1000000\n"
        "polarity=normal\n"
        "pps_alignment=initial\n"
        "pps_monitoring=continuous\n"
        "initial_enable_latency_us=123.0\n"
        "pps_sequence=42\n"
        "last_pps_latency_us=123.0\n"
        f"pid={os.getpid()}\n"
    )


def set_lpwm_running(device):
    (device / "lpwm_config_info").write_text(
        "core\tsource\toffset\tperiod\tduty_time\tthreshold\tadjust_step\toccupied\n"
        "0\t6\t10\t99999\t999\t0\t0\tCAMSYS\n"
        "1\t6\t0\t0\t0\t0\t0\tCAMSYS\n"
        "2\t6\t0\t0\t0\t0\t0\tCAMSYS\n"
        "3\t6\t0\t0\t0\t0\t0\tCAMSYS\n"
    )


def write_lpwm_ready(path, device_path):
    path.write_text(
        "backend=j14_lpwm\n"
        f"device={device_path}\n"
        "channel_id=4\n"
        "channel=0\n"
        "trigger_source=6\n"
        "trigger_mode=1\n"
        "period_us=100000\n"
        "offset_us=10\n"
        "duty_us=1000\n"
        "threshold_us=0\n"
        "adjust_step=0\n"
        "pps_sequence=42\n"
        f"pid={os.getpid()}\n"
    )


def test_pin32_prepare_and_status_require_initial_alignment_and_pps_monitoring(
    tmp_path,
):
    pwm = make_pwm_tree(tmp_path)
    ready = tmp_path / "ready"
    environment = base_environment(tmp_path, ready)

    prepared = run_trigger_script(environment, "prepare")
    assert prepared.returncode == 0, prepared.stderr
    assert (pwm / "period").read_text().strip() == "100000000"
    assert (pwm / "duty_cycle").read_text().strip() == "1000000"
    assert (pwm / "enable").read_text().strip() == "0"

    (pwm / "enable").write_text("1\n")
    write_pin32_ready(ready, pwm)
    status = run_trigger_script(environment, "status")
    assert status.returncode == 0, status.stderr
    assert "40Pin物理Pin 32" in status.stdout
    assert "初始PPS启动" in status.stdout
    assert "连续监测PPS" in status.stdout

    cleaned = run_trigger_script(environment, "cleanup")
    assert cleaned.returncode == 0, cleaned.stderr
    assert (pwm / "enable").read_text().strip() == "0"
    assert not ready.exists()


def test_j14_backend_is_retained_with_official_timesync2_source(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    device = make_lpwm_tree(tmp_path)
    register, fake_devmem = make_fake_devmem(tmp_path)
    ready = tmp_path / "ready"
    environment = base_environment(tmp_path, ready)
    environment.update(
        {
            "CAMERA_TRIGGER_BACKEND": "j14_lpwm",
            "AGRIBOT_DEVMEM_COMMAND": str(fake_devmem),
            "AGRIBOT_FAKE_REGISTER": str(register),
            "LPWM_DEVICE": str(tmp_path / "fake-hobot-lpwm1"),
        }
    )
    (pwm / "enable").write_text("1\n")

    prepared = run_trigger_script(environment, "prepare")
    assert prepared.returncode == 0, prepared.stderr
    assert int(register.read_text().strip(), 0) == 0x00B000AA
    assert (pwm / "enable").read_text().strip() == "0"

    set_lpwm_running(device)
    write_lpwm_ready(ready, environment["LPWM_DEVICE"])
    status = run_trigger_script(environment, "status")
    assert status.returncode == 0, status.stderr
    assert "J14 Pin 18" in status.stdout
    assert "source 6" in status.stdout


def trigger_context(pwm_path, lpwm_device_path, ready_path, **overrides):
    values = {
        "camera_driver": "hikrobot_mvs",
        "hikrobot_trigger_enable": "true",
        "hikrobot_trigger_backend": "auto",
        "hikrobot_trigger_pwm_path": str(pwm_path),
        "hikrobot_trigger_lpwm_device_path": str(lpwm_device_path),
        "hikrobot_trigger_ready_path": str(ready_path),
        "hikrobot_trigger_period_ns": "100000000",
        "hikrobot_trigger_duty_cycle_ns": "1000000",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_camera_launch_accepts_ready_pin32_trigger(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    device = make_lpwm_tree(tmp_path)
    ready = tmp_path / "ready"
    (pwm / "period").write_text("100000000\n")
    (pwm / "duty_cycle").write_text("1000000\n")
    (pwm / "enable").write_text("1\n")
    write_pin32_ready(ready, pwm)
    assert RIGHT_CAMERA._validate_hardware_trigger(
        trigger_context(pwm, device, ready)
    ) == []


def test_camera_launch_accepts_retained_j14_trigger(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    device = make_lpwm_tree(tmp_path)
    ready = tmp_path / "ready"
    set_lpwm_running(device)
    write_lpwm_ready(ready, "/dev/hobot-lpwm1")
    assert RIGHT_CAMERA._validate_hardware_trigger(
        trigger_context(pwm, device, ready)
    ) == []


def test_camera_launch_rejects_missing_trigger_service_state(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    device = make_lpwm_tree(tmp_path)
    with pytest.raises(RuntimeError, match="触发服务未就绪"):
        RIGHT_CAMERA._validate_hardware_trigger(
            trigger_context(pwm, device, tmp_path / "missing-ready")
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


def test_service_dispatches_exclusive_trigger_backends():
    service = (
        PACKAGE_ROOT / "systemd" / "agribot-camera-trigger.service"
    ).read_text()
    assert "Type=simple" in service
    assert "ExecStart=/usr/local/sbin/agribot-camera-trigger-run" in service
    assert "ExecStartPre=/usr/local/sbin/agribot-camera-trigger-pwm prepare" in service
    assert "ExecStopPost=/usr/local/sbin/agribot-camera-trigger-pwm cleanup" in service
    assert "Restart=on-failure" in service

    runner = (PACKAGE_ROOT / "scripts" / "run_camera_trigger.sh").read_text()
    assert "pin32_pwm)" in runner
    assert "j14_lpwm)" in runner
    assert "agribot-camera-trigger-pps-lock" in runner
    assert "agribot-camera-trigger-lpwm" in runner

    installer = (
        PACKAGE_ROOT / "scripts" / "install_camera_trigger_pwm.sh"
    ).read_text()
    assert "agribot-camera-trigger-run" in installer
    assert "agribot-camera-trigger-pps-lock" in installer
    assert "agribot-camera-trigger-lpwm" in installer

    environment = (
        PACKAGE_ROOT / "config" / "time_sync" / "camera_trigger_pwm.env"
    ).read_text()
    assert "CAMERA_TRIGGER_BACKEND=pin32_pwm" in environment
    assert "LPWM_TRIGGER_SOURCE=6" in environment

    pin32_helper = (
        PACKAGE_ROOT / "time_sync" / "src" / "camera_trigger_pps_lock.cpp"
    ).read_text()
    assert "pps_alignment=initial" in pin32_helper
    assert "pps_monitoring=continuous" in pin32_helper
    assert "write_pwm_enable(pwm.get(), false)" in pin32_helper

    lpwm_helper = (
        PACKAGE_ROOT / "time_sync" / "src" / "camera_trigger_lpwm.cpp"
    ).read_text()
    assert 'output << "backend=j14_lpwm' in lpwm_helper
    assert "trigger_source{6U}" in lpwm_helper
