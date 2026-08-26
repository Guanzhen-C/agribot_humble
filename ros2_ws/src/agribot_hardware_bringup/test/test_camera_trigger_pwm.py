import importlib.util
import os
import subprocess
from pathlib import Path

import pytest
from launch import LaunchContext


PACKAGE_ROOT = Path(__file__).parents[1]
PWM_SCRIPT = PACKAGE_ROOT / "scripts" / "configure_camera_trigger_pwm.sh"
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
    (chip / "export").write_text("")
    (pwm / "period").write_text("0\n")
    (pwm / "duty_cycle").write_text("0\n")
    (pwm / "enable").write_text("0\n")
    (pwm / "polarity").write_text("normal\n")
    return pwm


def run_pwm_script(tmp_path, action):
    environment = os.environ.copy()
    environment["AGRIBOT_PWM_SYSFS_ROOT"] = str(tmp_path)
    return subprocess.run(
        ["bash", str(PWM_SCRIPT), action],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def run_pps_locked_pwm_script(tmp_path):
    helper = tmp_path / "fake_pps_lock.sh"
    helper.write_text('#!/usr/bin/env bash\nprintf "1\\n" > "$4"\n')
    helper.chmod(0o755)
    pps_device = tmp_path / "pps-rtk"
    pps_device.write_text("")
    environment = os.environ.copy()
    environment.update(
        {
            "AGRIBOT_PWM_SYSFS_ROOT": str(tmp_path),
            "PWM_PHASE_LOCK_TO_PPS": "true",
            "PWM_PPS_DEVICE": str(pps_device),
            "PWM_PPS_LOCK_HELPER": str(helper),
        }
    )
    return subprocess.run(
        ["bash", str(PWM_SCRIPT), "start"],
        check=False,
        capture_output=True,
        text=True,
        env=environment,
    )


def test_pwm_script_configures_and_stops_expected_channel(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    started = run_pwm_script(tmp_path, "start")
    assert started.returncode == 0, started.stderr
    assert (pwm / "period").read_text().strip() == "100000000"
    assert (pwm / "duty_cycle").read_text().strip() == "1000000"
    assert (pwm / "polarity").read_text().strip() == "normal"
    assert (pwm / "enable").read_text().strip() == "1"

    status = run_pwm_script(tmp_path, "status")
    assert status.returncode == 0, status.stderr
    assert "使能：1" in status.stdout

    stopped = run_pwm_script(tmp_path, "stop")
    assert stopped.returncode == 0, stopped.stderr
    assert (pwm / "enable").read_text().strip() == "0"


def test_pwm_script_can_wait_for_pps_before_enabling(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    result = run_pps_locked_pwm_script(tmp_path)
    assert result.returncode == 0, result.stderr
    assert (pwm / "enable").read_text().strip() == "1"


def trigger_context(pwm_path, **overrides):
    values = {
        "camera_driver": "hikrobot_mvs",
        "hikrobot_trigger_enable": "true",
        "hikrobot_trigger_pwm_path": str(pwm_path),
        "hikrobot_trigger_period_ns": "100000000",
        "hikrobot_trigger_duty_cycle_ns": "1000000",
    }
    values.update(overrides)
    context = LaunchContext()
    context.launch_configurations.update(values)
    return context


def test_camera_launch_accepts_ready_hardware_trigger(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    (pwm / "period").write_text("100000000\n")
    (pwm / "duty_cycle").write_text("1000000\n")
    (pwm / "enable").write_text("1\n")
    assert RIGHT_CAMERA._validate_hardware_trigger(trigger_context(pwm)) == []


def test_camera_launch_rejects_disabled_hardware_trigger(tmp_path):
    pwm = make_pwm_tree(tmp_path)
    (pwm / "period").write_text("100000000\n")
    (pwm / "duty_cycle").write_text("1000000\n")
    with pytest.raises(RuntimeError, match="PWM未就绪"):
        RIGHT_CAMERA._validate_hardware_trigger(trigger_context(pwm))


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


def test_camera_trigger_service_has_symmetric_cleanup():
    service = (
        PACKAGE_ROOT / "systemd" / "agribot-camera-trigger.service"
    ).read_text()
    assert "ExecStart=/usr/local/sbin/agribot-camera-trigger-pwm start" in service
    assert "ExecStop=/usr/local/sbin/agribot-camera-trigger-pwm stop" in service
    assert "RemainAfterExit=yes" in service
    assert "Restart=on-failure" in service

    installer = (
        PACKAGE_ROOT / "scripts" / "install_camera_trigger_pwm.sh"
    ).read_text()
    assert "systemctl restart agribot-camera-trigger.service" in installer
    assert "agribot-camera-trigger-pps-lock" in installer

    checker = (
        PACKAGE_ROOT / "scripts" / "check_camera_trigger_pwm.sh"
    ).read_text()
    assert '[[ "${phase_lock_to_pps}" == "true" ]]' in checker
    assert '[[ -c "${pps_device}" ]]' in checker
    assert '[[ -x "${pps_lock_helper}" ]]' in checker

    environment = (
        PACKAGE_ROOT / "config" / "time_sync" / "camera_trigger_pwm.env"
    ).read_text()
    assert "PWM_PHASE_LOCK_TO_PPS=true" in environment
    assert "PWM_PPS_DEVICE=/dev/pps-rtk" in environment
