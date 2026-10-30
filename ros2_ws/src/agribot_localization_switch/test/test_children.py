import subprocess
import sys

import pytest

from agribot_localization_switch.children import EstimatorProcess


def test_estimator_launch_is_whitelisted_and_cleanup_reaps_process(monkeypatch, tmp_path):
    real_popen = subprocess.Popen
    commands = []

    def fake_estimator(command, **kwargs):
        commands.append(command)
        # Exercise real process-group cleanup without ROS drivers or hardware.
        return real_popen([sys.executable, "-c", "import time; time.sleep(60)"], **kwargs)

    monkeypatch.setattr(subprocess, "Popen", fake_estimator)
    process = EstimatorProcess("fastlivo", tmp_path / "map", tmp_path / "logs")
    try:
        process.start()
        child = process.process
        assert process.alive()
        with pytest.raises(RuntimeError):
            process.start()
        assert commands[0][:7] == ["nice", "-n", "5", "ros2", "launch", "agribot_localization_switch", "estimator_backend.launch.py"]
        assert "estimator:=fastlivo" in commands[0]
    finally:
        process.stop()
    assert child.poll() is not None
    assert not process.alive()
    process.stop()


def test_unknown_backend_cannot_run_an_arbitrary_launch(tmp_path):
    with pytest.raises(ValueError):
        EstimatorProcess("sensors", tmp_path / "map", tmp_path / "logs")
