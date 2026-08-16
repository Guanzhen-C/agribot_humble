import os
from pathlib import Path
import time

import pytest

from agribot_mobile_app.processes import ManagedProcess, ProcessError, ProcessSlots


def fake_ros2(tmp_path: Path):
    executable = tmp_path / "ros2"
    executable.write_text("#!/bin/sh\nprintf 'started %s\\n' \"$*\"\nexit 0\n")
    executable.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
    return environment


def test_managed_process_records_output_and_completion(tmp_path):
    process = ManagedProcess("测试")
    log = tmp_path / "process.log"
    process.start(["ros2", "launch", "package", "file.launch.py"], log, fake_ros2(tmp_path))
    deadline = time.monotonic() + 3.0
    while process.running and time.monotonic() < deadline:
        time.sleep(0.01)
    snapshot = process.snapshot()
    assert snapshot["state"] == "completed"
    assert snapshot["return_code"] == 0
    assert "started launch package file.launch.py" in log.read_text()


def test_rejects_non_ros_commands(tmp_path):
    with pytest.raises(ProcessError, match="只允许"):
        ManagedProcess("测试").start(["bash", "-c", "true"], tmp_path / "log")


def test_slots_prevent_conflicting_tasks(tmp_path):
    slots = ProcessSlots()
    slots.runtime._state = "running"
    slots.runtime._process = type("Process", (), {"poll": lambda self: None})()
    with pytest.raises(ProcessError, match="停止当前"):
        slots.assert_exclusive("collection")
