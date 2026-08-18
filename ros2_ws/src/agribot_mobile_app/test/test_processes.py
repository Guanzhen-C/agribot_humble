import os
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest

from agribot_mobile_app.processes import ManagedProcess, ProcessError, ProcessSlots
from agribot_mobile_app.gateway_node import MobileGateway


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


def test_stop_terminates_the_entire_process_group(tmp_path):
    child_pid_file = tmp_path / "child.pid"
    executable = tmp_path / "ros2"
    executable.write_text(
        "#!/usr/bin/env python3\n"
        "import os\n"
        "from pathlib import Path\n"
        "import subprocess\n"
        "child = subprocess.Popen(['sleep', '60'])\n"
        "Path(os.environ['CHILD_PID_FILE']).write_text(str(child.pid))\n"
        "child.wait()\n"
    )
    executable.chmod(0o755)
    environment = os.environ.copy()
    environment["PATH"] = f"{tmp_path}:{environment['PATH']}"
    environment["CHILD_PID_FILE"] = str(child_pid_file)

    process = ManagedProcess("进程组测试")
    process.start(["ros2", "launch", "package", "test.launch.py"], tmp_path / "log", environment)
    deadline = time.monotonic() + 3.0
    while not child_pid_file.is_file() and time.monotonic() < deadline:
        time.sleep(0.01)
    assert child_pid_file.is_file()

    process.stop(timeout=0.5)
    first_snapshot = process.snapshot()
    assert first_snapshot["state"] == "stopped"
    assert not ManagedProcess._group_alive(first_snapshot["process_group"])

    process.start(
        ["ros2", "launch", "package", "test.launch.py"],
        tmp_path / "second.log",
        environment,
    )
    assert process.snapshot()["pid"] != first_snapshot["pid"]
    process.stop(timeout=0.5)
    assert process.snapshot()["state"] == "stopped"


def test_slots_stop_motion_before_other_jobs():
    slots = ProcessSlots()
    stopped = []
    slots.runtime.stop = lambda: stopped.append("runtime")
    slots.collection.stop = lambda: stopped.append("collection")
    slots.processing.stop = lambda: stopped.append("processing")

    slots.stop_all()
    assert stopped == ["runtime", "collection", "processing"]


def test_stopping_runtime_clears_stale_navigation_state():
    gateway = object.__new__(MobileGateway)
    gateway._task_transition_lock = threading.RLock()
    gateway._lock = threading.RLock()
    gateway._goal_handle = object()
    gateway._state = {
        "active_runtime": {"profile_id": "test", "map_id": "map_test"},
        "semantic": {"status": "ready"},
        "navigation": {
            "kind": "semantic",
            "status": "canceling",
            "feedback": {"distance_remaining": 1.0},
            "goal": {"x": 1.0, "y": 2.0, "yaw": 0.0},
            "route": [{"x": 0.0, "y": 0.0, "yaw": 0.0}],
        },
    }
    gateway.cancel_navigation = lambda _body: {"status": "canceling"}
    gateway.processes = SimpleNamespace(runtime=SimpleNamespace(stop=lambda: None))
    gateway._empty_semantic_state = lambda: {"status": "idle"}
    gateway._touch = lambda: None

    result = gateway.stop_runtime({})

    assert result["runtime"]["map_id"] == "map_test"
    assert gateway._goal_handle is None
    assert gateway._state["active_runtime"] is None
    assert gateway._state["semantic"] == {"status": "idle"}
    assert gateway._state["navigation"] == {
        "kind": None,
        "status": "idle",
        "feedback": {},
        "goal": None,
        "route": [],
    }
