"""Guarded process slots for whitelisted ROS launch and offline jobs."""

from __future__ import annotations

from collections import deque
import os
from pathlib import Path
import signal
import subprocess
import threading
import time
from typing import Callable, Iterable


class ProcessError(RuntimeError):
    """Raised when a managed process cannot be started or stopped safely."""


class ManagedProcess:
    def __init__(self, name: str, on_change: Callable[[], None] | None = None):
        self.name = name
        self._on_change = on_change or (lambda: None)
        self._lock = threading.RLock()
        self._process: subprocess.Popen | None = None
        self._log_stream = None
        self._log_path: Path | None = None
        self._command: list[str] = []
        self._started_at: float | None = None
        self._ended_at: float | None = None
        self._return_code: int | None = None
        self._state = "idle"
        self._tail: deque[str] = deque(maxlen=80)

    @property
    def running(self) -> bool:
        with self._lock:
            return self._process is not None and self._state in (
                "running",
                "stopping",
            )

    def start(
        self,
        command: Iterable[str],
        log_path: Path,
        environment: dict[str, str] | None = None,
    ) -> None:
        command_list = [str(value) for value in command]
        if not command_list or command_list[0] not in ("ros2", "/usr/bin/ros2"):
            raise ProcessError("只允许启动预定义的ros2命令")
        with self._lock:
            if self.running:
                raise ProcessError(f"{self.name}已经在运行")
            log_path.parent.mkdir(parents=True, exist_ok=True)
            self._log_stream = log_path.open("a", encoding="utf-8", buffering=1)
            self._log_path = log_path
            self._command = command_list
            self._started_at = time.time()
            self._ended_at = None
            self._return_code = None
            self._state = "running"
            self._tail.clear()
            try:
                self._process = subprocess.Popen(
                    command_list,
                    env=environment,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.STDOUT,
                    text=True,
                    bufsize=1,
                    start_new_session=True,
                )
            except OSError as error:
                self._log_stream.close()
                self._log_stream = None
                self._state = "failed"
                raise ProcessError(f"无法启动{self.name}: {error}") from error
            threading.Thread(target=self._monitor, daemon=True).start()
        self._on_change()

    def _monitor(self) -> None:
        process = self._process
        if process is None:
            return
        if process.stdout is not None:
            for line in process.stdout:
                clean = line.rstrip()
                with self._lock:
                    self._tail.append(clean)
                    if self._log_stream is not None:
                        self._log_stream.write(line)
                # The gateway publishes a periodic state snapshot. Waking every
                # phone connection for every ROS log line can saturate the RDK.
        return_code = process.wait()
        with self._lock:
            self._return_code = return_code
            self._ended_at = time.time()
            self._state = "completed" if return_code == 0 else "failed"
            if self._log_stream is not None:
                self._log_stream.close()
                self._log_stream = None
        self._on_change()

    def stop(self, timeout: float = 35.0) -> None:
        with self._lock:
            process = self._process
            if process is None or process.poll() is not None:
                return
            self._state = "stopping"
            process_group = os.getpgid(process.pid)
        self._on_change()
        os.killpg(process_group, signal.SIGINT)
        try:
            process.wait(timeout=timeout)
        except subprocess.TimeoutExpired:
            os.killpg(process_group, signal.SIGTERM)
            try:
                process.wait(timeout=5.0)
            except subprocess.TimeoutExpired:
                os.killpg(process_group, signal.SIGKILL)
                process.wait(timeout=5.0)

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "name": self.name,
                "state": self._state,
                "running": self.running,
                "started_at": self._started_at,
                "ended_at": self._ended_at,
                "return_code": self._return_code,
                "log_path": str(self._log_path) if self._log_path else None,
                "tail": list(self._tail)[-12:],
            }


class ProcessSlots:
    def __init__(self, on_change: Callable[[], None] | None = None):
        self.runtime = ManagedProcess("导航运行栈", on_change)
        self.collection = ManagedProcess("传感器数据采集", on_change)
        self.processing = ManagedProcess("离线地图处理", on_change)

    def stop_all(self) -> None:
        for process in (self.collection, self.processing, self.runtime):
            process.stop()

    def snapshot(self) -> dict:
        return {
            "runtime": self.runtime.snapshot(),
            "collection": self.collection.snapshot(),
            "processing": self.processing.snapshot(),
        }

    def assert_exclusive(self, requested: str) -> None:
        for name, process in (
            ("runtime", self.runtime),
            ("collection", self.collection),
            ("processing", self.processing),
        ):
            if name != requested and process.running:
                raise ProcessError("必须先停止当前运行中的任务")
