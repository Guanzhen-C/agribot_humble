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
        self._process_group: int | None = None
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
            log_stream = log_path.open("a", encoding="utf-8", buffering=1)
            self._log_path = log_path
            self._command = command_list
            self._started_at = time.time()
            self._ended_at = None
            self._return_code = None
            self._state = "running"
            self._tail.clear()
            self._process = None
            self._process_group = None
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
                self._process_group = self._process.pid
            except OSError as error:
                log_stream.close()
                self._state = "failed"
                raise ProcessError(f"无法启动{self.name}: {error}") from error
            threading.Thread(
                target=self._monitor,
                args=(self._process, log_stream),
                daemon=True,
            ).start()
        self._on_change()

    def _monitor(self, process: subprocess.Popen, log_stream) -> None:
        if process.stdout is not None:
            for line in process.stdout:
                clean = line.rstrip()
                with self._lock:
                    if self._process is process:
                        self._tail.append(clean)
                log_stream.write(line)
                # The gateway publishes a periodic state snapshot. Waking every
                # phone connection for every ROS log line can saturate the RDK.
        return_code = process.wait()
        log_stream.close()
        with self._lock:
            if self._process is not process:
                return
            self._return_code = return_code
            self._ended_at = time.time()
            if self._state == "stopping":
                self._state = "stopped"
            else:
                self._state = "completed" if return_code == 0 else "failed"
        self._on_change()

    @staticmethod
    def _group_alive(process_group: int) -> bool:
        """Return whether a process group still has a non-zombie member."""
        proc_root = Path("/proc")
        try:
            for stat_path in proc_root.glob("[0-9]*/stat"):
                try:
                    fields = (
                        stat_path.read_text(encoding="utf-8")
                        .rsplit(")", 1)[1]
                        .split()
                    )
                    state = fields[0]
                    group = int(fields[2])
                except (IndexError, OSError, ValueError):
                    continue
                if group == process_group and state != "Z":
                    return True
            return False
        except OSError:
            try:
                os.killpg(process_group, 0)
                return True
            except ProcessLookupError:
                return False
            except PermissionError:
                return True

    @classmethod
    def _wait_group_exit(cls, process_group: int, timeout: float) -> bool:
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            if not cls._group_alive(process_group):
                return True
            time.sleep(0.05)
        return not cls._group_alive(process_group)

    @staticmethod
    def _signal_group(process_group: int, requested_signal: signal.Signals) -> None:
        try:
            os.killpg(process_group, requested_signal)
        except ProcessLookupError:
            pass

    def stop(self, timeout: float = 35.0) -> None:
        already_finished = False
        with self._lock:
            process = self._process
            process_group = self._process_group
            if process is None or process_group is None:
                return
            if process.poll() is not None and not self._group_alive(process_group):
                if self._state in ("running", "stopping"):
                    self._return_code = process.returncode
                    self._ended_at = time.time()
                    self._state = (
                        "completed" if process.returncode == 0 else "stopped"
                    )
                already_finished = True
            else:
                self._state = "stopping"
        self._on_change()
        if already_finished:
            return
        for requested_signal, wait_timeout in (
            (signal.SIGINT, timeout),
            (signal.SIGTERM, 5.0),
            (signal.SIGKILL, 5.0),
        ):
            self._signal_group(process_group, requested_signal)
            if self._wait_group_exit(process_group, wait_timeout):
                break
        else:
            raise ProcessError(f"{self.name}的旧进程组未能完全退出")

        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:  # pragma: no cover - group already gone
            pass
        with self._lock:
            if self._process is process and self._state == "stopping":
                self._return_code = process.poll()
                self._ended_at = time.time()
                self._state = "stopped"
        self._on_change()

    def snapshot(self) -> dict:
        with self._lock:
            return {
                "name": self.name,
                "state": self._state,
                "running": self.running,
                "started_at": self._started_at,
                "ended_at": self._ended_at,
                "return_code": self._return_code,
                "pid": self._process.pid if self._process is not None else None,
                "process_group": self._process_group,
                "log_path": str(self._log_path) if self._log_path else None,
                "tail": list(self._tail)[-12:],
            }


class ProcessSlots:
    def __init__(self, on_change: Callable[[], None] | None = None):
        self.runtime = ManagedProcess("导航运行栈", on_change)
        self.collection = ManagedProcess("传感器数据采集", on_change)
        self.processing = ManagedProcess("离线地图处理", on_change)

    def stop_all(self) -> None:
        # Stop motion first, then allow bag/offline jobs to flush cleanly.
        for process in (self.runtime, self.collection, self.processing):
            process.stop()

    def snapshot(self) -> dict:
        return {
            "runtime": self.runtime.snapshot(),
            "collection": self.collection.snapshot(),
            "processing": self.processing.snapshot(),
        }

    def running_names(self) -> list[str]:
        return [
            name
            for name, process in (
                ("runtime", self.runtime),
                ("collection", self.collection),
                ("processing", self.processing),
            )
            if process.running
        ]
