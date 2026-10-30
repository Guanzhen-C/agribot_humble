"""Only whitelisted driverless estimator launches, each in its own process group."""

import os
from pathlib import Path
import signal
import subprocess
import time


class EstimatorProcess:
    def __init__(self, name, map_base, log_root, use_sim_time=False):
        if name not in ("navsat", "fastlivo"):
            raise ValueError("Unknown estimator")
        self.name, self.map_base = name, str(Path(map_base).resolve())
        self.log_root, self.use_sim_time = Path(log_root), use_sim_time
        self.process = None
        self.group = None
        self.log = None

    def alive(self):
        process, group = self.process, self.group
        if process is not None:
            process.poll()
        if group is None:
            return False
        try:
            os.killpg(group, 0)
            return True
        except ProcessLookupError:
            return False

    def start(self):
        if self.alive():
            raise RuntimeError("Estimator process group is already running")
        self.log_root.mkdir(parents=True, exist_ok=True)
        self.log = (self.log_root / f"{self.name}_{time.time_ns()}.log").open("a", buffering=1)
        command = ["ros2", "launch", "agribot_localization_switch", "estimator_backend.launch.py",
                   f"estimator:={self.name}", f"map_base:={self.map_base}",
                   f"use_sim_time:={'true' if self.use_sim_time else 'false'}"]
        environment = os.environ.copy()
        environment.update(OPENBLAS_NUM_THREADS="1", MKL_NUM_THREADS="1", OMP_NUM_THREADS="1")
        if self.name == "fastlivo":
            command = ["nice", "-n", "5"] + command
        try:
            self.process = subprocess.Popen(command, stdout=self.log, stderr=subprocess.STDOUT,
                                            start_new_session=True, env=environment)
            self.group = self.process.pid
        except Exception:
            self.log.close()
            raise

    def stop(self):
        for requested, seconds in ((signal.SIGINT, 20), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
            if not self.alive():
                break
            try:
                os.killpg(self.group, requested)
            except ProcessLookupError:
                break
            end = time.monotonic() + seconds
            while self.alive() and time.monotonic() < end:
                time.sleep(.05)
        if self.alive():
            raise RuntimeError("Estimator children did not exit")
        if self.process:
            self.process.wait(timeout=1)
        if self.log:
            self.log.close()
        self.process = self.group = self.log = None
