"""Opt-in real GUI regression; never starts an estimator or a vehicle."""

import os
from pathlib import Path
import signal
import subprocess
import time

import pytest
import yaml


PACKAGE = Path(__file__).resolve().parents[1]
pytestmark = pytest.mark.skipif(os.environ.get("AGRIBOT_TEST_RVIZ") != "1",
                                reason="Requires an explicitly enabled local X11 RViz test")


@pytest.mark.parametrize("attempt", range(3))
@pytest.mark.parametrize("stop_signal", [signal.SIGINT, signal.SIGTERM])
def test_navigation_panel_shuts_down_without_unmapped_callback_deleters(tmp_path, attempt, stop_signal):
    config = tmp_path / ("nav2_panel_" + str(attempt) + ".rviz")
    config.write_text(yaml.safe_dump({
        "Panels": [{"Class": "nav2_rviz_plugins/Navigation 2", "Name": "Navigation 2"}],
        "Visualization Manager": {"Class": "", "Global Options": {"Fixed Frame": "map"},
            "Displays": [{"Class": "rviz_default_plugins/Grid", "Name": "Grid", "Enabled": True}]},
    }))
    environment = dict(os.environ, ROS_DOMAIN_ID="90", ROS_LOCALHOST_ONLY="1", RMW_IMPLEMENTATION="rmw_cyclonedds_cpp")
    log_path = tmp_path / "rviz.log"
    with log_path.open("w") as log:
        process = subprocess.Popen([str(PACKAGE / "scripts/run_rviz2_clean_env.sh"), str(config)],
            env=environment, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            time.sleep(6)
            assert process.poll() is None, log_path.read_text()
            assert "libnav2_rviz_plugins.so" in Path(f"/proc/{process.pid}/maps").read_text()
            process.send_signal(stop_signal)
            process.wait(timeout=12)
        finally:
            if process.poll() is None:
                os.killpg(process.pid, signal.SIGTERM)
                process.wait(timeout=5)
    assert process.returncode == 0, log_path.read_text()
