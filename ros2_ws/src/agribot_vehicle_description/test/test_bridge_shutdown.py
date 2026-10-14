import importlib.util
from pathlib import Path
from unittest.mock import Mock

import pytest
from rclpy.executors import ExternalShutdownException


SRC = Path(__file__).resolve().parents[2]
BRIDGES = (
    (SRC / "agribot_ackermann_mppi/scripts/sim_navsat_bridge.py", "SimNavSatBridge"),
    (SRC / "agribot_hardware_bringup/localization/navsat/scripts/navsat_pose_bridge.py", "NavSatPoseBridge"),
)


@pytest.mark.parametrize("script,class_name", BRIDGES)
@pytest.mark.parametrize("exception,context_ok,accepted", (
    (KeyboardInterrupt(), True, True),
    (ExternalShutdownException(), False, True),
    (RuntimeError("Unable to convert call argument to Python object"), False, True),
    (RuntimeError("Unable to convert call argument to Python object"), True, False),
    (RuntimeError("Unrelated error during shutdown"), False, False),
))
def test_shutdown_guard_does_not_hide_live_or_unrelated_errors(
        script, class_name, exception, context_ok, accepted, monkeypatch):
    spec = importlib.util.spec_from_file_location(class_name + "_shutdown_test", script)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    node = Mock()
    shutdown = Mock()
    monkeypatch.setattr(module, class_name, lambda: node)
    monkeypatch.setattr(module.rclpy, "init", Mock())
    monkeypatch.setattr(module.rclpy, "ok", lambda: context_ok)
    monkeypatch.setattr(module.rclpy, "try_shutdown", shutdown)
    monkeypatch.setattr(module.rclpy, "spin", Mock(side_effect=exception))
    if accepted:
        module.main()
    else:
        with pytest.raises(RuntimeError, match=str(exception)):
            module.main()
    node.destroy_node.assert_called_once_with()
    shutdown.assert_called_once_with()
