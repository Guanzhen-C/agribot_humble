"""Signal ordering for the two simulated LiDAR ingestion adapters."""

import ast
from pathlib import Path
from types import SimpleNamespace

import pytest


SCRIPTS = Path(__file__).resolve().parents[1] / "scripts"
ADAPTERS = [
    ("simulation_vehicle_cloud_filter.py", "SimulationVehicleCloudFilter"),
    ("simulation_lidar_time_adapter.py", "SimulationLidarTimeAdapter"),
]


def load_main(filename, class_name, events, handlers, spin, constructor=None):
    def install(sig, handler):
        previous = handlers[sig]
        handlers[sig] = handler
        return previous

    def init(**kwargs):
        assert kwargs == {"signal_handler_options": "disabled"}
        events.append("init")

    source = SCRIPTS / filename
    main = next(node for node in ast.parse(source.read_text()).body
                if isinstance(node, ast.FunctionDef) and node.name == "main")
    namespace = {
        "signal": SimpleNamespace(SIGINT=2, SIGTERM=15, signal=install),
        "SignalHandlerOptions": SimpleNamespace(NO="disabled"),
        class_name: constructor or (lambda: SimpleNamespace(
            destroy_node=lambda: events.append("destroy"))),
        "rclpy": SimpleNamespace(init=init, ok=lambda: True, spin_once=spin,
                                  try_shutdown=lambda: events.append("shutdown"),
                                  executors=SimpleNamespace(
                                      ExternalShutdownException=type("ExternalShutdown", (Exception,), {}))),
    }
    exec(compile(ast.Module(body=[main], type_ignores=[]), str(source), "exec"), namespace)
    return namespace["main"]


@pytest.mark.parametrize("filename,class_name", ADAPTERS)
@pytest.mark.parametrize("stop_signal", [2, 15])
def test_signals_wait_for_callback_return(filename, class_name, stop_signal):
    events, handlers = [], {2: "old_int", 15: "old_term"}

    def spin(_node, timeout_sec):
        assert timeout_sec == .1
        handlers[stop_signal](stop_signal, None)
        assert events == ["init"]
        events.append("callback_return")

    load_main(filename, class_name, events, handlers, spin)()
    assert events == ["init", "callback_return", "destroy", "shutdown"]
    assert handlers == {2: "old_int", 15: "old_term"}


@pytest.mark.parametrize("filename,class_name", ADAPTERS)
@pytest.mark.parametrize("failure_stage", ["construction", "callback"])
def test_real_errors_propagate_after_cleanup(filename, class_name, failure_stage):
    events, handlers = [], {2: "old_int", 15: "old_term"}

    def fail(*_args, **_kwargs):
        raise RuntimeError("actual adapter error")

    main = load_main(filename, class_name, events, handlers, fail,
                     constructor=fail if failure_stage == "construction" else None)
    with pytest.raises(RuntimeError, match="actual adapter error"):
        main()
    assert events == (["init", "shutdown"] if failure_stage == "construction"
                      else ["init", "destroy", "shutdown"])
    assert handlers == {2: "old_int", 15: "old_term"}
