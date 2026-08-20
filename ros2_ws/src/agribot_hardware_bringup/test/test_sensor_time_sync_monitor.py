import importlib.util
from pathlib import Path


SCRIPT = Path(__file__).parents[1] / "scripts" / "sensor_time_sync_monitor.py"
SPEC = importlib.util.spec_from_file_location("sensor_time_sync_monitor", SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(MODULE)


def test_nearest_residuals_selects_closest_measurement():
    assert MODULE.nearest_residuals([1.00, 1.10, 1.20], [0.99, 1.12, 1.19]) == [
        0.010000000000000009,
        0.020000000000000018,
        0.010000000000000009,
    ]


def test_topic_timing_reports_rate_and_regressions():
    timing = MODULE.TopicTiming("imu", "/imu/data", 80.0, 0.2, 20)
    timing.add(10.00, 10.01)
    timing.add(10.01, 10.02)
    timing.add(10.02, 10.03)
    assert abs(timing.rate_hz() - 100.0) < 1.0e-9
    assert timing.regressions == 0

    timing.add(9.90, 10.04)
    assert timing.regressions == 1


def test_topic_timing_trims_old_receipts():
    timing = MODULE.TopicTiming("lidar", "/lidar/points", 8.0, 0.3, 20)
    timing.add(1.0, 1.1)
    timing.add(2.0, 2.1)
    timing.trim(2.2, 0.5)
    assert timing.stamps() == [2.0]
