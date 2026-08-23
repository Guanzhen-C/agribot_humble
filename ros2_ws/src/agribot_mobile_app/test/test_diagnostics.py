from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

from agribot_mobile_app.gateway_node import MobileGateway, empty_time_sync_state


def test_humble_diagnostic_level_is_serialized_as_an_integer():
    status = DiagnosticStatus(
        level=DiagnosticStatus.OK,
        name="sensor_time_sync/lidar_camera",
        message="相邻测量时刻正常",
        hardware_id="software_sync",
        values=[KeyValue(key="p95_nearest_delta_ms", value="3.750")],
    )

    document = MobileGateway._diagnostic_document(status)

    assert document["level"] == 0
    assert document["values"]["p95_nearest_delta_ms"] == "3.750"


def test_stopped_diagnostics_are_reported_as_stale():
    state = empty_time_sync_state("授时诊断已停止")

    assert state["summary"]["level"] == 3
    assert state["summary"]["message"] == "授时诊断已停止"
    assert state["clocks"]["camera"]["message"] == "授时诊断已停止"
