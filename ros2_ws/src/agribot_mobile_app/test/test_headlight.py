from concurrent.futures import Future
from http.client import HTTPConnection
from http import HTTPStatus
import json
from pathlib import Path
import threading
import time
from types import SimpleNamespace

import pytest
from std_srvs.srv import SetBool
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
import rclpy
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from scout_msgs.msg import ScoutStatus

from agribot_mobile_app.gateway_node import ApiError, GatewayHttpServer, MobileGateway


def gateway(*, read_only=False, supported=True, fresh=True, result=True,
            confirmed=True, available=True):
    calls = []
    feedback = {"fresh": fresh, "headlight_supported": supported, "read_only": read_only}

    def call(request):
        calls.append(request.data)
        future = Future()
        future.set_result(SetBool.Response(success=result, message="CAN rejected"))
        return future

    node = SimpleNamespace(
        _state={"vehicle": {"id": "differential"}},
        _lock=threading.RLock(), _task_transition_lock=threading.RLock(),
        _headlight_lock=threading.Lock(),
        _chassis_feedback=SimpleNamespace(
            snapshot=lambda _: {"chassis_feedback": feedback},
            wait_for_headlight=lambda *_, **__: confirmed),
        _headlight_client=SimpleNamespace(
            wait_for_service=lambda **_: available, call_async=call),
        _touch=lambda: None,
    )
    return node, calls


@pytest.mark.parametrize("enabled", [False, True])
def test_light_command_uses_only_boolean_service_and_confirmed_feedback(enabled):
    node, calls = gateway()
    result = MobileGateway.set_headlight(node, {"enabled": enabled, "vehicle_type": "differential"})
    assert result["headlight"] is enabled
    assert result["confirmed"] is True
    assert calls == [enabled]


@pytest.mark.parametrize("enabled", [None, 0, 1, "false", "true", [], {}])
def test_light_command_rejects_non_boolean_values(enabled):
    node, calls = gateway()
    with pytest.raises(ApiError) as error:
        MobileGateway.set_headlight(node, {"enabled": enabled, "vehicle_type": "differential"})
    assert error.value.status == HTTPStatus.BAD_REQUEST
    assert not calls


@pytest.mark.parametrize("options", [{"read_only": True}, {"supported": False}, {"fresh": False}, {"available": False}])
def test_readonly_unsupported_stale_or_offline_cannot_send_lights(options):
    node, calls = gateway(**options)
    with pytest.raises(ApiError):
        MobileGateway.set_headlight(node, {"enabled": True, "vehicle_type": "differential"})
    assert not calls


def test_wrong_vehicle_cannot_control_another_chassis():
    node, calls = gateway()
    with pytest.raises(ApiError, match="车型"):
        MobileGateway.set_headlight(node, {"enabled": True, "vehicle_type": "ackermann"})
    assert not calls


@pytest.mark.parametrize("options,status", [({"result": False}, HTTPStatus.CONFLICT)])
def test_rejection_or_absent_feedback_never_reports_success(options, status):
    node, calls = gateway(**options)
    with pytest.raises(ApiError) as error:
        MobileGateway.set_headlight(node, {"enabled": True, "vehicle_type": "differential"})
    assert error.value.status == status
    assert calls == [True]


def test_missing_light_feedback_still_returns_the_accepted_switch_state():
    node, calls = gateway(confirmed=False)
    result = MobileGateway.set_headlight(node, {"enabled": True, "vehicle_type": "differential"})
    assert result["headlight"] is True
    assert result["confirmed"] is False
    assert calls == [True]


def test_waiting_for_lights_does_not_block_runtime_stop_or_task_transitions():
    node, _ = gateway()
    future = Future()
    sent = threading.Event()
    results = []

    def call(_):
        sent.set()
        return future

    node._headlight_client.call_async = call
    thread = threading.Thread(target=lambda: results.append(MobileGateway.set_headlight(
        node, {"enabled": True, "vehicle_type": "differential"})))
    thread.start()
    try:
        assert sent.wait(1)
        assert node._task_transition_lock.acquire(blocking=False)
        node._task_transition_lock.release()
    finally:
        future.set_result(SetBool.Response(success=True))
        thread.join(timeout=3)
    assert not thread.is_alive()
    assert results[0]["confirmed"]


def test_http_endpoint_calls_ros_service_and_waits_for_actual_status_feedback(tmp_path):
    rclpy.init()
    mobile = MobileGateway()
    chassis = Node("headlight_service_fixture")
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(mobile)
    executor.add_node(chassis)
    status_publisher = chassis.create_publisher(ScoutStatus, "/scout_status", 10)
    diagnostics_publisher = chassis.create_publisher(DiagnosticArray, "/diagnostics", 10)
    requested = [False]
    calls = []

    def service_callback(request, response):
        requested[0] = request.data
        calls.append(request.data)
        response.success = True
        return response

    service = chassis.create_service(SetBool, "/hardware/set_headlight", service_callback)

    def publish_feedback():
        message = ScoutStatus()
        message.light_control_enabled = True
        message.front_light_state.mode = int(requested[0])
        status_publisher.publish(message)
        status = DiagnosticStatus(name="agribot/chassis_can/differential", values=[
            KeyValue(key="feedback_fresh", value="true"),
            KeyValue(key="read_only", value="false"),
            KeyValue(key="headlight_supported", value="true"),
        ])
        diagnostics_publisher.publish(DiagnosticArray(status=[status]))

    timer = chassis.create_timer(0.05, publish_feedback)
    with mobile._lock:
        mobile._state["vehicle"]["id"] = "differential"
    server = GatewayHttpServer(("127.0.0.1", 0), mobile, Path(tmp_path))
    http_thread = threading.Thread(target=server.serve_forever, daemon=True)
    ros_thread = threading.Thread(target=executor.spin, daemon=True)
    connection = HTTPConnection("127.0.0.1", server.server_port, timeout=6)
    try:
        ros_thread.start()
        http_thread.start()
        deadline = time.monotonic() + 4
        while not mobile._chassis_feedback.snapshot("differential")["chassis_feedback"]["fresh"]:
            assert time.monotonic() < deadline
            time.sleep(0.02)
        assert mobile._headlight_client.wait_for_service(timeout_sec=1)
        for enabled in (True, False):
            connection.request("POST", "/api/v1/chassis/headlight",
                               json.dumps({"vehicle_type": "differential", "enabled": enabled}),
                               {"Content-Type": "application/json"})
            response = connection.getresponse()
            result = json.loads(response.read())
            assert response.status == 200, result
            assert result["ok"] and result["confirmed"]
            assert result["headlight"] is enabled
            assert mobile._chassis_feedback.snapshot("differential")["chassis"]["headlight"] is enabled
        assert calls == [True, False]
        assert service and timer
    finally:
        connection.close()
        server.shutdown()
        server.server_close()
        http_thread.join(timeout=2)
        executor.shutdown(timeout_sec=5)
        ros_thread.join(timeout=5)
        mobile.destroy_node()
        chassis.destroy_node()
        rclpy.shutdown()
