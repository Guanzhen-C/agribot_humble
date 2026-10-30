import threading
from types import SimpleNamespace

from rclpy.task import Future

from agribot_mobile_app.gateway_node import MobileGateway
from agribot_mobile_app.saved_path import handover_path_index


def route(count=104):
    return {"anchor_path_indices": [i*10 for i in range(count)]}


def gateway():
    node = object.__new__(MobileGateway)
    node._lock = threading.RLock()
    node._state = {"navigation": {"status": "executing"}}
    node._saved_runner = SimpleNamespace(stop_event=threading.Event())
    requests, updates, warnings = [], [], []

    def call(request):
        future = Future()
        requests.append(future)
        return future

    node._handover_client = SimpleNamespace(service_is_ready=lambda: True, call_async=call)
    node._saved_update = lambda status, message: updates.append((status, message))
    node.get_logger = lambda: SimpleNamespace(warning=warnings.append)
    return node, requests, updates, warnings


def test_hundredth_anchor_not_hundredth_dense_point_and_suffix_numbering():
    assert handover_path_index(route(), 1) == 990
    assert handover_path_index(route(), 51) == 490
    assert handover_path_index(route(), 100) == 0
    assert handover_path_index(route(), 101) == 0
    assert handover_path_index(route(99), 1) is None


def test_ordered_progress_sends_only_one_async_request():
    node, requests, updates, _ = gateway()
    progress = node._handover_progress_callback(route(), 51)
    for index in (0, 100, 489):progress(index)
    assert not requests
    progress(490);progress(491)
    assert len(requests) == 1
    requests[0].set_result(SimpleNamespace(success=True))
    progress(600)
    assert len(requests) == 1
    assert len(updates) == 1 and "KF-GINS继续定位" in updates[0][1]


def test_cancel_or_replaced_task_ignores_late_service_response():
    for replace in (False, True):
        node, requests, updates, _ = gateway()
        progress = node._handover_progress_callback(route(), 101)
        progress(0)
        if replace:node._state["navigation"] = {"status": "canceled"}
        else:node._saved_runner.stop_event.set()
        requests[0].set_result(SimpleNamespace(success=True))
        progress(2)
        assert not updates and len(requests) == 1


def test_unavailable_service_retries_without_blocking_or_cancelling_navigation(monkeypatch):
    node, requests, updates, warnings = gateway()
    clock = [100.]
    monkeypatch.setattr("agribot_mobile_app.gateway_node.time.monotonic", lambda: clock[0])
    node._handover_client.service_is_ready = lambda: False
    progress = node._handover_progress_callback(route(), 1)
    progress(990)
    assert warnings and not requests
    node._handover_client.service_is_ready = lambda: True
    progress(991)
    assert not requests
    clock[0] += 1.1
    progress(992)
    assert len(requests) == 1
    assert node._state["navigation"]["status"] == "executing"
