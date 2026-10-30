import threading
from types import SimpleNamespace

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
import pytest

from agribot_mobile_app.catalog import GridData
from agribot_mobile_app.saved_path_runner import SavedPathRunner
from agribot_mobile_app.navigation_health import LocalizationUnavailable


class ResultFuture:
    def __init__(self, status, pending_calls=0):
        self.status = status
        self.pending_calls = pending_calls

    def done(self):
        if self.pending_calls:
            self.pending_calls -= 1
            return False
        return True

    def result(self):
        return SimpleNamespace(status=self.status)


def runner_fixture(future):
    runner = object.__new__(SavedPathRunner)
    runner.stop_event = threading.Event()
    runner.goal_tolerance = 0.25
    runner.footprint = [[.7, .53], [.7, -.53], [-.7, -.53], [-.7, .53]]
    grid = GridData(200, 200, 1.0, -50.0, -50.0, 0.0, bytes(40000))
    runner.observe = lambda: ({"x": 0.0, "y": 0.0, "yaw": 0.0}, grid)
    runner.follow = object()
    requests, plans, updates = [], [], []

    def request(client, goal):
        requests.append(goal)
        return SimpleNamespace(get_result_async=lambda: future)

    def plan(pose, target):
        plans.append(target)
        return [pose, target]

    runner._request = request
    runner._plan = plan
    runner._cancel = lambda handle: None
    runner.message = lambda poses: Path()
    runner.publish = lambda poses: None
    runner.update = lambda status, message: updates.append((status, message))
    runner.recovery_ready = lambda: True
    runner.on_progress = lambda index: None
    original = [{"x": i*0.1, "y": 0.0, "yaw": 0.0} for i in range(101)]
    return runner, original, requests, plans, updates


def test_all_six_detours_are_allowed_before_the_limit_stops_the_task():
    runner, original, requests, plans, updates = runner_fixture(
        ResultFuture(GoalStatus.STATUS_ABORTED))
    runner._run(original)
    assert runner.MAX_DETOURS == 6
    assert len(plans) == 6
    assert len(requests) == 7
    assert updates[-1][0] == "failed"
    assert "绕行次数达到上限" in updates[-1][1]


def test_running_monitor_uses_one_meter_horizon(monkeypatch):
    runner, original, requests, plans, updates = runner_fixture(
        ResultFuture(GoalStatus.STATUS_SUCCEEDED, pending_calls=1))
    distances = []

    def blocked(*args, **kwargs):
        distances.append(kwargs["distance"])
        return False

    monkeypatch.setattr("agribot_mobile_app.saved_path_runner.blocked_ahead", blocked)
    runner._run(original)
    assert distances == [1.0]
    assert not plans
    assert updates[-1][0] == "succeeded"


class FastEvent(threading.Event):
    def wait(self, timeout=None):
        return super().wait(min(timeout or .001, .001))


def test_loss_cancels_old_follow_then_recovers_from_retained_progress():
    runner, original, requests, plans, updates = runner_fixture(ResultFuture(GoalStatus.STATUS_ABORTED))
    runner.stop_event = FastEvent()
    runner.RECOVERY_STABLE_SEC = .003
    phase = {"request": 0, "samples": 0, "recovering": False}
    events, displayed = [], []
    grid = runner.observe()[1]

    def observe():
        if phase["request"] == 1 and not phase["recovering"]:
            phase["samples"] += 1
            if phase["samples"] >= 2:
                phase["recovering"] = True
                raise LocalizationUnavailable("fusion gap")
        x = 2.0 if phase["request"] else 0.0
        return {"x": x, "y": 0.0, "yaw": 0.0}, grid

    def request(client, goal):
        phase["request"] += 1
        number = phase["request"]
        requests.append(goal)
        events.append(f"request{number}")
        if number == 2:
            assert "cancel1" in events
        future = ResultFuture(GoalStatus.STATUS_SUCCEEDED, pending_calls=1000 if number == 1 else 0)
        return SimpleNamespace(number=number, get_result_async=lambda: future)

    runner.observe, runner._request = observe, request
    runner._cancel = lambda handle: events.append(f"cancel{handle.number}") if handle else None
    runner.publish = lambda poses: displayed.append(list(poses))
    runner._run(original)
    assert len(requests) == 2
    assert len(displayed) == 2
    assert displayed[1][0]["x"] == 2.0
    assert displayed[1][-1] == original[-1]
    assert not plans
    assert any("保留路线" in message for _, message in updates)
    assert any("稳定恢复" in message for _, message in updates)
    assert updates[-1][0] == "succeeded"
    assert not any(status == "failed" for status, _ in updates)


def test_cancel_while_waiting_never_resends_a_follow_goal():
    runner, original, requests, plans, updates = runner_fixture(
        ResultFuture(GoalStatus.STATUS_SUCCEEDED, pending_calls=1000))
    runner.stop_event = FastEvent()
    grid = runner.observe()[1]

    def observe():
        if requests:
            raise LocalizationUnavailable("fusion gap")
        return {"x": 0.0, "y": 0.0, "yaw": 0.0}, grid

    def update(status, message):
        updates.append((status, message))
        if "保留路线" in message:
            runner.stop_event.set()

    canceled = []
    runner.observe, runner.update = observe, update
    runner._cancel = lambda handle: canceled.append(handle) if handle else None
    runner._run(original)
    assert len(requests) == 1
    assert len(canceled) == 1
    assert not plans
    assert updates[-1][0] == "canceled"


def test_readiness_chatter_must_restart_the_stability_window(monkeypatch):
    runner, _, _, _, _ = runner_fixture(ResultFuture(GoalStatus.STATUS_SUCCEEDED))
    runner.RECOVERY_STABLE_SEC = .3
    clock = [0.0]
    calls = [0]
    original_observe = runner.observe

    class ClockEvent:
        def wait(self, timeout):
            clock[0] += timeout
            return False

    def observe():
        calls[0] += 1
        if calls[0] in (1, 3):
            raise LocalizationUnavailable("fusion gap")
        return original_observe()

    runner.stop_event = ClockEvent()
    runner.observe = observe
    monkeypatch.setattr("agribot_mobile_app.saved_path_runner.time.monotonic", lambda: clock[0])
    assert runner._wait_for_localization() is not None
    assert calls[0] >= 7
    assert clock[0] >= .7


def test_hard_input_error_still_fails_instead_of_auto_resuming():
    runner, original, requests, _, updates = runner_fixture(ResultFuture(GoalStatus.STATUS_SUCCEEDED))

    def observe():
        raise RuntimeError("wrong map frame")

    runner.observe = observe
    runner._run(original)
    assert not requests
    assert updates[-1] == ("failed", "wrong map frame")
    assert not any("保留路线" in message for _, message in updates)


def test_long_localization_outage_waits_until_user_cancels():
    runner, original, requests, plans, updates = runner_fixture(ResultFuture(GoalStatus.STATUS_SUCCEEDED))
    calls = [0]

    class CancelAfterPolling(threading.Event):
        def wait(self, timeout=None):
            calls[0] += 1
            if calls[0] >= 20:
                self.set()
            return self.is_set()

    def observe():
        raise LocalizationUnavailable("fusion gap")

    runner.stop_event = CancelAfterPolling()
    runner.observe = observe
    runner._run(original)
    assert calls[0] == 20
    assert not requests and not plans
    assert updates[-1][0] == "canceled"


def test_localization_loss_during_planning_cancels_the_plan_request():
    runner, _, requests, _, _ = runner_fixture(
        ResultFuture(GoalStatus.STATUS_ABORTED, pending_calls=1000))
    runner.plan = object()
    runner.stop_event = FastEvent()
    runner.message = lambda poses: Path(poses=[PoseStamped(), PoseStamped()])
    canceled = []
    runner._cancel = canceled.append

    def observe():
        raise LocalizationUnavailable("fusion gap")

    runner.observe = observe
    with pytest.raises(LocalizationUnavailable):
        SavedPathRunner._plan(runner, {"x": 0.0}, {"x": 1.0})
    assert len(requests) == 1 and len(canceled) == 1


def test_failed_follow_cancellation_does_not_wait_or_resume():
    runner, original, requests, _, updates = runner_fixture(
        ResultFuture(GoalStatus.STATUS_SUCCEEDED, pending_calls=1000))
    runner.stop_event = FastEvent()
    runner.node = SimpleNamespace(get_logger=lambda: SimpleNamespace(error=lambda message: None))
    grid = runner.observe()[1]

    def observe():
        if requests:
            raise LocalizationUnavailable("fusion gap")
        return {"x": 0.0, "y": 0.0, "yaw": 0.0}, grid

    def cancel(handle):
        if handle:
            raise RuntimeError("cancel not acknowledged")

    runner.observe, runner._cancel = observe, cancel
    runner._wait_for_localization = lambda: pytest.fail("cannot resume before cancellation completes")
    runner._run(original)
    assert len(requests) == 1
    assert updates[-1][0] == "failed"
    assert "cancel not acknowledged" in updates[-1][1]


def test_nav2_abort_with_stale_inputs_waits_without_spending_detour_budget():
    runner, original, requests, plans, updates = runner_fixture(ResultFuture(GoalStatus.STATUS_ABORTED))
    runner.stop_event = FastEvent()
    runner.RECOVERY_STABLE_SEC = .003
    checks = [0]

    def ready():
        checks[0] += 1
        return checks[0] >= 3

    def request(client, goal):
        requests.append(goal)
        future = ResultFuture(GoalStatus.STATUS_ABORTED if len(requests) == 1 else GoalStatus.STATUS_SUCCEEDED)
        return SimpleNamespace(get_result_async=lambda: future)

    runner.recovery_ready, runner._request = ready, request
    runner._run(original)
    assert len(requests) == 2
    assert not plans
    assert any("保留路线" in message for _, message in updates)
    assert updates[-1][0] == "succeeded"
