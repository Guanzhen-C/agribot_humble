from types import SimpleNamespace
import copy
import threading
import numpy as np
import pytest
import rclpy
from builtin_interfaces.msg import Time
from nav_msgs.msg import Odometry
from sensor_msgs.msg import NavSatFix
from std_srvs.srv import Trigger

from agribot_localization_switch.node import LocalizationSwitch


class FakeProcess:
    def __init__(self, *args):self.running = False;self.starts = self.stops = 0
    def start(self):
        assert not self.running
        self.running = True;self.starts += 1
    def stop(self):self.running = False;self.stops += 1
    def alive(self):return self.running


@pytest.fixture
def manager(monkeypatch, tmp_path):
    import agribot_localization_switch.node as module
    monkeypatch.setattr(module, "EstimatorProcess", FakeProcess)
    rclpy.init(args=["--ros-args", "-p", f"map_base:={tmp_path}/map"])
    node = LocalizationSwitch()
    clock = [1000.]
    monkeypatch.setattr(module.time, "monotonic", lambda: clock[0])
    node.get_clock = lambda: SimpleNamespace(now=lambda: SimpleNamespace(nanoseconds=int(clock[0]*1e9)))
    node.enabled = True
    node.worker = lambda function: function()
    messages, transforms = [], []
    node.odom_pub = SimpleNamespace(publish=messages.append)
    node.tf = SimpleNamespace(sendTransform=transforms.append)
    try:yield node, clock, messages, transforms
    finally:node.close();node.destroy_node();rclpy.shutdown()


def feed(node, clock, fused=False, bias=.1, gnss=True, fused_variances=None):
    now = clock[0]
    if gnss:
        node.quality = 4
        node.quality_at = node.fix_at = node.heading_at = now
    nav = Odometry();nav.header.frame_id = "map";nav.child_frame_id = "base_link"
    stamp = now-.25
    nav.header.stamp = Time(sec=int(stamp), nanosec=int((stamp%1)*1e9))
    nav.pose.pose.orientation.w = 1.
    nav.pose.pose.position.x = .2*(stamp-1000.)
    nav.twist.twist.linear.x = .2
    nav.pose.covariance = list((np.eye(6)*.01).reshape(-1))
    node.navsat_callback(nav)
    if fused:
        msg = Odometry();msg.header.frame_id = "map";msg.child_frame_id = "base_link"
        msg.header.stamp = Time(sec=int(now), nanosec=int((now%1)*1e9))
        msg.pose.pose.orientation.w = 1.
        msg.pose.pose.position.x = .2*(now-1000.)+bias
        msg.twist.twist.linear.x = .2
        msg.pose.covariance = list((np.eye(6)*.01).reshape(-1))
        if fused_variances is not None:
            msg.pose.covariance[0], msg.pose.covariance[7] = fused_variances
        node.fused_callback(msg)
        node.fused_ready, node.fused_ready_at = True, now


def test_continues_navsat_through_warmup_then_continuously_hands_over(manager):
    node, clock, messages, transforms = manager
    feed(node, clock);node.tick()
    assert messages[-1].twist.twist.linear.x == .2
    assert node.fastlivo.starts == 0
    for _ in range(2):node.switch_callback(Trigger.Request(), Trigger.Response())
    assert node.fastlivo.starts == 1 and node.navsat.running
    clock[0] += .1;feed(node, clock);node.tick()
    assert node.core.mode == "navsat" and node.navsat.running
    before = len(messages)
    for _ in range(30):
        clock[0] += .1;feed(node, clock, fused=True)
        last_pose = messages[-1].pose.pose.position.x
        node.tick()
        if node.core.mode == "fastlivo_rtk":
            break
    assert len(messages) > before
    assert node.core.mode == "fastlivo_rtk"
    assert messages[-1].pose.pose.position.x == pytest.approx(last_pose)
    assert messages[-1].twist.twist.linear.x == .2
    assert not node.navsat.running
    assert node.fastlivo.running
    assert all([t.child_frame_id for t in group] == ["odom", "base_link"] for group in transforms)


def test_slow_backend_start_never_blocks_public_localization_or_revokes_ready(manager, monkeypatch):
    node, clock, messages, _ = manager
    entered, release = threading.Event(), threading.Event()
    start = node.fastlivo.start

    def slow_start():
        entered.set()
        assert release.wait(5.)
        start()

    monkeypatch.setattr(node.fastlivo, "start", slow_start)
    node.worker = LocalizationSwitch.worker.__get__(node)
    flags = []
    node.ready_pub = SimpleNamespace(publish=flags.append)
    try:
        feed(node, clock); node.tick(); node.flags()
        response = node.switch_callback(Trigger.Request(), Trigger.Response())
        assert response.success and entered.wait(1.)
        assert node.launching and not node.fastlivo.running
        for _ in range(30):
            clock[0] += .1
            feed(node, clock); node.tick(); node.flags()
        assert len(messages) == 31
        assert all(flag.data is True for flag in flags)
        assert node.navsat.running and node.navsat.stops == 0
        stamps = [m.header.stamp.sec + m.header.stamp.nanosec*1e-9 for m in messages]
        assert max(np.diff(stamps)) <= .101
    finally:
        release.set()
        for thread in node.workers:
            thread.join(timeout=2.)
    assert node.fastlivo.running


def test_ready_and_velocity_remain_continuous_through_handover_and_rtk_loss(manager):
    node, clock, messages, _ = manager
    flags = []
    node.ready_pub = SimpleNamespace(publish=flags.append)
    feed(node, clock); node.tick(); node.flags()
    node.switch_callback(Trigger.Request(), Trigger.Response())
    for _ in range(70):
        clock[0] += .1
        last_x = messages[-1].pose.pose.position.x
        feed(node, clock, fused=True)
        node.tick(); node.flags()
        assert len(messages) >= 2
        assert flags[-1].data is True
        assert abs(messages[-1].pose.pose.position.x-last_x) < .03
        assert messages[-1].twist.twist.linear.x == .2
    assert node.core.mode == "fastlivo_rtk"
    assert node.navsat.stops == 1
    node.quality = 0
    for _ in range(60):
        clock[0] += .1
        feed(node, clock, fused=True, gnss=False)
        node.tick(); node.flags()
        assert flags[-1].data is True
    assert len(messages) == 131


def test_bad_new_pose_keeps_navsat_and_does_not_stop_it(manager):
    node, clock, messages, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    for _ in range(6):
        feed(node, clock, fused=True, bias=2.)
        node.tick();clock[0] += .5
    assert node.core.mode == "navsat" and node.navsat.running
    assert len(messages) == 6


def test_low_rate_valid_fused_frames_keep_overlap_between_updates(manager):
    node, clock, messages, _ = manager
    feed(node, clock); node.tick()
    node.switch_callback(Trigger.Request(), Trigger.Response())
    start = clock[0]
    for index in range(65):
        clock[0] = start+index*.05
        feed(node, clock, fused=index % 8 == 0)
        node.tick()
        if node.core.mode == "fastlivo_rtk":
            break
    assert node.core.mode == "fastlivo_rtk"
    assert index <= 48
    assert not node.navsat.running and node.fastlivo.running
    assert len(messages) >= 20
    assert clock[0]-start >= node.core.stable_sec


def test_long_fused_gap_restarts_overlap_validation(manager):
    node, clock, _, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    feed(node, clock, fused=True); node.tick()
    clock[0] += 1.1
    feed(node, clock, fused=True); node.tick()
    assert node.core.stable_since == clock[0]
    assert node.core.mode == "navsat"


def test_one_float_frame_keeps_live_ins_and_does_not_interrupt_navigation(manager):
    node, clock, messages, _ = manager
    feed(node, clock);node.tick()
    node.switch_callback(Trigger.Request(), Trigger.Response())
    clock[0] += .1;feed(node, clock, gnss=False)
    node.quality = 5
    node.tick()
    assert len(messages) == 2
    assert node.core.mode == "navsat"


def test_short_gnss_gap_uses_real_ins_then_recovers_without_restart(manager):
    node, clock, messages, _ = manager
    feed(node, clock); node.tick()
    first_fix_at = node.fix_at
    node.quality = 5
    for elapsed in (.1, 1., 2., 3.9):
        clock[0] = first_fix_at + elapsed
        feed(node, clock, gnss=False); node.tick()
        assert node.navsat_healthy(clock[0])
    assert len(messages) == 5
    assert node.fix_at == first_fix_at
    clock[0] = first_fix_at + 4.01
    feed(node, clock, gnss=False); node.tick()
    assert not node.navsat_healthy(clock[0])
    assert len(messages) == 5
    clock[0] += .1
    feed(node, clock); node.tick()
    assert node.navsat_healthy(clock[0]) and len(messages) == 6
    assert node.navsat.starts == 1


def test_holdover_never_bypasses_covariance_or_imu_freshness_limits(manager):
    node, clock, messages, _ = manager
    feed(node, clock); node.tick()
    clock[0] += .2
    feed(node, clock, gnss=False)
    node.samples["navsat"].covariance[0, 0] = .26
    node.tick()
    assert not node.navsat_healthy(clock[0]) and len(messages) == 1
    node.samples["navsat"].covariance[0, 0] = .01
    clock[0] += .6
    assert not node.navsat_healthy(clock[0])


def test_initialization_cannot_start_on_float_or_missing_heading(manager):
    node, clock, messages, _ = manager
    feed(node, clock); node.quality = 5
    node.tick()
    assert not messages and not node.navsat_initialized
    node.quality = 4; node.heading_at = -float("inf")
    node.tick()
    assert not messages and not node.navsat_initialized


def fix_message(clock):
    message = NavSatFix()
    now = clock[0]
    message.header.stamp = Time(sec=int(now), nanosec=int((now%1)*1e9))
    message.status.status = 2
    message.latitude, message.longitude, message.altitude = 38., 115., 20.
    message.position_covariance = [.001, 0., 0., 0., .001, 0., 0., 0., .002]
    return message


@pytest.mark.parametrize("invalid", ["float", "quality_stale", "duplicate", "nan", "latitude", "covariance", "old_stamp"])
def test_rejected_gnss_never_extends_the_holdover_window(manager, invalid):
    node, clock, _, _ = manager
    feed(node, clock)
    first = fix_message(clock)
    node.fix_callback(first)
    accepted_at, accepted_stamp = node.fix_at, node.last_fix_stamp
    clock[0] += .1
    node.quality_at = clock[0]
    message = fix_message(clock)
    if invalid == "float": node.quality = 5
    elif invalid == "quality_stale": node.quality_at = clock[0] - .31
    elif invalid == "duplicate": message.header.stamp = first.header.stamp
    elif invalid == "nan": message.latitude = float("nan")
    elif invalid == "latitude": message.latitude = 91.
    elif invalid == "covariance": message.position_covariance[4] = float("nan")
    else: message.header.stamp.sec -= 2
    node.fix_callback(message)
    assert node.fix_at == accepted_at and node.last_fix_stamp == accepted_stamp


def test_fresh_fixed_gnss_refreshes_holdover_without_resetting_estimator(manager):
    node, clock, _, _ = manager
    feed(node, clock); node.tick()
    clock[0] += 1.
    node.quality_at = clock[0]
    node.fix_callback(fix_message(clock))
    assert node.fix_at == clock[0]
    assert node.navsat_initialized and node.navsat.starts == 1


def test_raw_odom_seed_uses_predicted_ins_pose_at_real_measurement_stamp(manager):
    node, clock, _, _ = manager
    feed(node, clock)
    node.switch_callback(Trigger.Request(), Trigger.Response())
    seeds = []
    node.seed_pub = SimpleNamespace(publish=seeds.append)
    raw = Odometry();raw.header.stamp.sec = 1000
    raw.header.frame_id = "odom";raw.child_frame_id = "base_link";raw.pose.pose.orientation.w = 1.
    node.raw_callback(raw)
    assert len(seeds) == 1
    assert seeds[0].header.stamp == raw.header.stamp
    assert seeds[0].pose.pose.position.x == pytest.approx(0.)
    clock[0] += 1.
    node.raw_callback(raw)
    assert len(seeds) == 1


def test_fused_not_ready_does_not_revoke_healthy_navsat_or_refresh_old_pose(manager):
    node, clock, messages, _ = manager
    flags = []
    node.ready_pub = SimpleNamespace(publish=flags.append)
    node.switch_callback(Trigger.Request(), Trigger.Response())
    feed(node, clock, fused=True)
    node.fused_ready = False
    node.tick(); node.flags()
    assert flags[-1].data is True
    assert node.core.mode == "navsat" and node.navsat.running
    clock[0] += .1
    node.tick()
    assert len(messages) == 1
    clock[0] += .5
    node.flags()
    assert flags[-1].data is False


def test_camera_init_alignment_requires_matching_raw_measurement_time(manager):
    node, clock, messages, transforms = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True)
    raw = Odometry()
    raw.header.frame_id = "odom"; raw.child_frame_id = "base_link"
    raw.header.stamp.sec = 1000
    raw.pose.pose.orientation.w = 1.
    raw.pose.pose.position.x = 2.
    node.raw_callback(raw)
    node.tick()
    assert node.core.mode == "fastlivo_rtk"
    camera = transforms[-1][-1]
    assert camera.child_frame_id == "camera_init"
    assert camera.header.frame_id == "odom"
    assert camera.transform.translation.x + 2. == pytest.approx(messages[-1].pose.pose.position.x)
    clock[0] += .1
    feed(node, clock, fused=True)
    node.tick()
    assert [tf.child_frame_id for tf in transforms[-1]] == ["odom", "base_link"]


def test_completed_handover_rejects_stale_fused_source_even_with_old_ready_flag(manager):
    node, clock, messages, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True); node.tick()
    assert node.core.mode == "fastlivo_rtk"
    before = len(messages)
    clock[0] += 4.1
    node.fused_ready_at = clock[0]
    node.tick()
    assert not node.fused_healthy(clock[0])
    assert len(messages) == before


@pytest.mark.parametrize("invalid", [False, True])
def test_fused_covariance_readiness_is_native_bool_and_publishable(manager, invalid):
    node, clock, _, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True); node.tick()
    assert node.core.mode == "fastlivo_rtk"
    flags = []
    node.ready_pub = SimpleNamespace(publish=flags.append)
    if invalid:
        node.samples["fastlivo_rtk"].covariance[0, 0] = -.01
    expected = not invalid
    assert node.sample_healthy("fastlivo_rtk", clock[0], 4.) is expected
    assert node.fused_healthy(clock[0]) is expected
    node.flags()
    assert flags[-1].data is expected
    if invalid:
        clock[0] += .1
        feed(node, clock, fused=True)
        node.tick(); node.flags()
        assert flags[-1].data is True


def test_high_global_covariance_still_blocks_initial_takeover(manager):
    node, clock, messages, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    for _ in range(30):
        feed(node, clock, fused=True, fused_variances=(6.04, 2.33))
        node.tick(); node.flags()
        clock[0] += .1
    assert node.core.mode == "navsat"
    assert node.navsat.running and len(messages) == 30
    assert "covariance" in node.fused_health_reason(clock[0])


def test_ten_minutes_without_gnss_preserves_output_and_real_growing_covariance(manager):
    node, clock, messages, transforms = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True); node.tick()
    assert node.core.mode == "fastlivo_rtk"
    flags, stages, statuses = [], [], []
    node.ready_pub = SimpleNamespace(publish=flags.append)
    node.stage_pub = SimpleNamespace(publish=stages.append)
    node.status_pub = SimpleNamespace(publish=statuses.append)
    node.quality = 0
    for index in range(1, 1801):
        clock[0] = 1000. + index/3.
        variances = (6.04*index/1800., 2.33*index/1800.)
        feed(node, clock, fused=True, gnss=False, fused_variances=variances)
        # NavSat is no longer available indoors, even as a stale fallback.
        node.samples.pop("navsat", None)
        node.tick(); node.flags()
        assert flags[-1].data is True
    assert len(messages) == 1801 and len(transforms) == 1801
    assert node.core.mode == "fastlivo_rtk" and not node.navsat.running
    assert messages[-1].pose.covariance[0] == pytest.approx(6.04)
    assert messages[-1].pose.covariance[7] == pytest.approx(2.33)
    assert stages[-1].data == "ready"
    assert "no fixed RTK" in statuses[-1].data


@pytest.mark.parametrize("failure", ["backend_false", "heartbeat", "receipt", "source", "process"])
def test_runtime_health_still_stops_on_real_failure_with_high_covariance(manager, failure):
    node, clock, messages, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True); node.tick()
    clock[0] += .1
    feed(node, clock, fused=True, gnss=False, fused_variances=(6.04, 2.33))
    node.tick()
    before = len(messages)
    if failure == "backend_false": node.fused_ready = False
    elif failure == "heartbeat": node.fused_ready_at = clock[0]-2.01
    elif failure == "receipt": node.samples["fastlivo_rtk"].received = clock[0]-4.01
    elif failure == "source": node.samples["fastlivo_rtk"].stamp = clock[0]-4.01
    else: node.fastlivo.running = False
    flags, stages, statuses = [], [], []
    node.ready_pub = SimpleNamespace(publish=flags.append)
    node.stage_pub = SimpleNamespace(publish=stages.append)
    node.status_pub = SimpleNamespace(publish=statuses.append)
    node.tick(); node.flags()
    assert flags[-1].data is False and len(messages) == before
    assert stages[-1].data == "wait_localization"
    assert statuses[-1].data != "FAST-LIVO2/RTK active"


def test_relaxed_handover_accepts_bounded_ins_overlap_after_fixed_gnss_loss(manager):
    node, clock, messages, _ = manager
    feed(node, clock); node.tick()
    assert node.navsat_initialized
    node.switch_callback(Trigger.Request(), Trigger.Response())
    clock[0] += 4.2
    for _ in range(15):
        feed(node, clock, fused=True, gnss=False, bias=.8, fused_variances=(.7, .7))
        assert not node.navsat_healthy(clock[0])
        node.tick()
        if node.core.mode == "fastlivo_rtk":
            break
        clock[0] += .1
    assert node.core.mode == "fastlivo_rtk"
    assert messages and node.fastlivo.running and not node.navsat.running


@pytest.mark.parametrize("failure", ["stale_ins", "uncertain_ins", "dead_ins"])
def test_gnss_loss_does_not_bypass_independent_ins_overlap_protections(manager, failure):
    node, clock, _, _ = manager
    feed(node, clock); node.tick()
    node.switch_callback(Trigger.Request(), Trigger.Response())
    clock[0] += 4.2
    feed(node, clock, fused=True, gnss=False)
    if failure == "stale_ins": node.samples["navsat"].received = clock[0]-1.0
    elif failure == "uncertain_ins": node.samples["navsat"].covariance[0,0] = 1.01
    else: node.navsat.running = False
    assert not node.check_handover(clock[0], clock[0], False)
    assert node.core.stable_since is None and node.core.mode == "navsat"


@pytest.mark.parametrize("invalid", ["negative_covariance", "indefinite_covariance", "position_jump", "rotation_jump", "nan_twist"])
def test_invalid_runtime_frame_stops_output_without_reanchoring_and_can_recover(manager, invalid):
    node, clock, messages, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True); node.tick()
    previous = node.samples["fastlivo_rtk"]
    bad = copy.deepcopy(node.messages["fastlivo_rtk"])
    clock[0] += .1
    bad.header.stamp.nanosec = 100000000
    if invalid == "negative_covariance": bad.pose.covariance[0] = -.01
    elif invalid == "indefinite_covariance":
        bad.pose.covariance[1] = bad.pose.covariance[6] = 1.
    elif invalid == "position_jump": bad.pose.pose.position.x += 5.
    elif invalid == "rotation_jump":
        bad.pose.pose.orientation.z, bad.pose.pose.orientation.w = 1., 0.
    else: bad.twist.twist.linear.x = float("nan")
    node.fused_callback(bad)
    node.fused_ready_at = clock[0]
    node.tick(); node.flags()
    assert node.samples["fastlivo_rtk"] is previous and len(messages) == 1
    assert not node.fused_healthy(clock[0])
    clock[0] += .1
    feed(node, clock, fused=True, gnss=False, fused_variances=(6.04, 2.33))
    node.tick(); node.flags()
    assert node.fused_healthy(clock[0]) and len(messages) == 2


def test_repeated_jump_cannot_become_valid_by_waiting(manager):
    node, clock, messages, _ = manager
    node.switch_callback(Trigger.Request(), Trigger.Response())
    node.core.stable_sec = 0.
    feed(node, clock, fused=True); node.tick()
    previous = node.samples["fastlivo_rtk"]
    for index in range(1, 61):
        clock[0] = 1000.+index*.1
        feed(node, clock, fused=True, bias=5., gnss=False, fused_variances=(6.04, 2.33))
        node.tick(); node.flags()
        assert not node.fused_healthy(clock[0])
        assert node.samples["fastlivo_rtk"] is previous
    assert len(messages) == 1


@pytest.mark.parametrize("invalid", ["frame", "quaternion", "covariance"])
def test_invalid_new_odometry_cannot_replace_a_good_sample(manager, invalid):
    node, clock, _, _ = manager
    feed(node, clock)
    previous = node.samples["navsat"]
    message = Odometry()
    message.header.frame_id = "map"; message.child_frame_id = "base_link"
    message.header.stamp.sec = 1000
    message.pose.pose.orientation.w = 1.
    if invalid == "frame": message.header.frame_id = "unrelated"
    elif invalid == "quaternion": message.pose.pose.orientation.w = 0.
    else: message.pose.covariance[0] = float("nan")
    node.navsat_callback(message)
    assert node.samples["navsat"] is previous


def test_cleanup_attempts_both_estimators_if_one_stop_fails(manager, monkeypatch):
    node, _, _, _ = manager

    def fail():
        raise RuntimeError("Synthetic shutdown failure")

    with monkeypatch.context() as patch:
        patch.setattr(node.fastlivo, "stop", fail)
        with pytest.raises(RuntimeError):
            node.close()
    assert not node.navsat.running
