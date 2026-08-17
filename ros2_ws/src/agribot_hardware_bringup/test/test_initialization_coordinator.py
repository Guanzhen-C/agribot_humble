import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from initialization_coordinator import (  # noqa: E402
    InitializationPolicy,
    MANUAL_REFINING,
    MANUAL_REQUIRED,
    READY,
    RTK_REFINING,
    VISUAL_REFINING,
    WAIT_RTK,
    WAIT_VISUAL,
)


def action_kinds(actions):
    return [action.kind for action in actions]


def policy(*, rtk=True, visual=True):
    return InitializationPolicy(
        rtk_enabled=rtk,
        visual_enabled=visual,
        rtk_wait_sec=10.0,
        visual_wait_sec=5.0,
        now=0.0,
    )


def test_rtk_is_tried_first_and_acceptance_stops_fallback():
    state = policy()
    assert state.stage == WAIT_RTK
    assert action_kinds(state.receive_rtk(1.0)) == ["forward_rtk"]
    assert state.stage == RTK_REFINING
    assert action_kinds(state.attempt_result(True, 2.0)) == [
        "request_visual",
        "ready",
    ]
    assert state.stage == READY
    assert state.source == "rtk"
    assert state.tick(100.0) == []


def test_missing_or_rejected_rtk_falls_back_to_visual_candidates():
    state = policy()
    assert action_kinds(state.tick(10.1)) == ["request_visual"]
    assert state.stage == WAIT_VISUAL
    assert action_kinds(state.receive_visual_candidates(2, 11.0)) == [
        "request_visual",
        "try_visual",
    ]
    assert state.stage == VISUAL_REFINING
    actions = state.attempt_result(False, 12.0)
    assert [(item.kind, item.candidate_index) for item in actions] == [
        ("try_visual", 1)
    ]
    assert state.attempt_result(True, 13.0)[-1].kind == "ready"
    assert state.source == "visual"


def test_unavailable_or_exhausted_visual_falls_back_to_manual():
    state = policy()
    state.set_visual_available(False, 1.0)
    actions = state.tick(10.1)
    assert action_kinds(actions) == ["request_visual", "manual_required"]
    assert state.stage == MANUAL_REQUIRED
    assert action_kinds(state.receive_manual(11.0)) == ["forward_manual"]
    assert state.stage == MANUAL_REFINING
    assert action_kinds(state.attempt_result(False, 12.0)) == [
        "manual_required"
    ]
    assert state.stage == MANUAL_REQUIRED


def test_manual_input_is_ignored_until_automatic_sources_finish():
    state = policy()
    assert state.receive_manual(1.0) == []
    assert state.stage == WAIT_RTK
    no_rtk = policy(rtk=False, visual=False)
    assert no_rtk.stage == MANUAL_REQUIRED
    assert action_kinds(no_rtk.receive_manual(1.0)) == ["forward_manual"]


def test_active_registration_never_times_out_into_an_overlapping_attempt():
    state = policy()
    state.receive_rtk(1.0)
    assert state.stage == RTK_REFINING
    assert state.tick(10_000.0) == []
    assert state.stage == RTK_REFINING

    state.attempt_result(False, 10_001.0)
    state.receive_visual_candidates(1, 10_002.0)
    assert state.stage == VISUAL_REFINING
    assert state.tick(20_000.0) == []
    assert state.stage == VISUAL_REFINING
