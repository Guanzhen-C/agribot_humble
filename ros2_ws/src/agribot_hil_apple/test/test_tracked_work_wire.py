import math
import struct

import pytest

from agribot_hil_apple.tracked_work_wire import (
    CounterLease, body_velocity, checked, decode_drive, decode_work, drive_feedback, plant_feedback,
)


def test_forward_velocity_remains_positive_after_orchard_u_turn():
    assert body_velocity([0, 0, 1, 0], [-.3, 0, 0], [0, 0, .4]) == pytest.approx((.3, .4))
    assert body_velocity([0, 0, math.sqrt(.5), math.sqrt(.5)], [0, .3, 0], [0, 0, .4]) == pytest.approx((.3, .4))


def test_native_both_work_protocols_and_little_endian():
    data = checked([0x83, 0x10, 0x27, 0, 0, 0, 0, 0], 15)
    assert decode_work(data) == (3, 8, 10000)
    for raw in ([0x99, 0, 0, 0, 0, 0, 0, 0], [0, 0x11, 0x27, 0, 0, 0, 0, 0]):
        with pytest.raises(ValueError):
            decode_work(checked(raw, 0))


def test_drive_matches_real_measured_command_curve():
    data = checked([0, 27, 27, 1, 0, 0, 0, 0], 1)
    speed, yaw, lamp = decode_drive(data)
    assert speed == pytest.approx(.4245)
    assert yaw == 0 and lamp
    data = checked([0, 229, 27, 0, 0, 0, 0, 0], 2)
    assert decode_drive(data)[1] == pytest.approx(2 * .4245 / .6)


def test_reconnect_requires_zero_and_duplicate_does_not_refresh():
    lease = CounterLease(.3)
    zero = checked(bytes(8), 15)
    moving = checked([0, 10, 10, 0, 0, 0, 0, 0], 0)
    assert not lease.accept(moving, 0, False)
    assert lease.accept(zero, .01, True)
    assert lease.accept(moving, .1, False)
    assert not lease.accept(moving, .2, False)
    assert not lease.fresh(.401)
    assert not lease.accept(checked(moving, 1), .5, False)
    assert lease.accept(checked(zero, 4), .6, True)


def test_feedback_uses_actual_motion_not_command():
    state, left, right = drive_feedback(.42, .2, True, 7)
    assert state[0] == 0x532 and state[1][0] == 9
    assert struct.unpack_from('<h', right[1], 1)[0] > struct.unpack_from('<h', left[1], 1)[0]
    work = plant_feedback(0, 0, (3, 4), 7)
    assert work[0][1][1] == 0
    assert work[1][1][0] == 3 and work[1][1][1:3] == bytes(2)
    with pytest.raises(ValueError):
        plant_feedback(math.nan, 0, (3, 4), 0)
