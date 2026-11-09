from agribot_hil_apple.tracked_drive_guard import drive_enabled
from agribot_hil_apple.tracked_work_wire import CounterLease, checked


def test_physical_command_loss_requires_neutral_then_recovers():
    lease = CounterLease(.5)
    zero = checked(bytes(8), 0)
    moving = bytearray(8)
    moving[1:3] = bytes((19, 19))
    assert lease.accept(zero, 0., True)
    assert lease.accept(checked(moving, 1), .2, False)
    assert not lease.accept(checked(moving, 4), .8, False)
    status = dict(motion_authorized=True, truth_fresh=True, fault='',
                  drive_lease_fresh=lease.fresh(.8))
    assert not drive_enabled(status, .1, .1)
    assert lease.accept(checked(zero, 5), 1., True)
    status['drive_lease_fresh'] = lease.fresh(1.1)
    assert drive_enabled(status, .1, .1)
    assert lease.accept(checked(moving, 6), 1.2, False)


def test_stale_feedback_command_or_fault_cannot_enable_motion():
    ready = dict(motion_authorized=True, truth_fresh=True, drive_lease_fresh=True, fault='')
    assert drive_enabled(ready, .1, .1)
    assert not drive_enabled(ready, 2., .1)
    assert not drive_enabled(ready, .1, .3)
    assert not drive_enabled(ready, -.1, .1)
    assert not drive_enabled({}, 0., 0.)
    for key, value in [('motion_authorized', False), ('truth_fresh', False),
                       ('drive_lease_fresh', False), ('fault', 'bus fault')]:
        assert not drive_enabled(dict(ready, **{key: value}), .1, .1)
