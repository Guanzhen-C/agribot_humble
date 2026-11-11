import importlib.util
from pathlib import Path
import struct

import pytest

spec = importlib.util.spec_from_file_location('hitch_wire_test_module', Path(__file__).parents[1]/'scripts/wire.py')
wire = importlib.util.module_from_spec(spec)
spec.loader.exec_module(wire)


def test_crc_reference_and_round_trip():
    assert wire.crc(b'123456789') == 0x4b
    assert wire.decode_drive(wire.drive_command(.3, -.4, True, 255)) == (.3, -.4, True)


@pytest.mark.parametrize('speed,steer,enabled', [(.6, 0, True), (-.1, 0, True),
                         (.2, .7, True), (.1, 0, False), (float('nan'), 0, True)])
def test_drive_limits(speed, steer, enabled):
    with pytest.raises(ValueError):
        wire.drive_command(speed, steer, enabled, 1)


def test_crc_corruption_and_real_vehicle_payload_rejected():
    data = bytearray(wire.drive_command(.2, 0., True, 3))
    data[0] ^= 1
    with pytest.raises(ValueError):
        wire.decode_drive(data)
    with pytest.raises(ValueError):
        wire.decode_drive(bytes(8))


def test_valve_hold_cannot_lower():
    assert wire.decode_valve(wire.seal(struct.pack('<hBBBBB', 0, 0, 0, 0, 0x21, 0))) == (0, False)
    with pytest.raises(ValueError):
        wire.decode_valve(wire.seal(struct.pack('<hBBBBB', -100, 1, 0, 0, 0x21, 1)))


def test_sensor_limits_and_finite_validation():
    lower = wire.sensor_feedback(0., 0., 0, 2)
    upper = wire.sensor_feedback(10000., 0., 0, 3)
    assert lower[4] == 1 and upper[4] == 2
    with pytest.raises(ValueError):
        wire.sensor_feedback(float('nan'), 0, 0, 1)


def test_lease_requires_neutral_and_rejects_duplicate():
    lease = wire.Lease()
    a = wire.drive_command(.2, 0., True, 2)
    assert not lease.accept(a, 1., False)
    assert lease.accept(wire.drive_command(0., 0., False, 1), 1., True)
    assert lease.accept(a, 1.1, False)
    assert not lease.accept(a, 1.2, False)
    assert not lease.fresh(1.31)
    assert not lease.accept(wire.drive_command(.2, 0., True, 3), 1.32, False)


def test_lease_sequence_rollover():
    lease = wire.Lease()
    assert lease.accept(wire.drive_command(0., 0., False, 255), 1., True)
    assert lease.accept(wire.drive_command(.2, 0., True, 0), 1.1, False)


def test_message_allocation_does_not_reuse_real_protocol_ids():
    assert not {wire.DRIVE, wire.DRIVE_STATE, wire.VALVE, wire.SENSOR} & \
        {0x181, 0x101, 0x102, 0x103, 0x514, 0x532, 0x533, 0x534, 0x580, 0x5b0, 0x5b2, 0x5b3, 0x5b4}
