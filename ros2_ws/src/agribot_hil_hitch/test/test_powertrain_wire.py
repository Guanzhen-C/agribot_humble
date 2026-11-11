import json
from pathlib import Path
import struct
import sys
import pytest
sys.path.insert(0, str(Path(__file__).parents[1]/'scripts'))
from powertrain_wire import ActuatorPair, ENGINE, TRANSMISSION, parse_profile, decode_actuators, sensors, healthy_feedback, feedback_fresh
from wire import seal


def pair(sequence, enabled=False):
    return (seal(struct.pack('<HHBBB', 100 if enabled else 0, 1000 if enabled else 0,
                             9 if enabled else 16, 0x22, sequence)),
            seal(struct.pack('<HHBBB', 6400, 0, 1 if enabled else 0, 0x22, sequence)))


def profile(**changes):
    value = dict(mode='cvt_auto', gear=0, engine_rpm=1600, ratio=0, shift_ms=600, ratio_rate=150)
    value.update(changes); return json.dumps(value)


def test_profile_and_manual_types():
    assert parse_profile(profile()) == (3, 0, 1600, 0., 600, 150.)
    assert parse_profile(profile(mode='powershift_manual', gear=3))[1] == 3
    assert parse_profile(profile(mode='cvt_manual', ratio=350))[3] == 350


@pytest.mark.parametrize('values', [dict(mode='CVT'), dict(gear=1), dict(ratio=300),
    dict(engine_rpm=999), dict(engine_rpm=2400), dict(shift_ms=0), dict(ratio_rate=0),
    dict(mode='powershift_manual', gear=0), dict(mode='cvt_manual', ratio=50), dict(gear=True)])
def test_bad_profiles(values):
    with pytest.raises(ValueError):
        parse_profile(profile(**values))


def test_paired_actuators_need_neutral_and_matching_sequence():
    state = ActuatorPair()
    a, b = pair(1, True)
    assert not state.accept(ENGINE, a, 1)
    assert not state.accept(TRANSMISSION, b, 1)
    a, b = pair(2)
    assert not state.accept(ENGINE, a, 1.01)
    assert state.accept(TRANSMISSION, b, 1.01)
    a, b = pair(3, True)
    assert not state.accept(ENGINE, a, 1.02)
    assert state.accept(TRANSMISSION, b, 1.02)
    assert state.current(1.03)[7] == 1
    assert state.current(1.3)[7] == 0 and state.current(1.3)[8] == 1
    assert not state.accept(TRANSMISSION, b, 1.31)


def test_crc_and_version_and_pair_mismatch():
    a, b = pair(1, True)
    with pytest.raises(ValueError): decode_actuators(a, pair(2, True)[1])
    with pytest.raises(ValueError): decode_actuators(bytes(8), b)
    damaged = bytearray(a); damaged[0] ^= 1
    with pytest.raises(ValueError): decode_actuators(damaged, b)


def test_sensor_encoding_and_fault_bit():
    state = [3, 1600, 1550, 6, 260, 1900, 0, .2, 20, 1600, .45, 1, 0, 0, 600, 0, 50, 2, 260, 1]
    encoded = sensors(state, 255, 1)
    assert len(encoded) == 4 and all(len(value) == 8 for value in encoded)
    assert encoded[2][5] == 1 and all(value[6] == 255 for value in encoded)
    assert struct.unpack('<H', encoded[1][:2])[0] == 2600


def test_feedback_requires_all_fresh_fault_free_sources():
    keys = ('powertrain_engine', 'powertrain_transmission', 'powertrain_vehicle', 'powertrain_status')
    status = seal(bytes((0x22, 3, 0, 0, 0, 0, 1))).hex()
    records = {key: dict(at=1., data=status if key.endswith('status') else seal(bytes(7)).hex()) for key in keys}
    assert healthy_feedback(records, 1.1)
    assert not healthy_feedback(records, 1.31)
    missing = dict(records); del missing['powertrain_engine']
    assert not healthy_feedback(missing, 1.1)
    records['powertrain_status']['data'] = seal(bytes((0x22, 0, 0, 0, 0, 1, 2))).hex()
    assert not healthy_feedback(records, 1.1)


def test_stale_snapshot_is_not_fresh_just_because_republished():
    records = {'sensor': dict(at=1., data='unused')}
    assert feedback_fresh(records, ('sensor',), 1.1)
    assert not feedback_fresh(records, ('sensor',), 2.)
    assert not feedback_fresh(records, ('sensor',), .9)
    assert not feedback_fresh(records, ('sensor',), float('inf'))
