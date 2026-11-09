import json

import pytest

from agribot_hil_apple.isobus_work_client import parse_target


def test_spraying_default_never_enables_unmounted_shredder():
    target = dict(fan_gear=8, pump_gear=8, shredder_level=0)
    assert parse_target(json.dumps(target)) == [8, 8, 0]
    target['shredder_level'] = 10000
    with pytest.raises(ValueError):
        parse_target(json.dumps(target))
    assert parse_target(json.dumps(target), True) == [8, 8, 10000]


@pytest.mark.parametrize('value', [True, -1, 9, '3', 1.5, None])
def test_invalid_gear_rejected(value):
    with pytest.raises(ValueError):
        parse_target(json.dumps(dict(fan_gear=value, pump_gear=0, shredder_level=0)))


def test_fields_must_be_explicit_and_known():
    with pytest.raises(ValueError):
        parse_target('{"fan_gear": 1}')
    with pytest.raises(ValueError):
        parse_target('[]')
