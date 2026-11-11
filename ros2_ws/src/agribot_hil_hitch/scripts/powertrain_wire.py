"""Version 0x22 nominal powertrain extension of the isolated HIL protocol."""
import json
import math
import struct
import time
from wire import Lease, seal, validate

ENGINE, TRANSMISSION = 0x640, 0x641
SENSORS = (0x642, 0x643, 0x644, 0x645)
MODES = ('off', 'powershift_manual', 'powershift_auto', 'cvt_auto', 'cvt_manual')


def feedback_fresh(records, keys, now, timeout=.3):
    return all(key in records and math.isfinite(records[key]['at']) and
               0 <= now-records[key]['at'] <= timeout for key in keys)


def healthy_feedback(records, now):
    keys = ('powertrain_engine', 'powertrain_transmission', 'powertrain_vehicle', 'powertrain_status')
    if not feedback_fresh(records, keys, now):
        return False
    status = bytes.fromhex(records['powertrain_status']['data'])
    vehicle = bytes.fromhex(records['powertrain_vehicle']['data'])
    return status[0] == 0x22 and status[5] == 0 and vehicle[5] == 0


def parse_profile(text):
    values = json.loads(text)
    keys = {'mode', 'gear', 'engine_rpm', 'ratio', 'shift_ms', 'ratio_rate'}
    if not isinstance(values, dict) or set(values) != keys or values['mode'] not in MODES:
        raise ValueError('Expected a complete powertrain profile')
    mode = MODES.index(values['mode'])
    gear, rpm, ratio, shift, rate = (values[k] for k in ('gear', 'engine_rpm', 'ratio', 'shift_ms', 'ratio_rate'))
    if type(gear) is not int or type(rpm) is not int or type(shift) is not int or \
            type(ratio) not in (int, float) or type(rate) not in (int, float) or \
            not all(math.isfinite(x) for x in (ratio, rate)) or not 0 <= gear <= 4 or \
            not 800 <= rpm <= 2200 or rpm % 20 or not 200 <= shift <= 2000 or shift % 10 or \
            not 10 <= rate <= 300 or (mode == 1 and gear == 0) or (mode != 1 and gear != 0) or \
            (not 200 <= ratio <= 900 if mode == 4 else ratio != 0):
        raise ValueError('Powertrain parameters outside bench limits')
    return mode, gear, rpm, float(ratio), shift, float(rate)


def decode_actuators(a, b):
    validate(a); validate(b)
    throttle, ca, flags, version, sequence = struct.unpack('<HHBBB', a[:7])
    ratio, cb, gears, version_b, sequence_b = struct.unpack('<HHBBB', b[:7])
    mode, enabled, brake = flags & 7, bool(flags & 8), bool(flags & 16)
    ga, gb = gears & 15, gears >> 4
    if version != 0x22 or version_b != 0x22 or sequence != sequence_b or flags & ~31 or \
            mode > 4 or max(throttle, ca, cb) > 1000 or not 2000 <= ratio <= 9000 or max(ga, gb) > 4 or \
            (not enabled and (mode or throttle or ca or cb)) or (brake and (ca or cb)):
        raise ValueError('Invalid paired powertrain actuation')
    return [throttle*.001, ca*.001, cb*.001, ratio*.1, ga, gb, mode, float(enabled), float(brake)]


class ActuatorPair:
    def __init__(self):
        self.pending = {}
        self.lease = Lease()
        self.value = [0., 0., 0., 640., 0., 0., 0., 0., 1.]

    def accept(self, identity, data, now):
        validate(data)
        self.pending[identity] = (data, now)
        if ENGINE not in self.pending or TRANSMISSION not in self.pending:
            return False
        a, at = self.pending[ENGINE]; b, bt = self.pending[TRANSMISSION]
        if a[6] != b[6] or max(now-at, now-bt) > .1:
            return False
        value = decode_actuators(a, b)
        if self.lease.accept(a, now, not value[7] and bool(value[8])):
            self.value = value
            return True
        return False

    def current(self, now):
        return self.value if self.lease.fresh(now) else [0., 0., 0., 640., 0., 0., 0., 0., 1.]


def sensors(state, counter, fault=0):
    if len(state) != 20 or not all(math.isfinite(x) for x in state):
        raise ValueError('Invalid powertrain plant state')
    mode, rpm, input_rpm, output_rpm, ratio, pa, pb, throttle, torque, _, speed, ga, gb, gear, load, brake, _, _, _, enabled = state
    if not 0 <= rpm <= 3000 or not 150 <= ratio <= 950 or min(pa, pb) < 0 or max(pa, pb) > 2100 or \
            abs(speed) > 2 or not 0 <= mode <= 4 or not 0 <= gear <= 4 or not 0 <= load <= 3000:
        raise ValueError('Powertrain feedback outside nominal model bounds')
    result = [struct.pack('<HHhB', round(rpm), round(abs(input_rpm)), round(torque*10), counter),
              struct.pack('<HHHB', round(ratio*10), round(pa), round(pb), counter),
              struct.pack('<hhBBB', round(speed*1000), round(output_rpm*10), int(gear) | int(mode)<<4, fault, counter),
              struct.pack('<HBBBBB', round(load), min(100, round(abs(torque)/70*100)), int(brake), int(enabled), 0x22, counter)]
    return [seal(value) for value in result]
