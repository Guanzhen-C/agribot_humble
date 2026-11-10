"""Three-in-one native CAN codec for the isolated tracked HIL plant emulator."""
import math
import struct
from functools import reduce
from operator import xor


TRACK_WIDTH = .60
RPM_TO_MPS = .000436332313


def body_velocity(quaternion, linear, angular):
    if not all(math.isfinite(v) for values in (quaternion, linear, angular) for v in values):
        raise ValueError('Non-finite ground-truth transform')
    norm = math.sqrt(sum(v*v for v in quaternion))
    if norm < 1e-8:
        raise ValueError('Invalid ground-truth orientation')
    x, y, z, w = (value/norm for value in quaternion)
    forward = (1-2*(y*y+z*z), 2*(x*y+w*z), 2*(x*z-w*y))
    up = (2*(x*z+w*y), 2*(y*z-w*x), 1-2*(x*x+y*y))
    return sum(a*b for a, b in zip(forward, linear)), sum(a*b for a, b in zip(up, angular))


def checked(payload, counter):
    data = bytearray(payload)
    if len(data) != 8:
        raise ValueError('Expected eight bytes')
    data[6] = counter & 15
    data[7] = reduce(xor, data[:7], 0)
    return bytes(data)


def validate(data):
    if len(data) != 8 or reduce(xor, data[:7], 0) != data[7]:
        raise ValueError('Invalid native length/checksum')


def percent_speed(percent):
    if abs(percent) > 54:
        raise ValueError('Beyond the tested 1600-level tracked HIL command limit')
    level = abs(percent) * 30.
    speed = level * .42 / 800 if level <= 800 else .42 + (level - 800) * .36 / 800
    return math.copysign(speed, percent)


def decode_drive(data):
    validate(data)
    if data[0] or data[3] & ~1 or any(data[4:6]) or data[6] & 0xf0:
        raise ValueError('Unsupported drive fields')
    left, right = (percent_speed(value) for value in struct.unpack('bb', data[1:3]))
    return (left + right) / 2, (right - left) / TRACK_WIDTH, bool(data[3] & 1)


def decode_work(data):
    validate(data)
    fan, pump = data[0] & 15, data[0] >> 4
    shredder = int.from_bytes(data[1:3], 'little')
    if fan > 8 or pump > 8 or shredder > 10000 or any(data[3:6]) or data[6] & 0xf0:
        raise ValueError('Invalid implement command fields')
    return fan, pump, shredder


class CounterLease:
    """Duplicates cannot renew a lease; a restarted controller must send zero."""
    def __init__(self, seconds):
        self.seconds = seconds
        self.at = -math.inf
        self.counter = None

    def accept(self, data, now, neutral):
        validate(data)
        current = data[6] & 15
        if now - self.at >= self.seconds:
            if not neutral:
                return False
        elif self.counter is not None and not 1 <= (current - self.counter) % 16 <= 8:
            return False
        self.counter, self.at = current, now
        return True

    def fresh(self, now):
        return now - self.at < self.seconds


def motor_feedback(rpm, counter, gear=None):
    if not math.isfinite(rpm):
        raise ValueError('Non-finite actual motor speed')
    data = bytearray(8)
    rounded = max(-32768, min(32767, round(rpm)))
    if gear is not None:
        if not 0 <= gear <= 8:
            raise ValueError('Invalid gear')
        data[0] = gear | (16 if rounded < 0 else 0)
    struct.pack_into('<h', data, 1, rounded)
    data[3] = 75
    if gear is None:
        data[5] = 65  # Simulated 25 C; not a physical sensor measurement.
    return checked(data, counter)


def drive_feedback(speed, yaw_rate, headlight, counter):
    if not all(math.isfinite(value) for value in (speed, yaw_rate)):
        raise ValueError('Non-finite actual chassis velocity')
    state = bytearray(8)
    state[0] = 1 | (8 if abs(speed) + abs(yaw_rate) > .001 else 0)
    state[1] = 4 if headlight else 0
    struct.pack_into('<H', state, 2, 750)
    return [(0x532, checked(state, counter)),
            (0x533, motor_feedback((speed - yaw_rate * TRACK_WIDTH / 2) / RPM_TO_MPS, counter)),
            (0x534, motor_feedback((speed + yaw_rate * TRACK_WIDTH / 2) / RPM_TO_MPS, counter))]


def plant_feedback(fan_rpm, pump_rpm, gears, counter, fault=False):
    common = bytearray(8)
    common[1] = int(fault) | (8 if abs(fan_rpm) + abs(pump_rpm) > 1 else 0)
    return [(0x5b0, checked(common, counter)),
            (0x5b2, motor_feedback(fan_rpm, counter, gears[0])),
            (0x5b3, motor_feedback(pump_rpm, counter, gears[1]))]


def mounted_work(values, kind):
    if kind not in ('sprayer', 'shredder'):
        raise ValueError('Unknown mounted implement')
    if (kind == 'sprayer' and values[2]) or (kind == 'shredder' and any(values[:2])):
        raise ValueError('Command addresses an unmounted implement')
    return tuple(values)


def work_targets(values, kind):
    values = mounted_work(values, kind)
    # The spreadsheet defines shredder input as 0..10000 = 0..100.00 percent.
    # 3600 rpm is the simulation curve limit, not a measured calibration.
    return [values[2] * .36] if kind == 'shredder' else [values[0] * 300., values[1] * 200.]


def shredder_feedback(rpm, counter, fault=False):
    if not math.isfinite(rpm):
        raise ValueError('Non-finite actual shredder speed')
    common = bytearray(8)
    common[0] = 1
    common[2] = int(fault) | (8 if abs(rpm) > 1 else 0)
    motor = bytearray(8)
    motor[0] = int(fault)
    struct.pack_into('<h', motor, 1, max(-32768, min(32767, round(rpm))))
    motor[5] = 65  # Explicit simulated 25 C, not a physical temperature reading.
    return [(0x5b0, checked(common, counter)), (0x5b4, checked(motor, counter))]
