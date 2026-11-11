"""Bench-only protocol 0x21. Not an extension of either real vehicle protocol."""
import math
import struct

VERSION, DRIVE, DRIVE_STATE, VALVE, SENSOR = 0x21, 0x620, 0x621, 0x630, 0x631


def crc(data):
    value = 255
    for byte in data:
        value ^= byte
        for _ in range(8):
            value = ((value << 1) ^ (0x1d if value & 128 else 0)) & 255
    return value ^ 255


def seal(data):
    if len(data) != 7:
        raise ValueError('Expected seven bytes before CRC')
    return data+bytes([crc(data)])


def validate(data):
    if len(data) != 8 or crc(data[:7]) != data[7]:
        raise ValueError('Invalid hitch HIL length/CRC')


def drive_command(speed, steering, enabled, sequence):
    if not all(math.isfinite(x) for x in (speed, steering)) or not 0 <= speed <= .5 or abs(steering) > .61:
        raise ValueError('Outside dedicated tractor bench speed/steering limits')
    if not enabled and (speed or steering):
        raise ValueError('Disabled drive must be neutral')
    return seal(struct.pack('<hhBBB', round(speed*1000), round(steering*10000),
                            int(enabled), VERSION, sequence & 255))


def decode_drive(data):
    validate(data)
    speed, steering, enable, version, _ = struct.unpack('<hhBBB', data[:7])
    if version != VERSION or enable > 1 or not 0 <= speed <= 500 or abs(steering) > 6100 or \
            (not enable and (speed or steering)):
        raise ValueError('Invalid dedicated drive command')
    return speed*.001, steering*.0001, bool(enable)


def decode_valve(data):
    validate(data)
    value, enabled, mode, fault, version, _ = struct.unpack('<hBBBBB', data[:7])
    if version != VERSION or abs(value) > 1000 or enabled > 1 or mode > 3:
        raise ValueError('Invalid valve output')
    if (not enabled or fault or mode == 0) and value:
        raise ValueError('Nonzero valve in HOLD/fault/disabled output')
    return value, bool(enabled)


def sensor_feedback(position, velocity, fault, counter):
    if not all(math.isfinite(v) for v in (position, velocity)) or not -100 <= position <= 10100:
        raise ValueError('Invalid physical lift measurement')
    limits = int(position <= 20) | (int(position >= 9980) << 1)
    return seal(struct.pack('<HhBBB', round(max(0, min(10000, position))),
                            round(max(-32768, min(32767, velocity))), limits, fault, counter & 255))


class Lease:
    def __init__(self, seconds=.2):
        self.seconds, self.at, self.sequence = seconds, -math.inf, 0

    def fresh(self, now):
        return now-self.at < self.seconds

    def accept(self, data, now, neutral):
        validate(data)
        if not self.fresh(now):
            if not neutral:
                return False
        elif not 1 <= (data[6]-self.sequence) % 256 <= 127:
            return False
        self.at, self.sequence = now, data[6]
        return True
