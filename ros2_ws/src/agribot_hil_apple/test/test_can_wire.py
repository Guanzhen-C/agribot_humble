from functools import reduce
from operator import xor
import struct
import pytest

from agribot_hil_apple.can_wire import CAN_FRAME, decode_command, feedback_payload, int16
from agribot_hil_apple import can_wire


def test_socketcan_filter_does_not_select_error_only_list(monkeypatch):
    class FakeSocket:
        def setsockopt(self, level, option, value):
            self.filters = value

        def bind(self, address):
            pass

        def settimeout(self, value):
            pass

        def close(self):
            pass

    sock = FakeSocket()
    monkeypatch.setattr(can_wire, 'check_usb_can', lambda *args: None)
    monkeypatch.setattr(can_wire.socket, 'socket', lambda *args: sock)
    bus = can_wire.CanSocket('can0', 'test', [0x181])
    identifier, mask = struct.unpack('=II', sock.filters)
    assert identifier == 0x181
    assert mask == 0xc00007ff
    assert not mask & 0x20000000
    bus.close()


def test_ackermann_classic_frame_and_endianness():
    assert CAN_FRAME.size == 16
    assert decode_command(bytes.fromhex('01900000ff060000')) == (.4, -.25)
    for invalid in (b'', bytes.fromhex('0000010000000000'), b'\0' * 9):
        with pytest.raises(ValueError):
            decode_command(invalid)


def test_telemetry_matches_physical_decoder_layout():
    payload = feedback_payload(.4, -.3, voltage=24.123, stopped=True)
    assert len(payload) == 24 and payload[:2] == b'\x7b\x01' and payload[-1] == 0x7d
    assert struct.unpack_from('>hhh', payload, 2) == (400, 0, -300)
    assert struct.unpack_from('>H', payload, 20)[0] == 24123
    assert payload[22] == reduce(xor, payload[:22], 0)
    assert int16(.5) == 1 and int16(-.5) == -1
    assert int16(1e6) == 32767 and int16(-1e6) == -32768
    with pytest.raises(ValueError):
        feedback_payload(voltage=float('nan'))
