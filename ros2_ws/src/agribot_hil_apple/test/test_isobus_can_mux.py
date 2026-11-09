import pytest

from agribot_hil_apple.isobus_can_mux import Decoder, Frame, allowed, packet, pgn


def test_manufacturer_extended_data_layout():
    frame = Frame(0x18eeff90, bytes.fromhex('123456789abcdef0'), True)
    assert packet(frame).hex() == '5a084418eeff90123456789abcdef0a5'
    decoder = Decoder()
    raw = packet(frame)
    assert decoder.feed(raw[:6]) == []
    assert decoder.feed(raw[6:]) == [frame]


def test_drive_and_isobus_coexist_without_identifier_truncation():
    drive = Frame(0x514, bytes(8))
    claim = Frame(0x18eeff90, bytes(8), True)
    request = Frame(0x18ea9190, bytes.fromhex('00ee00'), True)
    decoder = Decoder()
    assert decoder.feed(packet(drive) + packet(claim) + packet(request)) == [drive, claim, request]
    assert allowed(drive, True) and allowed(claim, True) and allowed(request, True)
    assert pgn(0x18ef9190) == 0xef00
    assert pgn(0x18ff5291) == 0xff52
    assert not allowed(Frame(0x580, bytes(8)), True)
    assert not allowed(drive, False)
    assert allowed(Frame(0x532, bytes(8)), False)


def test_fd_remote_and_wrong_channel_are_not_farm_commands():
    frame = packet(Frame(0x18ef9190, bytes(8), True))
    for position, mask in ((2, 1), (2, 2), (2, 8), (2, 128), (3, 128)):
        malformed = bytearray(frame); malformed[position] |= mask
        decoder = Decoder()
        assert decoder.feed(malformed) == [] and decoder.invalid == 1


def test_status_and_resynchronization():
    heartbeat = bytes((0x5a, 0xfe)) + bytes(29) + b'\xa5'
    frame = Frame(0x533, bytes(8))
    decoder = Decoder()
    assert decoder.feed(b'garbage' + heartbeat + packet(frame)) == [frame]
    with pytest.raises(ValueError):
        Frame(0x1800, bytes(8))
