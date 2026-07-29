import importlib.util
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
GUI_PATH = PACKAGE_ROOT / "scripts" / "wheeltec_car_gui.py"


def load_gui_module():
    spec = importlib.util.spec_from_file_location("wheeltec_car_gui", GUI_PATH)
    module = importlib.util.module_from_spec(spec)
    assert spec.loader is not None
    sys.modules[spec.name] = module
    spec.loader.exec_module(module)
    return module


def test_c50c_command_and_zqwl_packet_encoding():
    gui = load_gui_module()
    payload = gui.encode_command(0.10, 0.0, -0.10)
    assert payload.hex() == "00640000ff9c0000"
    assert (
        gui.encode_can_packet(gui.COMMAND_ID, payload).hex()
        == "5a08000000018100640000ff9c0000a5"
    )


def test_decoder_handles_status_fragmentation_and_feedback_coalescing():
    gui = load_gui_module()
    status = bytes.fromhex(
        "5afe0014003c00000000000000000000000020000000000000000000000000a5"
    )
    feedback = b"".join(
        gui.encode_can_packet(can_id, bytes((can_id & 0xFF,)) + bytes(7))
        for can_id in sorted(gui.TELEMETRY_IDS)
    )
    decoder = gui.ZqwlFrameDecoder()
    assert decoder.feed(b"\x49\x3b" + status[:11]) == []
    assert decoder.feed(status[11:] + feedback[:19]) == [
        (0x101, b"\x01" + bytes(7))
    ]
    assert decoder.feed(feedback[19:]) == [
        (0x102, b"\x02" + bytes(7)),
        (0x103, b"\x03" + bytes(7)),
    ]
    assert decoder.invalid_frames == 0


def test_decoder_resynchronizes_after_malformed_packet():
    gui = load_gui_module()
    decoder = gui.ZqwlFrameDecoder()
    valid = gui.encode_can_packet(0x101, bytes(8))
    assert decoder.feed(b"\x5a\xfd\x00\x00\x00" + valid) == [
        (0x101, bytes(8))
    ]
    assert decoder.invalid_frames > 0


def test_dry_run_link_never_opens_hardware_and_records_commands():
    gui = load_gui_module()
    link = gui.CanLink("/not/a/device", dry_run=True)
    assert link.connect()
    assert link.fd is None
    assert link.telemetry_alive()
    assert link.send(gui.encode_command(-0.10, 0.0, 0.12))
    assert link.tx_count == 1
    assert link.dry_run_payloads == [
        bytes.fromhex("ff9c000000780000")
    ]
