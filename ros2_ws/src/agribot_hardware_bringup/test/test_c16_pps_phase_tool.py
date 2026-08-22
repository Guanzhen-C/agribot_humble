import importlib.util
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
TOOL_PATH = PACKAGE_ROOT / "scripts" / "c16_pps_phase_tool.py"


def load_tool():
    spec = importlib.util.spec_from_file_location("c16_pps_phase_tool", TOOL_PATH)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


TOOL = load_tool()


def make_device_packet(target=0, error=9710, valid=True):
    packet = bytearray(TOOL.PACKET_SIZE)
    packet[:8] = TOOL.DEVICE_HEADER
    packet[-2:] = TOOL.PACKET_TAIL
    packet[TOOL.CLOCK_SOURCE_OFFSET : TOOL.CLOCK_SOURCE_OFFSET + 2] = (1).to_bytes(
        2, "big"
    )
    packet[TOOL.TARGET_ANGLE_OFFSET : TOOL.TARGET_ANGLE_OFFSET + 2] = target.to_bytes(
        2, "big"
    )
    raw_error = error & 0x7FFF
    if not valid:
        raw_error |= 0x8000
    packet[TOOL.ANGLE_ERROR_OFFSET : TOOL.ANGLE_ERROR_OFFSET + 2] = (
        raw_error.to_bytes(2, "big")
    )
    return bytes(packet)


def test_parse_current_c16_pps_status():
    status = TOOL.parse_device_packet(make_device_packet())
    assert status["clock_source"] == 1
    assert status["target_angle_deg"] == 0.0
    assert status["pps_valid"] is True
    assert status["angle_error_deg"] == 97.1


def test_build_config_packet_only_changes_header_target_and_tail():
    original = make_device_packet()
    configured = TOOL.build_config_packet(original, 90.0)
    assert len(configured) == TOOL.PACKET_SIZE
    assert configured[:8] == TOOL.CONFIG_HEADER
    assert configured[-2:] == TOOL.PACKET_TAIL
    assert int.from_bytes(
        configured[TOOL.TARGET_ANGLE_OFFSET : TOOL.TARGET_ANGLE_OFFSET + 2],
        "big",
    ) == 9000
    changed = {
        index for index, (before, after) in enumerate(zip(original, configured))
        if before != after
    }
    expected = set(range(8)) | {
        TOOL.TARGET_ANGLE_OFFSET,
        TOOL.TARGET_ANGLE_OFFSET + 1,
    }
    assert changed == expected


def test_parse_negative_angle_error():
    status = TOOL.parse_device_packet(make_device_packet(error=-250))
    assert status["angle_error_deg"] == -2.5
