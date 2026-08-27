import importlib.util
import sys
from pathlib import Path


PACKAGE_ROOT = Path(__file__).parents[1]
SCRIPTS_DIR = PACKAGE_ROOT / "scripts"
GUI_PATH = SCRIPTS_DIR / "differential_car_gui.py"


def load_gui_module():
    sys.path.insert(0, str(SCRIPTS_DIR))
    try:
        spec = importlib.util.spec_from_file_location(
            "differential_car_gui", GUI_PATH
        )
        module = importlib.util.module_from_spec(spec)
        assert spec.loader is not None
        sys.modules[spec.name] = module
        spec.loader.exec_module(module)
        return module
    finally:
        sys.path.remove(str(SCRIPTS_DIR))


def with_checksum(gui, values):
    payload = bytearray(values)
    payload[7] = gui.xor_checksum(payload[:7])
    return bytes(payload)


def test_3000_level_maps_to_protocol_percentage_and_1_1_mps():
    gui = load_gui_module()
    assert gui.MAX_DRIVE_LEVEL == 3000
    assert gui.MAX_SPEED_MPS == 1.10
    assert gui.drive_level_to_percent(3000) == 100
    assert gui.drive_level_to_percent(-3000) == -100
    assert gui.drive_level_to_speed(3000) == 1.10
    assert gui.encode_command(3000, -3000, 2).hex() == (
        "00649c00000002fa"
    )


def test_command_is_saturated_and_has_rolling_counter_and_checksum():
    gui = load_gui_module()
    payload = gui.encode_command(9000, -9000, 31, headlight=True)
    assert gui.signed_byte(payload[1]) == 100
    assert gui.signed_byte(payload[2]) == -100
    assert payload[3] == 1
    assert payload[6] == 15
    assert gui.has_valid_checksum(payload)


def test_motion_table_matches_tracked_vehicle_kinematics():
    gui = load_gui_module()
    assert gui.MOTIONS["forward"].left_factor == 1.0
    assert gui.MOTIONS["forward"].right_factor == 1.0
    assert gui.MOTIONS["rotate_left"].left_factor < 0
    assert gui.MOTIONS["rotate_left"].right_factor > 0
    assert gui.MOTIONS["reverse"].left_factor == -1.0
    assert gui.MOTIONS["reverse"].right_factor == -1.0


def test_chassis_and_motor_feedback_decoding():
    gui = load_gui_module()
    chassis = with_checksum(
        gui,
        [0x09, 0x04, 0xEE, 0x02, 0x00, 0x00, 0x03, 0x00],
    )
    decoded_chassis = gui.decode_chassis_feedback(chassis)
    assert decoded_chassis is not None
    assert decoded_chassis.work_mode == 1
    assert decoded_chassis.running
    assert decoded_chassis.headlight
    assert decoded_chassis.battery_voltage == 75.0
    assert not decoded_chassis.has_fault

    motor = bytearray(8)
    motor[1:3] = (1500).to_bytes(2, "little", signed=True)
    motor[3] = 75
    motor[4] = (-8) & 0xFF
    motor[5] = 70
    motor = with_checksum(gui, motor)
    decoded_motor = gui.decode_motor_feedback(motor)
    assert decoded_motor is not None
    assert decoded_motor.speed_rpm == 1500
    assert decoded_motor.speed_mps == 0.55
    assert decoded_motor.current == -8
    assert decoded_motor.temperature_c == 30


def test_captured_feedback_explains_why_unlock_is_rejected():
    gui = load_gui_module()
    payload = bytes.fromhex("040077000100097b")
    feedback = gui.decode_chassis_feedback(payload)
    assert feedback is not None
    assert gui.chassis_fault_reason(feedback) == (
        "底盘未切换到无人模式；底盘急停已按下；遥控器通信故障"
    )


def test_autonomous_mode_without_faults_is_allowed():
    gui = load_gui_module()
    payload = with_checksum(
        gui,
        [0x01, 0x00, 0x77, 0x00, 0x00, 0x00, 0x04, 0x00],
    )
    feedback = gui.decode_chassis_feedback(payload)
    assert feedback is not None
    assert gui.chassis_fault_reason(feedback) == ""


def test_dry_run_transport_uses_differential_command_id():
    gui = load_gui_module()
    link = gui.CanLink(
        "/not/a/device",
        dry_run=True,
        command_id=gui.COMMAND_ID,
        telemetry_ids=gui.TELEMETRY_IDS,
        bitrate=gui.CAN_BITRATE,
    )
    assert link.command_id == 0x514
    assert link.telemetry_ids == {0x532, 0x533, 0x534}
    assert link.bitrate == 250000
    assert link.connect()
    payload = gui.encode_command(300, 300)
    assert link.send(payload)
    assert link.dry_run_payloads == [payload]


def test_module_self_test_covers_standalone_protocol_path():
    gui = load_gui_module()
    assert gui.run_self_test() == 0
