from types import SimpleNamespace

from agribot_mobile_app.chassis_feedback import ChassisFeedback


def message(voltage=75.4):
    return SimpleNamespace(
        linear_velocity=0.0, angular_velocity=0.0, battery_voltage=voltage,
        control_mode=1, base_state=0, fault_code=0,
        MOTOR_ID_FRONT_LEFT=0, MOTOR_ID_FRONT_RIGHT=1,
        motor_states=[SimpleNamespace(rpm=120), SimpleNamespace(rpm=121)],
    )


def diagnostic(vehicle="differential", fresh="true", error=""):
    return SimpleNamespace(
        name=f"agribot/chassis_can/{vehicle}", message="CAN feedback",
        values=[SimpleNamespace(key=k, value=v) for k, v in {
            "feedback_fresh": fresh, "read_only": "true", "transport_error": error,
        }.items()],
    )


def test_requires_real_status_and_complete_can_feedback():
    clock = [10.0]
    receiver = ChassisFeedback(lambda: clock[0])
    receiver.update_status(message())
    assert receiver.snapshot("differential")["chassis"] is None
    receiver.update_diagnostic(diagnostic(fresh="false"))
    assert receiver.snapshot("differential")["chassis"] is None
    receiver.update_diagnostic(diagnostic())
    state = receiver.snapshot("differential")
    assert state["chassis"]["battery_voltage"] == 75.4
    assert state["chassis"]["left_rpm"] == 120
    assert state["chassis"]["right_rpm"] == 121
    assert state["chassis_feedback"]["fresh"]
    assert state["chassis_feedback"]["read_only"]


def test_feedback_expires_even_when_publisher_remains_alive():
    clock = [10.0]
    receiver = ChassisFeedback(lambda: clock[0])
    receiver.update_status(message())
    receiver.update_diagnostic(diagnostic())
    clock[0] += 1.3
    state = receiver.snapshot("differential")
    assert state["chassis"] is None
    assert not state["chassis_feedback"]["fresh"]
    assert "超时" in state["chassis_feedback"]["message"]


def test_diagnostic_expiry_and_vehicle_mismatch_fail_closed():
    clock = [10.0]
    receiver = ChassisFeedback(lambda: clock[0])
    receiver.update_status(message())
    receiver.update_diagnostic(diagnostic())
    assert receiver.snapshot("ackermann")["chassis"] is None
    clock[0] += 3
    receiver.update_status(message())
    assert not receiver.snapshot("differential")["chassis_feedback"]["fresh"]


def test_adapter_owner_error_is_visible_and_does_not_fake_data():
    receiver = ChassisFeedback()
    receiver.update_diagnostic(diagnostic(fresh="false", error="Device or resource busy"))
    state = receiver.snapshot("differential")
    assert state["chassis"] is None
    assert "GUI" in state["chassis_feedback"]["message"]


def test_invalid_numbers_and_reset_do_not_retain_feedback():
    receiver = ChassisFeedback()
    receiver.update_status(message(float("nan")))
    receiver.update_diagnostic(diagnostic())
    assert receiver.snapshot("differential")["chassis"] is None
    receiver.update_status(message())
    assert receiver.snapshot("differential")["chassis"] is not None
    receiver.reset()
    assert receiver.snapshot("differential")["chassis"] is None


def test_ackermann_keeps_voltage_without_inventing_motor_rpm():
    receiver = ChassisFeedback()
    receiver.update_status(message(25.4))
    receiver.update_diagnostic(diagnostic(vehicle="ackermann"))
    state = receiver.snapshot("ackermann")
    assert state["chassis"]["battery_voltage"] == 25.4
    assert "left_rpm" not in state["chassis"]
