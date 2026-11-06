import math
import struct
import pytest
from agribot_hil_apple.gazebo_can_bridge import actuator_command


def test_native_steering_uses_ackermann_kinematics_and_authorization():
    payload = struct.pack('>hhhh', 300, 0, 200, 0)
    speed, yaw = actuator_command(payload, .5265874862670898, True)
    assert speed == .3
    assert yaw == pytest.approx(.3 * math.tan(.2) / .5265874862670898)
    assert actuator_command(payload, .5265874862670898, False) == (0., 0.)
    with pytest.raises(ValueError):
        actuator_command(struct.pack('>hhhh', 1000, 0, 0, 0), .526, True)
