from pathlib import Path


UNIT = (
    Path(__file__).resolve().parents[1]
    / "systemd"
    / "agribot-mobile-app.service"
)


def test_service_starts_only_the_mobile_gateway_launch():
    text = UNIT.read_text(encoding="utf-8")

    assert "mobile_app.launch.py" in text
    assert "ackermann_" not in text
    assert "enable_chassis_output" not in text


def test_service_is_boot_enabled_and_restartable():
    text = UNIT.read_text(encoding="utf-8")

    assert "After=network-online.target" in text
    assert "Environment=ROS_LOCALHOST_ONLY=1" in text
    assert "Restart=always" in text
    assert "WantedBy=multi-user.target" in text
    assert "KillSignal=SIGINT" in text
