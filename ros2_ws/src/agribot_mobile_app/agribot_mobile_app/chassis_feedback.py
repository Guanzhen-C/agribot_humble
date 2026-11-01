"""Fresh, vehicle-specific chassis telemetry without opening the CAN device."""

from __future__ import annotations

import math
import threading
import time


class ChassisFeedback:
    def __init__(self, clock=time.monotonic):
        self._clock = clock
        self._lock = threading.RLock()
        self._condition = threading.Condition(self._lock)
        self.reset()

    def reset(self):
        with self._lock:
            self._data = None
            self._received = None
            self._diagnostic = None
            self._diagnostic_received = None
            self._vehicle = None
            self._condition.notify_all()

    def update_status(self, message):
        data = {
            "linear_velocity": float(message.linear_velocity),
            "angular_velocity": float(message.angular_velocity),
            "battery_voltage": float(message.battery_voltage),
            "control_mode": int(message.control_mode),
            "base_state": int(message.base_state),
            "fault_code": int(message.fault_code),
        }
        for side, index in (
            ("left", message.MOTOR_ID_FRONT_LEFT),
            ("right", message.MOTOR_ID_FRONT_RIGHT),
        ):
            if index < len(message.motor_states):
                data[f"{side}_rpm"] = float(message.motor_states[index].rpm)
        if not all(math.isfinite(value) for value in data.values()):
            return
        data["headlight"] = (
            bool(message.front_light_state.mode)
            if getattr(message, "light_control_enabled", False) else None
        )
        with self._lock:
            self._data = data
            self._received = self._clock()
            self._condition.notify_all()

    def update_diagnostic(self, status):
        vehicle = status.name.rsplit("/", 1)[-1]
        if vehicle not in ("ackermann", "differential"):
            return
        with self._lock:
            self._vehicle = vehicle
            self._diagnostic = {
                "message": status.message,
                "values": {value.key: value.value for value in status.values},
            }
            self._diagnostic_received = self._clock()
            self._condition.notify_all()

    def wait_for_headlight(self, vehicle, enabled, since, timeout=2.0):
        deadline = time.monotonic() + timeout
        with self._condition:
            while True:
                state = self.snapshot(vehicle)
                chassis = state["chassis"]
                if (chassis is not None and self._received >= since
                        and chassis.get("headlight") is enabled):
                    return True
                remaining = deadline - time.monotonic()
                if remaining <= 0:
                    return False
                self._condition.wait(remaining)

    def snapshot(self, vehicle):
        with self._lock:
            now = self._clock()
            age = None if self._received is None else now - self._received
            diagnostic_age = (
                None if self._diagnostic_received is None
                else now - self._diagnostic_received
            )
            diagnostic = self._diagnostic or {"message": "等待底盘CAN反馈", "values": {}}
            values = diagnostic["values"]
            fresh = (
                self._data is not None
                and self._vehicle == vehicle
                and age is not None and 0 <= age <= 1.2
                and diagnostic_age is not None and 0 <= diagnostic_age <= 2.5
                and values.get("feedback_fresh") == "true"
            )
            message = diagnostic["message"]
            if self._vehicle is not None and self._vehicle != vehicle:
                message = "底盘反馈车型与当前车型不一致"
            elif self._data is not None and age is not None and age > 1.2:
                message = "底盘反馈已超时"
            elif diagnostic_age is not None and diagnostic_age > 2.5:
                message = "底盘CAN诊断已停止"
            error = values.get("transport_error", "")
            if error:
                message = f"USB-CAN不可用，请退出桌面CAN GUI后重试：{error}"
            feedback = {
                "fresh": fresh,
                "source": "ros_can",
                "vehicle_type": self._vehicle,
                "age_sec": age,
                "read_only": values.get("read_only") == "true",
                "headlight_supported": values.get("headlight_supported") == "true",
                "message": message,
                "error": error,
            }
            data = dict(self._data) if fresh else None
            if data is not None and vehicle != "differential":
                data.pop("left_rpm", None)
                data.pop("right_rpm", None)
            return {"chassis": data, "chassis_feedback": feedback}
