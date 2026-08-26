#!/usr/bin/env python3
"""Standalone touch controller for the three-in-one tracked chassis."""

from __future__ import annotations

import argparse
import signal
import sys
import time
import tkinter as tk
import tkinter.font as tkfont
from dataclasses import dataclass

from wheeltec_car_gui import CanLink, ZQWL_PORT, clamp


COMMAND_ID = 0x514
CHASSIS_STATE_ID = 0x532
LEFT_MOTOR_STATE_ID = 0x533
RIGHT_MOTOR_STATE_ID = 0x534
TELEMETRY_IDS = {
    CHASSIS_STATE_ID,
    LEFT_MOTOR_STATE_ID,
    RIGHT_MOTOR_STATE_ID,
}

# The chassis command frame carries signed PWM percentage. The controller's
# 3000 rpm calibration point corresponds to 100 percent and 1.1 m/s.
MAX_DRIVE_LEVEL = 3000
MAX_SPEED_MPS = 1.10
DEFAULT_DRIVE_LEVEL = 800
DRIVE_LEVEL_STEP = 100
INNER_TRACK_RATIO = 0.35
TICK_MS = 100
FEEDBACK_TIMEOUT_SEC = 1.2
STARTUP_FEEDBACK_TIMEOUT_SEC = 1.5


def signed_byte(value: int) -> int:
    return value if value < 0x80 else value - 0x100


def xor_checksum(data: bytes | bytearray) -> int:
    checksum = 0
    for value in data:
        checksum ^= value
    return checksum


def has_valid_checksum(payload: bytes) -> bool:
    return len(payload) == 8 and xor_checksum(payload[:7]) == payload[7]


def drive_level_to_percent(level: float) -> int:
    limited = clamp(level, -MAX_DRIVE_LEVEL, MAX_DRIVE_LEVEL)
    return round(limited / MAX_DRIVE_LEVEL * 100.0)


def drive_level_to_speed(level: float) -> float:
    limited = clamp(level, -MAX_DRIVE_LEVEL, MAX_DRIVE_LEVEL)
    return limited / MAX_DRIVE_LEVEL * MAX_SPEED_MPS


def encode_command(
    left_level: float,
    right_level: float,
    rolling_counter: int = 0,
    headlight: bool = False,
) -> bytes:
    """Encode the three-in-one chassis 0x514 Intel-order command."""
    payload = bytearray(8)
    payload[1] = drive_level_to_percent(left_level) & 0xFF
    payload[2] = drive_level_to_percent(right_level) & 0xFF
    payload[3] = 0x01 if headlight else 0x00
    payload[6] = rolling_counter & 0x0F
    payload[7] = xor_checksum(payload[:7])
    return bytes(payload)


@dataclass(frozen=True)
class ChassisFeedback:
    work_mode: int
    emergency_stop: bool
    running: bool
    headlight: bool
    battery_voltage: float
    fault_bits: int

    @property
    def has_fault(self) -> bool:
        return self.fault_bits != 0


@dataclass(frozen=True)
class MotorFeedback:
    speed_rpm: int
    voltage: int
    current: int
    temperature_c: int
    fault_bits: int

    @property
    def speed_mps(self) -> float:
        return drive_level_to_speed(self.speed_rpm)

    @property
    def has_fault(self) -> bool:
        return self.fault_bits != 0


def decode_chassis_feedback(payload: bytes) -> ChassisFeedback | None:
    if not has_valid_checksum(payload):
        return None
    return ChassisFeedback(
        work_mode=payload[0] & 0x03,
        emergency_stop=bool(payload[0] & 0x04),
        running=bool(payload[0] & 0x08),
        headlight=bool(payload[1] & 0x04),
        battery_voltage=int.from_bytes(payload[2:4], "little") * 0.1,
        fault_bits=payload[4] & 0x0F,
    )


def decode_motor_feedback(payload: bytes) -> MotorFeedback | None:
    if not has_valid_checksum(payload):
        return None
    return MotorFeedback(
        speed_rpm=int.from_bytes(payload[1:3], "little", signed=True),
        voltage=payload[3],
        current=signed_byte(payload[4]),
        temperature_c=payload[5] - 40,
        fault_bits=payload[0],
    )


@dataclass(frozen=True)
class Motion:
    name: str
    left_factor: float
    right_factor: float


MOTIONS = {
    "forward_left": Motion("左前弧线", INNER_TRACK_RATIO, 1.0),
    "forward": Motion("前进", 1.0, 1.0),
    "forward_right": Motion("右前弧线", 1.0, INNER_TRACK_RATIO),
    "rotate_left": Motion("原地左转", -1.0, 1.0),
    "rotate_right": Motion("原地右转", 1.0, -1.0),
    "reverse_left": Motion("左后弧线", -INNER_TRACK_RATIO, -1.0),
    "reverse": Motion("后退", -1.0, -1.0),
    "reverse_right": Motion("右后弧线", -1.0, -INNER_TRACK_RATIO),
}


class DifferentialControlGui:
    DESIGN_WIDTH = 1024
    DESIGN_HEIGHT = 768
    BG = "#101820"
    SURFACE = "#1D2A33"
    BUTTON = "#2D3E49"
    BUTTON_ACTIVE = "#176B3A"
    TEXT = "#F5F7FA"
    MUTED = "#C5D0D8"
    OUTLINE = "#7690A0"
    ONLINE = "#067647"
    WARNING = "#B54708"
    DANGER = "#B42318"

    def __init__(self, root: tk.Tk, args: argparse.Namespace) -> None:
        self.root = root
        self.args = args
        self.link = CanLink(
            args.port,
            args.dry_run,
            command_id=COMMAND_ID,
            telemetry_ids=TELEMETRY_IDS,
            telemetry_timeout_sec=FEEDBACK_TIMEOUT_SEC,
        )
        self.armed = False
        self.drive_level = DEFAULT_DRIVE_LEVEL
        self.headlight = tk.BooleanVar(value=False)
        self.pointer_motion: Motion | None = None
        self.pressed_keys: set[str] = set()
        self.current_motion: Motion | None = None
        self.stop_burst_remaining = 0
        self.rolling_counter = 0
        self.last_connect_attempt = 0.0
        self.last_activity = time.monotonic()
        self.motion_buttons: dict[str, tk.Button] = {}
        self._placements: list[tuple[tk.Widget, int, int, int, int]] = []
        self._fonts: dict[tuple[int, str], tkfont.Font] = {}
        self._layout_job: str | None = None
        self._last_layout_size = (self.DESIGN_WIDTH, self.DESIGN_HEIGHT)

        root.title("履带差速车控制")
        root.configure(bg=self.BG)
        root.geometry(f"{self.DESIGN_WIDTH}x{self.DESIGN_HEIGHT}+0+0")
        root.minsize(640, 480)
        root.resizable(True, True)
        if not args.windowed:
            root.attributes("-fullscreen", True)
        root.protocol("WM_DELETE_WINDOW", self.close)

        self.font_family = "Noto Sans CJK SC"
        self._build_ui()
        self._bind_inputs()
        self.root.bind("<Configure>", self._on_configure, add="+")
        self.link.connect()
        self.root.after(TICK_MS, self.tick)

        if args.ui_self_test:
            self.root.after(300, self._schedule_ui_self_test)

    def _font(self, size: int, weight: str = "bold") -> tkfont.Font:
        key = (size, weight)
        if key not in self._fonts:
            self._fonts[key] = tkfont.Font(
                family=self.font_family,
                size=size,
                weight=weight,
            )
        return self._fonts[key]

    def _place(
        self,
        widget: tk.Widget,
        x: int,
        y: int,
        width: int,
        height: int,
    ) -> None:
        self._placements.append((widget, x, y, width, height))
        widget.place(x=x, y=y, width=width, height=height)

    def _label(
        self,
        text: str,
        size: int,
        x: int,
        y: int,
        width: int,
        height: int,
        bg: str | None = None,
        fg: str | None = None,
        anchor: str = "center",
    ) -> tk.Label:
        label = tk.Label(
            self.root,
            text=text,
            font=self._font(size),
            bg=bg or self.BG,
            fg=fg or self.TEXT,
            anchor=anchor,
            justify="left",
        )
        self._place(label, x, y, width, height)
        return label

    def _on_configure(self, event) -> None:
        if event.widget is not self.root:
            return
        if self._layout_job is not None:
            self.root.after_cancel(self._layout_job)
        self._layout_job = self.root.after_idle(self._apply_layout)

    def _apply_layout(self) -> None:
        self._layout_job = None
        width = max(1, self.root.winfo_width())
        height = max(1, self.root.winfo_height())
        scale = min(width / self.DESIGN_WIDTH, height / self.DESIGN_HEIGHT)
        offset_x = round((width - self.DESIGN_WIDTH * scale) / 2)
        offset_y = round((height - self.DESIGN_HEIGHT * scale) / 2)
        for widget, x, y, widget_width, widget_height in self._placements:
            widget.place(
                x=offset_x + round(x * scale),
                y=offset_y + round(y * scale),
                width=max(1, round(widget_width * scale)),
                height=max(1, round(widget_height * scale)),
            )
        for (base_size, _weight), font in self._fonts.items():
            font.configure(size=max(9, round(base_size * scale)))
        self._last_layout_size = (width, height)

    def _build_ui(self) -> None:
        self._label(
            "履带差速车控制",
            26,
            24,
            12,
            520,
            58,
            anchor="w",
        )
        self.status_badge = self._label(
            "● 正在连接",
            17,
            714,
            16,
            286,
            50,
            bg=self.WARNING,
        )

        button_specs = [
            ("forward_left", "↖\n左前", 24, 110),
            ("forward", "↑\n前进", 202, 110),
            ("forward_right", "↗\n右前", 380, 110),
            ("rotate_left", "↶\n原地左转", 24, 254),
            ("rotate_right", "↷\n原地右转", 380, 254),
            ("reverse_left", "↙\n左后", 24, 398),
            ("reverse", "↓\n后退", 202, 398),
            ("reverse_right", "↘\n右后", 380, 398),
        ]
        for key, text, x, y in button_specs:
            button = tk.Button(
                self.root,
                text=text,
                font=self._font(20),
                bg=self.BUTTON,
                fg=self.TEXT,
                activebackground=self.BUTTON_ACTIVE,
                activeforeground=self.TEXT,
                relief="raised",
                bd=3,
                highlightthickness=2,
                highlightbackground=self.OUTLINE,
                takefocus=True,
            )
            self._place(button, x, y, 164, 132)
            button.bind(
                "<ButtonPress-1>",
                lambda event, motion_key=key: self.pointer_press(motion_key),
            )
            button.bind("<ButtonRelease-1>", self.pointer_release)
            button.bind("<Leave>", self.pointer_release)
            self.motion_buttons[key] = button

        self.stop_button = tk.Button(
            self.root,
            text="■\n立即停止",
            font=self._font(22),
            bg=self.DANGER,
            fg=self.TEXT,
            activebackground="#7A271A",
            activeforeground=self.TEXT,
            relief="flat",
            bd=2,
            highlightthickness=3,
            highlightbackground="#F97066",
            command=lambda: self.emergency_stop("手动停止"),
            takefocus=True,
        )
        self._place(self.stop_button, 202, 254, 164, 132)
        self.stop_button.bind(
            "<ButtonPress-1>",
            lambda _event: self.emergency_stop("手动停止"),
        )

        side_panel = tk.Frame(self.root, bg=self.SURFACE)
        self._place(side_panel, 570, 92, 430, 638)
        self.arm_button = tk.Button(
            self.root,
            text="控制锁定\n点击启用",
            font=self._font(19),
            bg=self.WARNING,
            fg=self.TEXT,
            activebackground="#7A2E0E",
            activeforeground=self.TEXT,
            relief="flat",
            command=self.toggle_arm,
            takefocus=True,
        )
        self._place(self.arm_button, 592, 112, 386, 74)

        self._label(
            "速度标定值",
            15,
            594,
            202,
            130,
            34,
            bg=self.SURFACE,
            anchor="w",
        )
        self.drive_value = self._label(
            "",
            17,
            720,
            202,
            256,
            34,
            bg=self.SURFACE,
            anchor="e",
        )
        self.drive_scale = tk.Scale(
            self.root,
            from_=0,
            to=MAX_DRIVE_LEVEL,
            resolution=DRIVE_LEVEL_STEP,
            orient=tk.HORIZONTAL,
            showvalue=False,
            command=self.set_drive_level,
            bg=self.SURFACE,
            fg=self.TEXT,
            troughcolor=self.BUTTON,
            activebackground=self.BUTTON_ACTIVE,
            highlightthickness=0,
            bd=0,
            sliderlength=34,
            takefocus=True,
        )
        self.drive_scale.set(DEFAULT_DRIVE_LEVEL)
        self._place(self.drive_scale, 592, 238, 386, 48)
        self.speed_value = self._label(
            "",
            14,
            594,
            286,
            382,
            30,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="w",
        )
        self._refresh_drive_setting()

        self.headlight_button = tk.Checkbutton(
            self.root,
            text="大灯",
            variable=self.headlight,
            font=self._font(15),
            bg=self.SURFACE,
            fg=self.TEXT,
            activebackground=self.SURFACE,
            activeforeground=self.TEXT,
            selectcolor=self.BUTTON_ACTIVE,
            command=self._headlight_changed,
            takefocus=True,
        )
        self._place(self.headlight_button, 594, 322, 116, 42)

        self.command_label = self._label(
            "当前指令：停止",
            16,
            592,
            372,
            386,
            48,
            bg=self.BUTTON,
            anchor="w",
        )
        self.track_command_label = self._label(
            "左 0 / 右 0",
            14,
            600,
            422,
            370,
            30,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="w",
        )
        self.feedback_label = self._label(
            "等待 0x532 / 0x533 / 0x534 反馈",
            14,
            594,
            458,
            382,
            92,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="nw",
        )
        self.counter_label = self._label(
            "RX 0  ·  TX 0",
            13,
            594,
            554,
            382,
            30,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="w",
        )
        self.error_label = self._label(
            "",
            12,
            594,
            586,
            382,
            44,
            bg=self.SURFACE,
            fg="#FFB4AB",
            anchor="nw",
        )
        self._label(
            "WASD/方向键移动，空格停止并锁定",
            13,
            594,
            630,
            382,
            32,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="w",
        )
        self.exit_button = tk.Button(
            self.root,
            text="退出程序",
            font=self._font(14),
            bg=self.BUTTON,
            fg=self.TEXT,
            activebackground=self.DANGER,
            activeforeground=self.TEXT,
            relief="flat",
            command=self.close,
            takefocus=True,
        )
        self._place(self.exit_button, 842, 674, 136, 42)

        self._label(
            "按住运动，松开立即停止；窗口失焦或反馈中断自动锁定",
            14,
            24,
            560,
            532,
            54,
            bg=self.SURFACE,
        )

    def _bind_inputs(self) -> None:
        self.root.bind_all("<KeyPress>", self.key_press)
        self.root.bind_all("<KeyRelease>", self.key_release)
        self.root.bind_all("<ButtonRelease-1>", self.pointer_release, add="+")
        self.root.bind_all("<FocusOut>", self.focus_out)
        self.root.bind_all("<F11>", self.toggle_fullscreen)
        self.root.bind_all(
            "<Escape>",
            lambda event: self.emergency_stop("Esc 停止"),
        )

    def toggle_fullscreen(self, _event=None):
        current = bool(self.root.attributes("-fullscreen"))
        self.root.attributes("-fullscreen", not current)
        return "break"

    def focus_out(self, _event=None) -> None:
        self.root.after(80, self._check_focus)

    def _check_focus(self) -> None:
        if self.root.focus_displayof() is None and self.armed:
            self.emergency_stop("窗口失焦")

    def set_drive_level(self, value: str | float) -> None:
        self.drive_level = round(
            clamp(float(value), 0.0, float(MAX_DRIVE_LEVEL))
        )
        self._refresh_drive_setting()
        self.last_activity = time.monotonic()

    def _refresh_drive_setting(self) -> None:
        percent = drive_level_to_percent(self.drive_level)
        speed = drive_level_to_speed(self.drive_level)
        self.drive_value.configure(
            text=f"{self.drive_level} / {MAX_DRIVE_LEVEL}"
        )
        self.speed_value.configure(
            text=f"0x514 PWM {percent}%  ·  约 {speed:.2f} m/s"
        )

    def _headlight_changed(self) -> None:
        self.last_activity = time.monotonic()

    def _all_feedback_alive(self, now: float | None = None) -> bool:
        if self.args.dry_run:
            return True
        current = time.monotonic() if now is None else now
        return all(
            current - self.link.latest_payload_times.get(can_id, 0.0)
            < FEEDBACK_TIMEOUT_SEC
            for can_id in TELEMETRY_IDS
        )

    def _chassis_feedback(self) -> ChassisFeedback | None:
        payload = self.link.latest_payloads.get(CHASSIS_STATE_ID)
        return decode_chassis_feedback(payload) if payload else None

    def _fault_reason(self) -> str:
        chassis = self._chassis_feedback()
        if chassis is None:
            return "底盘状态帧缺失或校验错误"
        if chassis.emergency_stop:
            return "底盘急停已按下"
        if chassis.has_fault:
            return f"底盘通信故障 0x{chassis.fault_bits:02X}"
        for can_id, side in (
            (LEFT_MOTOR_STATE_ID, "左电机"),
            (RIGHT_MOTOR_STATE_ID, "右电机"),
        ):
            payload = self.link.latest_payloads.get(can_id)
            motor = decode_motor_feedback(payload) if payload else None
            if motor is None:
                return f"{side}反馈缺失或校验错误"
            if motor.has_fault:
                return f"{side}故障 0x{motor.fault_bits:02X}"
        return ""

    def toggle_arm(self) -> None:
        if self.armed:
            self.emergency_stop("控制已锁定")
            return
        if not self._all_feedback_alive():
            self.error_label.configure(text="无法启用：三条底盘反馈未全部就绪")
            return
        fault = "" if self.args.dry_run else self._fault_reason()
        if fault:
            self.error_label.configure(text=f"无法启用：{fault}")
            return
        self.armed = True
        self.last_activity = time.monotonic()
        self.arm_button.configure(text="控制已启用\n点击锁定", bg=self.ONLINE)
        self.command_label.configure(text="当前指令：停止")

    def emergency_stop(self, reason: str = "立即停止") -> None:
        self.armed = False
        self.pointer_motion = None
        self.pressed_keys.clear()
        self.current_motion = None
        self.stop_burst_remaining = max(self.stop_burst_remaining, 8)
        self.arm_button.configure(text="控制锁定\n点击启用", bg=self.WARNING)
        self.command_label.configure(text=f"当前指令：停止（{reason}）")
        self.track_command_label.configure(text="左 0 / 右 0")
        self._refresh_button_states()

    def pointer_press(self, key: str) -> None:
        if not self.armed:
            self.error_label.configure(text="请先点击右侧“启用控制”")
            return
        self.pointer_motion = MOTIONS[key]
        self.last_activity = time.monotonic()
        self._update_motion()

    def pointer_release(self, _event=None) -> None:
        if self.pointer_motion is not None:
            self.pointer_motion = None
            self.last_activity = time.monotonic()
            self._update_motion()
        for button in self.motion_buttons.values():
            button.configure(relief="raised")

    @staticmethod
    def _normalise_key(keysym: str) -> str:
        key = keysym.lower()
        aliases = {"up": "w", "down": "s", "left": "a", "right": "d"}
        return aliases.get(key, key)

    def key_press(self, event) -> str | None:
        key = self._normalise_key(event.keysym)
        if key == "space":
            self.emergency_stop("空格停止")
            return "break"
        if key not in {"w", "a", "s", "d"}:
            return None
        if not self.armed:
            self.error_label.configure(text="控制已锁定，方向键不会驱动车辆")
            return "break"
        self.pressed_keys.add(key)
        self.last_activity = time.monotonic()
        self._update_motion()
        return "break"

    def key_release(self, event) -> str | None:
        key = self._normalise_key(event.keysym)
        if key not in {"w", "a", "s", "d"}:
            return None
        self.pressed_keys.discard(key)
        self.last_activity = time.monotonic()
        self._update_motion()
        return "break"

    def _keyboard_motion(self) -> Motion | None:
        forward = int("w" in self.pressed_keys) - int(
            "s" in self.pressed_keys
        )
        left = int("a" in self.pressed_keys) - int(
            "d" in self.pressed_keys
        )
        key_map = {
            (1, 1): "forward_left",
            (1, 0): "forward",
            (1, -1): "forward_right",
            (0, 1): "rotate_left",
            (0, -1): "rotate_right",
            (-1, 1): "reverse_left",
            (-1, 0): "reverse",
            (-1, -1): "reverse_right",
        }
        motion_key = key_map.get((forward, left))
        return MOTIONS[motion_key] if motion_key else None

    def _update_motion(self) -> None:
        self.current_motion = self.pointer_motion or self._keyboard_motion()
        name = self.current_motion.name if self.current_motion else "停止"
        self.command_label.configure(text=f"当前指令：{name}")
        left, right = self._current_levels()
        self.track_command_label.configure(text=f"左 {left:+d} / 右 {right:+d}")
        self._refresh_button_states()

    def _refresh_button_states(self) -> None:
        active_name = self.current_motion.name if self.current_motion else ""
        for key, button in self.motion_buttons.items():
            button.configure(
                bg=(
                    self.BUTTON_ACTIVE
                    if MOTIONS[key].name == active_name
                    else self.BUTTON
                )
            )

    def _current_levels(self) -> tuple[int, int]:
        if not self.armed or self.current_motion is None:
            return 0, 0
        return (
            round(self.current_motion.left_factor * self.drive_level),
            round(self.current_motion.right_factor * self.drive_level),
        )

    def _next_payload(self, left: int, right: int) -> bytes:
        payload = encode_command(
            left,
            right,
            rolling_counter=self.rolling_counter,
            headlight=self.headlight.get(),
        )
        self.rolling_counter = (self.rolling_counter + 1) & 0x0F
        return payload

    def _refresh_feedback(self) -> None:
        chassis_payload = self.link.latest_payloads.get(CHASSIS_STATE_ID)
        left_payload = self.link.latest_payloads.get(LEFT_MOTOR_STATE_ID)
        right_payload = self.link.latest_payloads.get(RIGHT_MOTOR_STATE_ID)
        chassis = (
            decode_chassis_feedback(chassis_payload)
            if chassis_payload
            else None
        )
        left = decode_motor_feedback(left_payload) if left_payload else None
        right = decode_motor_feedback(right_payload) if right_payload else None
        if not chassis or not left or not right:
            self.feedback_label.configure(
                text="等待有效的 0x532 / 0x533 / 0x534 反馈"
            )
            return
        mode = "无人" if chassis.work_mode == 1 else "遥控"
        state = "运动" if chassis.running else "静止"
        fault = self._fault_reason() or "无故障"
        self.feedback_label.configure(
            text=(
                f"电池 {chassis.battery_voltage:.1f} V  ·  {mode}/{state}\n"
                f"左 {left.speed_rpm:+d} rpm ({left.speed_mps:+.2f} m/s)  ·  "
                f"右 {right.speed_rpm:+d} rpm ({right.speed_mps:+.2f} m/s)\n"
                f"状态：{fault}"
            )
        )

    def _refresh_status(self, now: float) -> None:
        complete = self._all_feedback_alive(now)
        if self.args.dry_run:
            self.status_badge.configure(text="● 模拟模式", bg=self.ONLINE)
        elif complete:
            self.status_badge.configure(text="● 差速底盘正常", bg=self.ONLINE)
        elif self.link.fd is not None:
            missing = TELEMETRY_IDS - self.link.connection_seen_ids
            suffix = ",".join(f"{can_id:03X}" for can_id in sorted(missing))
            self.status_badge.configure(
                text=f"▲ 反馈不完整 {suffix}",
                bg=self.WARNING,
            )
        else:
            self.status_badge.configure(text="✕ USB CAN 离线", bg=self.DANGER)
        self.counter_label.configure(
            text=(
                f"RX {self.link.rx_count}  ·  TX {self.link.tx_count}"
                f"  ·  ERR {self.link.decoder.invalid_frames}"
            )
        )
        if self.link.last_error:
            self.error_label.configure(text=self.link.last_error)
        elif complete and not self.armed:
            self.error_label.configure(text="")
        self._refresh_feedback()

    def tick(self) -> None:
        now = time.monotonic()
        if (
            self.link.fd is None
            and not self.args.dry_run
            and now - self.last_connect_attempt > 1.0
        ):
            self.last_connect_attempt = now
            self.link.connect()
        self.link.poll()
        self.link.recover_silent_startup(
            timeout=STARTUP_FEEDBACK_TIMEOUT_SEC
        )

        if self.armed and not self._all_feedback_alive(now):
            self.emergency_stop("CAN 反馈中断")
        if self.armed and not self.args.dry_run:
            fault = self._fault_reason()
            if fault:
                self.emergency_stop(fault)
        if (
            self.armed
            and self.current_motion is None
            and now - self.last_activity > 30.0
        ):
            self.emergency_stop("30 秒无操作")

        if self.armed:
            left, right = self._current_levels()
            if not self.link.send(self._next_payload(left, right)):
                self.emergency_stop("CAN 发送失败")
        elif self.stop_burst_remaining > 0:
            self.link.send(self._next_payload(0, 0))
            self.stop_burst_remaining -= 1

        self._refresh_status(now)
        self.root.after(TICK_MS, self.tick)

    def _schedule_ui_self_test(self) -> None:
        self.toggle_arm()
        forward = self.motion_buttons["forward"]
        forward.event_generate("<ButtonPress-1>", x=20, y=20)
        self.root.after(
            250,
            lambda: self.root.event_generate(
                "<ButtonRelease-1>", x=700, y=500
            ),
        )
        self.root.after(400, lambda: self.root.event_generate("<KeyPress-a>"))
        self.root.after(650, lambda: self.root.event_generate("<KeyRelease-a>"))
        self.root.after(750, lambda: self.root.geometry("800x600+0+0"))
        self.root.after(1000, self._finish_ui_self_test)

    def _finish_ui_self_test(self) -> None:
        commands = [
            (signed_byte(payload[1]), signed_byte(payload[2]))
            for payload in self.link.dry_run_payloads
            if has_valid_checksum(payload)
        ]
        forward_seen = any(left > 0 and right > 0 for left, right in commands)
        rotate_seen = any(left < 0 < right for left, right in commands)
        stop_seen = (0, 0) in commands
        forward = self.motion_buttons["forward"]
        release_ok = self.pointer_motion is None and self.current_motion is None
        relief_ok = str(forward.cget("relief")) == "raised"
        layout_ok = self._last_layout_size == (
            self.root.winfo_width(),
            self.root.winfo_height(),
        )
        if not all(
            (forward_seen, rotate_seen, stop_seen, release_ok, relief_ok, layout_ok)
        ):
            print(
                "UI_SELF_TEST_FAIL "
                f"forward={forward_seen},rotate={rotate_seen},stop={stop_seen},"
                f"release={release_ok},relief={relief_ok},layout={layout_ok}",
                flush=True,
            )
            self.args.self_test_failed = True
        else:
            print(
                "UI_SELF_TEST_OK differential-input-release-and-layout",
                flush=True,
            )
        self.close()

    def close(self) -> None:
        self.armed = False
        for _ in range(8):
            self.link.send(self._next_payload(0, 0))
        self.link.close()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def run_self_test() -> int:
    cases = (
        ((0, 0, 0, False), "0000000000000000"),
        ((3000, 3000, 1, False), "0064640000000101"),
        ((-3000, 3000, 2, True), "009c6401000002fb"),
        ((1500, -1500, 15, False), "0032ce0000000ff3"),
    )
    for values, expected in cases:
        actual = encode_command(*values).hex()
        if actual != expected:
            print(f"SELF_TEST_FAIL {values}: {actual} != {expected}")
            return 1
    if drive_level_to_speed(MAX_DRIVE_LEVEL) != MAX_SPEED_MPS:
        print("SELF_TEST_FAIL maximum speed calibration")
        return 1
    if drive_level_to_percent(MAX_DRIVE_LEVEL + 1) != 100:
        print("SELF_TEST_FAIL command saturation")
        return 1

    chassis = bytearray.fromhex("0900ee0200000000")
    chassis[7] = xor_checksum(chassis[:7])
    decoded_chassis = decode_chassis_feedback(bytes(chassis))
    if (
        decoded_chassis is None
        or decoded_chassis.work_mode != 1
        or not decoded_chassis.running
        or decoded_chassis.battery_voltage != 75.0
    ):
        print(f"SELF_TEST_FAIL chassis feedback={decoded_chassis}")
        return 1

    motor = bytearray(8)
    motor[1:3] = (-1234).to_bytes(2, "little", signed=True)
    motor[3] = 75
    motor[4] = (-7) & 0xFF
    motor[5] = 68
    motor[7] = xor_checksum(motor[:7])
    decoded_motor = decode_motor_feedback(bytes(motor))
    if (
        decoded_motor is None
        or decoded_motor.speed_rpm != -1234
        or decoded_motor.current != -7
        or decoded_motor.temperature_c != 28
    ):
        print(f"SELF_TEST_FAIL motor feedback={decoded_motor}")
        return 1
    print("SELF_TEST_OK differential-encoding-feedback-and-calibration")
    return 0


def run_link_test(port: str, duration: float) -> int:
    link = CanLink(
        port,
        command_id=COMMAND_ID,
        telemetry_ids=TELEMETRY_IDS,
        telemetry_timeout_sec=FEEDBACK_TIMEOUT_SEC,
    )
    if not link.connect():
        print(f"LINK_TEST_FAIL connect: {link.last_error}")
        return 1
    deadline = time.monotonic() + duration
    next_tx = time.monotonic()
    rolling_counter = 0
    try:
        while time.monotonic() < deadline:
            link.poll()
            link.recover_silent_startup(timeout=STARTUP_FEEDBACK_TIMEOUT_SEC)
            now = time.monotonic()
            if now >= next_tx:
                if not link.send(encode_command(0, 0, rolling_counter)):
                    print(f"LINK_TEST_FAIL send: {link.last_error}")
                    return 1
                rolling_counter = (rolling_counter + 1) & 0x0F
                next_tx += TICK_MS / 1000.0
            time.sleep(0.005)
        link.poll()
        missing = TELEMETRY_IDS - link.connection_seen_ids
        if missing:
            missing_text = ",".join(
                f"0x{can_id:03X}" for can_id in sorted(missing)
            )
            print(
                "LINK_TEST_FAIL "
                f"missing={missing_text} rx={link.rx_count} tx={link.tx_count} "
                f"invalid={link.decoder.invalid_frames}"
            )
            return 1
        print(
            "LINK_TEST_OK "
            f"rx={link.rx_count} tx={link.tx_count} "
            f"ids={','.join(f'0x{x:03X}' for x in sorted(link.seen_ids))} "
            f"invalid={link.decoder.invalid_frames}"
        )
        return 0
    finally:
        link.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--port",
        default=ZQWL_PORT,
        help="ZQWL USB CDC serial device",
    )
    parser.add_argument(
        "--windowed",
        action="store_true",
        help="Run in a 1024x768 window",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Do not open or send CAN frames",
    )
    parser.add_argument(
        "--self-test",
        action="store_true",
        help="Test protocol handling without a display",
    )
    parser.add_argument(
        "--ui-self-test",
        action="store_true",
        help="Exercise GUI mappings in dry-run mode",
    )
    parser.add_argument(
        "--link-test",
        action="store_true",
        help="Send zero commands and verify all three feedback frames",
    )
    parser.add_argument(
        "--link-test-duration",
        type=float,
        default=3.0,
        help="Duration of the zero-speed hardware link test",
    )
    args = parser.parse_args()
    args.self_test_failed = False
    if args.ui_self_test:
        args.dry_run = True
        args.windowed = True
    return args


def main() -> int:
    args = parse_args()
    if args.self_test:
        return run_self_test()
    if args.link_test:
        return run_link_test(args.port, max(0.5, args.link_test_duration))
    root = tk.Tk()
    app = DifferentialControlGui(root, args)
    signal.signal(
        signal.SIGINT,
        lambda _signum, _frame: root.after(0, app.close),
    )
    signal.signal(
        signal.SIGTERM,
        lambda _signum, _frame: root.after(0, app.close),
    )
    root.mainloop()
    return 1 if args.self_test_failed else 0


if __name__ == "__main__":
    sys.exit(main())
