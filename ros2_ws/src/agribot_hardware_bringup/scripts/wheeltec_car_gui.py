#!/usr/bin/env python3
"""Touch and keyboard controller for the WHEELTEC C50C Ackermann base."""

from __future__ import annotations

import argparse
import errno
import fcntl
import os
import select
import signal
import struct
import sys
import termios
import time
import tkinter as tk
import tkinter.font as tkfont
import tty
from dataclasses import dataclass


ZQWL_PORT = (
    "/dev/serial/by-id/"
    "usb-ZQWL-CANFD_ZQWL-CANFD_966960660237-if00"
)
ZQWL_START_PACKET = bytes.fromhex(
    "493b425700000300000000000000000000000000452e"
)
ZQWL_STOP_PACKET = bytes.fromhex(
    "493b445701000100000000000000000000000000452e"
)
COMMAND_ID = 0x181
TELEMETRY_IDS = {0x101, 0x102, 0x103}
TICK_MS = 50
STARTUP_FEEDBACK_TIMEOUT_SEC = 1.0
MIN_SPEED_MPS = 0.05
MAX_SPEED_MPS = 0.50
DEFAULT_SPEED_MPS = 0.30
MIN_STEERING_RAD = 0.05
MAX_STEERING_RAD = 0.30
DEFAULT_STEERING_RAD = 0.30


def clamp(value: float, minimum: float, maximum: float) -> float:
    return max(minimum, min(maximum, value))


def encode_command(vx: float, vy: float, steering: float) -> bytes:
    """Encode C50C 0x181 payload: signed big-endian values scaled by 1000."""
    values = (
        round(clamp(vx, -MAX_SPEED_MPS, MAX_SPEED_MPS) * 1000),
        round(clamp(vy, -MAX_SPEED_MPS, MAX_SPEED_MPS) * 1000),
        round(clamp(steering, -MAX_STEERING_RAD, MAX_STEERING_RAD) * 1000),
        0,
    )
    return struct.pack(">hhhH", *values)


def encode_can_packet(can_id: int, payload: bytes, channel: int = 0) -> bytes:
    if not 0 <= can_id <= 0x7FF:
        raise ValueError("only standard 11-bit CAN identifiers are supported")
    if len(payload) != 8:
        raise ValueError("CAN payload must contain exactly 8 bytes")
    if channel != 0:
        raise ValueError("the ZQWL adapter is configured for channel 0")
    return (
        bytes((0x5A, len(payload), channel))
        + can_id.to_bytes(4, "big")
        + payload
        + bytes((0xA5,))
    )


class ZqwlFrameDecoder:
    """Decode fragmented and coalesced ZQWL classic-CAN packets."""

    STATUS_PACKET_SIZE = 32
    MAX_BUFFER_SIZE = 65536

    def __init__(self) -> None:
        self.buffer = bytearray()
        self.invalid_frames = 0

    def feed(self, data: bytes) -> list[tuple[int, bytes]]:
        self.buffer.extend(data)
        if len(self.buffer) > self.MAX_BUFFER_SIZE:
            del self.buffer[: len(self.buffer) - self.MAX_BUFFER_SIZE]

        frames: list[tuple[int, bytes]] = []
        while True:
            try:
                start = self.buffer.index(0x5A)
            except ValueError:
                self.buffer.clear()
                break
            if start:
                del self.buffer[:start]
            if len(self.buffer) < 2:
                break

            payload_size = self.buffer[1]
            if payload_size == 0xFE:
                if len(self.buffer) < self.STATUS_PACKET_SIZE:
                    break
                if self.buffer[self.STATUS_PACKET_SIZE - 1] != 0xA5:
                    del self.buffer[0]
                    self.invalid_frames += 1
                    continue
                del self.buffer[: self.STATUS_PACKET_SIZE]
                continue

            if payload_size > 8:
                del self.buffer[0]
                self.invalid_frames += 1
                continue
            packet_size = payload_size + 8
            if len(self.buffer) < packet_size:
                break
            if self.buffer[packet_size - 1] != 0xA5:
                del self.buffer[0]
                self.invalid_frames += 1
                continue
            if self.buffer[2] != 0 or payload_size != 8:
                del self.buffer[:packet_size]
                self.invalid_frames += 1
                continue

            can_id = int.from_bytes(self.buffer[3:7], "big")
            if can_id > 0x7FF:
                del self.buffer[:packet_size]
                self.invalid_frames += 1
                continue
            frames.append((can_id, bytes(self.buffer[7:15])))
            del self.buffer[:packet_size]
        return frames


class CanLink:
    """Exclusive ZQWL USB CDC transport used by the standalone GUI."""

    def __init__(self, port: str, dry_run: bool = False) -> None:
        self.port = port
        self.dry_run = dry_run
        self.fd: int | None = None
        self.decoder = ZqwlFrameDecoder()
        self.last_error = ""
        self.last_rx_time = 0.0
        self.rx_count = 0
        self.tx_count = 0
        self.seen_ids: set[int] = set()
        self.connection_seen_ids: set[int] = set()
        self.connected_at = 0.0
        self.startup_reopen_attempted = False
        self.startup_reopen_count = 0
        self.last_payload = bytes(8)
        self.dry_run_payloads: list[bytes] = []

    def connect(self, reset_startup_recovery: bool = True) -> bool:
        if self.dry_run:
            self.last_error = ""
            self.connected_at = time.monotonic()
            self.last_rx_time = self.connected_at
            if reset_startup_recovery:
                self.startup_reopen_attempted = False
            return True
        if self.fd is not None:
            return True

        fd: int | None = None
        try:
            fd = os.open(
                self.port,
                os.O_RDWR | os.O_NOCTTY | os.O_NONBLOCK,
            )
            fcntl.ioctl(fd, termios.TIOCEXCL)
            tty.setraw(fd, termios.TCSANOW)
            attributes = termios.tcgetattr(fd)
            attributes[4] = termios.B115200
            attributes[5] = termios.B115200
            attributes[2] |= termios.CLOCAL | termios.CREAD
            attributes[2] &= ~(termios.PARENB | termios.CSTOPB | termios.CSIZE)
            attributes[2] |= termios.CS8
            termios.tcsetattr(fd, termios.TCSANOW, attributes)
            termios.tcflush(fd, termios.TCIOFLUSH)
            self.fd = fd
            self._write_all(ZQWL_STOP_PACKET)
            time.sleep(0.1)
            termios.tcflush(fd, termios.TCIFLUSH)
            self._write_all(ZQWL_START_PACKET)
            self.decoder = ZqwlFrameDecoder()
            self.connection_seen_ids.clear()
            self.connected_at = time.monotonic()
            self.last_rx_time = 0.0
            if reset_startup_recovery:
                self.startup_reopen_attempted = False
            self.last_error = ""
            return True
        except (OSError, RuntimeError) as exc:
            self.last_error = str(exc)
            if fd is not None:
                try:
                    os.close(fd)
                except OSError:
                    pass
            self.fd = None
            return False

    def _write_all(self, packet: bytes) -> None:
        if self.fd is None:
            raise OSError(errno.ENOTCONN, "ZQWL adapter is not connected")
        offset = 0
        deadline = time.monotonic() + 0.5
        while offset < len(packet):
            try:
                written = os.write(self.fd, packet[offset:])
                if written:
                    offset += written
                    continue
            except BlockingIOError:
                pass
            except InterruptedError:
                continue
            remaining = deadline - time.monotonic()
            if remaining <= 0:
                raise RuntimeError("write(ZQWL CDC) timed out")
            select.select([], [self.fd], [], min(0.02, remaining))

    def close(self, send_stop: bool = True) -> None:
        if self.fd is None:
            return
        fd = self.fd
        try:
            if send_stop:
                zero_packet = encode_can_packet(COMMAND_ID, bytes(8))
                for _ in range(8):
                    self._write_all(zero_packet)
                    time.sleep(0.01)
                self._write_all(ZQWL_STOP_PACKET)
        except (OSError, RuntimeError):
            pass
        self.fd = None
        try:
            os.close(fd)
        except OSError:
            pass

    def telemetry_alive(self, now: float | None = None) -> bool:
        if self.dry_run:
            return True
        current = time.monotonic() if now is None else now
        return self.fd is not None and current - self.last_rx_time < 0.6

    def recover_silent_startup(
        self,
        now: float | None = None,
        timeout: float = STARTUP_FEEDBACK_TIMEOUT_SEC,
    ) -> bool:
        if (
            self.dry_run
            or self.fd is None
            or self.startup_reopen_attempted
            or self.connection_seen_ids == TELEMETRY_IDS
        ):
            return False

        current = time.monotonic() if now is None else now
        if current - self.connected_at < timeout:
            return False

        self.startup_reopen_attempted = True
        self.close()
        time.sleep(0.1)
        if not self.connect(reset_startup_recovery=False):
            return False
        self.startup_reopen_count += 1
        return True

    def poll(self) -> None:
        if self.dry_run:
            self.last_rx_time = time.monotonic()
            return
        if self.fd is None:
            return

        try:
            while True:
                try:
                    chunk = os.read(self.fd, 4096)
                except BlockingIOError:
                    break
                if not chunk:
                    break
                for can_id, _payload in self.decoder.feed(chunk):
                    if can_id in TELEMETRY_IDS:
                        self.last_rx_time = time.monotonic()
                        self.rx_count += 1
                        self.seen_ids.add(can_id)
                        self.connection_seen_ids.add(can_id)
        except OSError as exc:
            self.last_error = str(exc)
            self.close(send_stop=False)

    def send(self, payload: bytes) -> bool:
        self.last_payload = payload
        if self.dry_run:
            self.tx_count += 1
            self.dry_run_payloads.append(payload)
            self.dry_run_payloads = self.dry_run_payloads[-200:]
            return True
        if self.fd is None and not self.connect():
            return False
        try:
            self._write_all(encode_can_packet(COMMAND_ID, payload))
            self.tx_count += 1
            self.last_error = ""
            return True
        except (OSError, RuntimeError) as exc:
            self.last_error = str(exc)
            self.close(send_stop=False)
            return False


@dataclass(frozen=True)
class Motion:
    name: str
    velocity_factor: int
    steering_factor: int


MOTIONS = {
    "forward_left": Motion("左前", 1, 1),
    "forward": Motion("前进", 1, 0),
    "forward_right": Motion("右前", 1, -1),
    "left": Motion("左打方向", 0, 1),
    "right": Motion("右打方向", 0, -1),
    "reverse_left": Motion("左后", -1, 1),
    "reverse": Motion("后退", -1, 0),
    "reverse_right": Motion("右后", -1, -1),
}


class CarControlGui:
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
        self.link = CanLink(args.port, args.dry_run)
        self.armed = False
        self.speed = DEFAULT_SPEED_MPS
        self.steering = DEFAULT_STEERING_RAD
        self.pointer_motion: Motion | None = None
        self.pressed_keys: set[str] = set()
        self.current_motion: Motion | None = None
        self.stop_burst_remaining = 0
        self.last_activity = time.monotonic()
        self.last_connect_attempt = 0.0
        self.motion_buttons: dict[str, tk.Button] = {}
        self._placements: list[tuple[tk.Widget, int, int, int, int]] = []
        self._fonts: dict[tuple[int, str], tkfont.Font] = {}
        self._layout_job: str | None = None
        self._last_layout_size = (self.DESIGN_WIDTH, self.DESIGN_HEIGHT)

        root.title("WHEELTEC 大型阿克曼控制")
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
            "WHEELTEC 大型阿克曼控制",
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
            704,
            16,
            288,
            50,
            bg=self.WARNING,
        )

        button_specs = [
            ("forward_left", "↖\n左前", 24, 110),
            ("forward", "↑\n前进", 206, 110),
            ("forward_right", "↗\n右前", 388, 110),
            ("left", "←\n左打方向", 24, 254),
            ("right", "→\n右打方向", 388, 254),
            ("reverse_left", "↙\n左后", 24, 398),
            ("reverse", "↓\n后退", 206, 398),
            ("reverse_right", "↘\n右后", 388, 398),
        ]
        for key, text, x, y in button_specs:
            button = tk.Button(
                self.root,
                text=text,
                font=self._font(21),
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
            self._place(button, x, y, 170, 132)
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
        self._place(self.stop_button, 206, 254, 170, 132)
        self.stop_button.bind(
            "<ButtonPress-1>",
            lambda _event: self.emergency_stop("手动停止"),
        )

        side_panel = tk.Frame(self.root, bg=self.SURFACE)
        self._place(side_panel, 584, 92, 416, 638)
        self.arm_button = tk.Button(
            self.root,
            text="控制锁定\n点击启用",
            font=self._font(20),
            bg=self.WARNING,
            fg=self.TEXT,
            activebackground="#7A2E0E",
            activeforeground=self.TEXT,
            relief="flat",
            command=self.toggle_arm,
            takefocus=True,
        )
        self._place(self.arm_button, 606, 116, 372, 80)

        self._label(
            "速度",
            16,
            610,
            218,
            90,
            42,
            bg=self.SURFACE,
            anchor="w",
        )
        self.speed_value = self._label(
            f"{self.speed:.2f} m/s",
            20,
            700,
            218,
            150,
            42,
            bg=self.SURFACE,
        )
        self._make_adjust_button(
            "−",
            852,
            214,
            lambda: self.adjust_speed(-0.05),
        )
        self._make_adjust_button(
            "+",
            918,
            214,
            lambda: self.adjust_speed(0.05),
        )

        self._label(
            "转角",
            16,
            610,
            286,
            90,
            42,
            bg=self.SURFACE,
            anchor="w",
        )
        self.steering_value = self._label(
            f"{self.steering:.2f} rad",
            20,
            700,
            286,
            150,
            42,
            bg=self.SURFACE,
        )
        self._make_adjust_button(
            "−",
            852,
            282,
            lambda: self.adjust_steering(-0.05),
        )
        self._make_adjust_button(
            "+",
            918,
            282,
            lambda: self.adjust_steering(0.05),
        )

        self.command_label = self._label(
            "当前指令：停止",
            18,
            606,
            356,
            372,
            58,
            bg=self.BUTTON,
            anchor="w",
        )
        self.counter_label = self._label(
            "RX 0  ·  TX 0",
            15,
            606,
            424,
            372,
            42,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="w",
        )
        self.error_label = self._label(
            "",
            13,
            606,
            470,
            372,
            52,
            bg=self.SURFACE,
            fg="#FFB4AB",
            anchor="nw",
        )
        self._label(
            "键盘",
            16,
            606,
            538,
            80,
            36,
            bg=self.SURFACE,
            anchor="w",
        )
        self._label(
            "WASD / 方向键：移动\n空格：停止并锁定    F11：全屏",
            14,
            606,
            574,
            372,
            70,
            bg=self.SURFACE,
            fg=self.MUTED,
            anchor="nw",
        )
        self._label(
            "按住运动，松开停止；窗口失焦自动锁定",
            15,
            24,
            560,
            534,
            54,
            bg=self.SURFACE,
        )
        self.exit_button = tk.Button(
            self.root,
            text="退出程序",
            font=self._font(15),
            bg=self.BUTTON,
            fg=self.TEXT,
            activebackground=self.DANGER,
            activeforeground=self.TEXT,
            relief="flat",
            command=self.close,
            takefocus=True,
        )
        self._place(self.exit_button, 842, 662, 136, 48)

    def _make_adjust_button(self, text: str, x: int, y: int, command) -> None:
        button = tk.Button(
            self.root,
            text=text,
            font=self._font(22),
            bg=self.BUTTON,
            fg=self.TEXT,
            activebackground=self.BUTTON_ACTIVE,
            activeforeground=self.TEXT,
            relief="flat",
            command=command,
            takefocus=True,
        )
        self._place(button, x, y, 56, 50)

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

    def adjust_speed(self, delta: float) -> None:
        self.speed = round(
            clamp(self.speed + delta, MIN_SPEED_MPS, MAX_SPEED_MPS),
            2,
        )
        self.speed_value.configure(text=f"{self.speed:.2f} m/s")
        self.last_activity = time.monotonic()

    def adjust_steering(self, delta: float) -> None:
        self.steering = round(
            clamp(
                self.steering + delta,
                MIN_STEERING_RAD,
                MAX_STEERING_RAD,
            ),
            2,
        )
        self.steering_value.configure(text=f"{self.steering:.2f} rad")
        self.last_activity = time.monotonic()

    def toggle_arm(self) -> None:
        if self.armed:
            self.emergency_stop("控制已锁定")
            return
        if not self.link.telemetry_alive():
            self.error_label.configure(text="无法启用：未收到 C50C 遥测")
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
        velocity = int("w" in self.pressed_keys) - int(
            "s" in self.pressed_keys
        )
        steering = int("a" in self.pressed_keys) - int(
            "d" in self.pressed_keys
        )
        if velocity == 0 and steering == 0:
            return None
        names = {
            (1, 1): "左前",
            (1, 0): "前进",
            (1, -1): "右前",
            (0, 1): "左打方向",
            (0, -1): "右打方向",
            (-1, 1): "左后",
            (-1, 0): "后退",
            (-1, -1): "右后",
        }
        return Motion(
            names.get((velocity, steering), "停止"),
            velocity,
            steering,
        )

    def _update_motion(self) -> None:
        self.current_motion = self.pointer_motion or self._keyboard_motion()
        name = self.current_motion.name if self.current_motion else "停止"
        self.command_label.configure(text=f"当前指令：{name}")
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

    def _current_payload(self) -> bytes:
        if not self.armed or self.current_motion is None:
            return bytes(8)
        motion = self.current_motion
        return encode_command(
            motion.velocity_factor * self.speed,
            0.0,
            motion.steering_factor * self.steering,
        )

    def _refresh_status(self, now: float) -> None:
        alive = self.link.telemetry_alive(now)
        if self.args.dry_run:
            self.status_badge.configure(text="● 模拟模式", bg=self.ONLINE)
        elif alive:
            self.status_badge.configure(text="● USB CAN 正常", bg=self.ONLINE)
        elif self.link.fd is not None:
            self.status_badge.configure(text="▲ 无底盘遥测", bg=self.WARNING)
        else:
            self.status_badge.configure(text="✕ USB CAN 离线", bg=self.DANGER)
        self.counter_label.configure(
            text=(
                f"RX {self.link.rx_count}  ·  TX {self.link.tx_count}"
                f"  ·  ERR {self.link.decoder.invalid_frames}"
            )
        )
        self.error_label.configure(text=self.link.last_error)

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
        self.link.recover_silent_startup()

        if self.armed and not self.link.telemetry_alive(now):
            self.emergency_stop("CAN 遥测中断")
        if (
            self.armed
            and self.current_motion is None
            and now - self.last_activity > 30.0
        ):
            self.emergency_stop("30 秒无操作")

        if self.armed:
            if not self.link.send(self._current_payload()):
                self.emergency_stop("CAN 发送失败")
        elif self.stop_burst_remaining > 0:
            self.link.send(bytes(8))
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
                "<ButtonRelease-1>",
                x=700,
                y=500,
            ),
        )
        self.root.after(
            400,
            lambda: self.root.event_generate("<KeyPress-a>"),
        )
        self.root.after(
            650,
            lambda: self.root.event_generate("<KeyRelease-a>"),
        )
        self.root.after(750, lambda: self.root.geometry("800x600+0+0"))
        self.root.after(1000, self._finish_ui_self_test)

    def _finish_ui_self_test(self) -> None:
        expected = {
            encode_command(DEFAULT_SPEED_MPS, 0.0, 0.0),
            encode_command(0.0, 0.0, DEFAULT_STEERING_RAD),
            bytes(8),
        }
        seen = set(self.link.dry_run_payloads)
        missing = expected - seen
        forward = self.motion_buttons["forward"]
        release_ok = (
            self.pointer_motion is None
            and self.current_motion is None
        )
        relief_ok = str(forward.cget("relief")) == "raised"
        layout_ok = self._last_layout_size == (
            self.root.winfo_width(),
            self.root.winfo_height(),
        )
        if missing or not release_ok or not relief_ok or not layout_ok:
            details = [payload.hex() for payload in missing]
            details.extend(
                [
                    f"release={release_ok}",
                    f"relief={relief_ok}",
                    f"layout={layout_ok}:{self._last_layout_size}",
                ]
            )
            print("UI_SELF_TEST_FAIL", ",".join(details), flush=True)
            self.args.self_test_failed = True
        else:
            print(
                "UI_SELF_TEST_OK input-release-and-responsive-layout",
                flush=True,
            )
        self.close()

    def close(self) -> None:
        self.armed = False
        for _ in range(8):
            self.link.send(bytes(8))
        self.link.close()
        try:
            self.root.destroy()
        except tk.TclError:
            pass


def run_self_test() -> int:
    cases = {
        (0.0, 0.0, 0.0): "0000000000000000",
        (0.10, 0.0, 0.0): "0064000000000000",
        (-0.10, 0.0, 0.0): "ff9c000000000000",
        (0.0, 0.0, 0.12): "0000000000780000",
        (0.0, 0.0, -0.12): "00000000ff880000",
    }
    for values, expected in cases.items():
        actual = encode_command(*values).hex()
        if actual != expected:
            print(f"SELF_TEST_FAIL {values}: {actual} != {expected}")
            return 1

    packet = encode_can_packet(
        COMMAND_ID,
        encode_command(0.10, 0.0, -0.10),
    )
    expected_packet = "5a08000000018100640000ff9c0000a5"
    if packet.hex() != expected_packet:
        print(f"SELF_TEST_FAIL packet={packet.hex()}")
        return 1

    status = bytes.fromhex(
        "5afe0014003c00000000000000000000000020000000000000000000000000a5"
    )
    feedback = encode_can_packet(0x101, bytes.fromhex("7b00000000000000"))
    decoder = ZqwlFrameDecoder()
    if decoder.feed(status[:9]):
        print("SELF_TEST_FAIL fragmented-status")
        return 1
    decoded = decoder.feed(status[9:] + feedback[:7])
    decoded += decoder.feed(feedback[7:])
    if decoded != [(0x101, bytes.fromhex("7b00000000000000"))]:
        print(f"SELF_TEST_FAIL decoder={decoded}")
        return 1
    if decoder.invalid_frames:
        print(
            f"SELF_TEST_FAIL invalid-frames={decoder.invalid_frames}"
        )
        return 1
    print("SELF_TEST_OK encoding-and-zqwl-cdc")
    return 0


def run_link_test(port: str, duration: float) -> int:
    link = CanLink(port)
    if not link.connect():
        print(f"LINK_TEST_FAIL connect: {link.last_error}")
        return 1
    deadline = time.monotonic() + duration
    next_tx = time.monotonic()
    try:
        while time.monotonic() < deadline:
            link.poll()
            link.recover_silent_startup()
            now = time.monotonic()
            if now >= next_tx:
                if not link.send(bytes(8)):
                    print(f"LINK_TEST_FAIL send: {link.last_error}")
                    return 1
                next_tx += TICK_MS / 1000.0
            time.sleep(0.005)
        link.poll()
        missing = TELEMETRY_IDS - link.connection_seen_ids
        if missing:
            missing_text = ",".join(
                f"0x{can_id:03x}" for can_id in sorted(missing)
            )
            print(
                "LINK_TEST_FAIL "
                f"missing={missing_text} rx={link.rx_count} "
                f"tx={link.tx_count} "
                f"invalid={link.decoder.invalid_frames}"
            )
            return 1
        print(
            "LINK_TEST_OK "
            f"rx={link.rx_count} tx={link.tx_count} "
            f"ids={','.join(f'0x{x:03x}' for x in sorted(link.seen_ids))} "
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
        help="Test frame encoding without a display",
    )
    parser.add_argument(
        "--ui-self-test",
        action="store_true",
        help="Exercise GUI mappings in dry-run mode",
    )
    parser.add_argument(
        "--link-test",
        action="store_true",
        help="Send only zero-speed frames and verify all feedback IDs",
    )
    parser.add_argument(
        "--link-test-duration",
        type=float,
        default=2.0,
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
        return run_link_test(args.port, max(0.2, args.link_test_duration))
    root = tk.Tk()
    app = CarControlGui(root, args)
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
