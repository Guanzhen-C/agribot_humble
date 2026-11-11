"""Read-only live parameter curves from the X5's verified ISOBUS receive stream."""
import argparse
from collections import deque
import csv
import json
import math
import signal
from pathlib import Path
import struct
import time

import matplotlib.pyplot as plt
from matplotlib import animation, font_manager
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import String
from wire import validate
from powertrain_wire import feedback_fresh, parse_profile


class Monitor(Node):
    def __init__(self):
        super().__init__('powertrain_read_only_monitor')
        self.data = {}; self.received = -math.inf; self.request_speed = 0.; self.request_ratio = math.nan
        self.speed_at = self.command_at = -math.inf
        self.request_rpm = self.shift_ms = self.ratio_rate = math.nan
        self.create_subscription(String, '/hil_hitch/powertrain/status', self.receive,
            QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self.create_subscription(Twist, '/nav2/cmd_vel', self.velocity, 1)
        self.create_subscription(String, '/hil_hitch/powertrain/command', self.command, 1)

    def command(self, message):
        try:
            mode, _, rpm, ratio, shift, rate = parse_profile(message.data)
        except (TypeError, ValueError):
            self.command_at = -math.inf
            return
        self.request_ratio = ratio if mode == 4 else math.nan
        self.request_rpm, self.shift_ms, self.ratio_rate = rpm, shift, rate
        self.command_at = time.monotonic()

    def velocity(self, message):
        self.request_speed = message.linear.x; self.speed_at = time.monotonic()

    def receive(self, message):
        self.data = json.loads(message.data); self.received = time.monotonic()

    def values(self):
        keys = ('powertrain_engine', 'powertrain_transmission', 'powertrain_vehicle', 'powertrain_load', 'powertrain_status')
        if time.monotonic()-self.received > .8 or not feedback_fresh(
                self.data, keys, self.data.get('snapshot_at', math.inf), .35): return None
        raw = [bytes.fromhex(self.data[key]['data']) for key in keys]
        for data in raw: validate(data)
        engine, input_rpm, torque, _ = struct.unpack('<HHhB', raw[0][:7])
        ratio, pa, pb, _ = struct.unpack('<HHHB', raw[1][:7])
        speed, output, gears, fault, _ = struct.unpack('<hhBBB', raw[2][:7])
        load, percent, brake, enabled, version, _ = struct.unpack('<HBBBBB', raw[3][:7])
        status = raw[4]
        if status[0] != 0x22 or status[1] > 4 or status[4] > 2: return None
        return dict(engine_rpm=engine, input_rpm=input_rpm, input_torque=torque*.1,
            ratio=ratio*.1, pressure_a=pa*.001, pressure_b=pb*.001, speed=speed*.001,
            output_rpm=output*.1, actual_gear=gears&15, mode=status[1], selected_gear=status[2],
            shift_state=status[4], fault=fault | status[5], load_force=load, brake=brake,
            requested_speed=self.request_speed if time.monotonic()-self.speed_at < .5 else math.nan,
            requested_ratio=self.request_ratio if time.monotonic()-self.command_at < .5 else math.nan,
            requested_engine_rpm=self.request_rpm if time.monotonic()-self.command_at < .5 else math.nan,
            requested_shift_ms=self.shift_ms if time.monotonic()-self.command_at < .5 else math.nan,
            requested_ratio_rate=self.ratio_rate if time.monotonic()-self.command_at < .5 else math.nan)


def main(args):
    font = Path('/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc')
    if font.is_file():
        font_manager.fontManager.addfont(str(font))
        plt.rcParams['font.family'] = font_manager.FontProperties(fname=str(font)).get_name()
    plt.rcParams['axes.unicode_minus'] = False
    rclpy.init(); node = Monitor()
    figure, axes = plt.subplots(2, 2, figsize=(12, 7), dpi=110)
    signal.signal(signal.SIGTERM, lambda *_: plt.close(figure))
    figure.canvas.manager.set_window_title('动力传动 HIL 参数监视 · 只读')
    figure.subplots_adjust(top=.76, hspace=.38, wspace=.28, left=.08, right=.97)
    heading = figure.text(.06, .94, '动力传动 HIL · 等效模型', fontsize=16, weight='bold')
    summary = figure.text(.06, .81, '等待 X5 经真实 ISOBUS 返回的参数', fontsize=11)
    configs = [(axes[0, 0], ('requested_speed', 'speed'), ('请求车速', '实际车速'), '车速 (m/s)'),
               (axes[0, 1], ('engine_rpm', 'input_rpm'), ('发动机', '输入轴'), '转速 (rpm)'),
               (axes[1, 0], ('requested_ratio', 'ratio'), ('手动请求速比', '实际速比'), '总减速比'),
               (axes[1, 1], ('pressure_a', 'pressure_b'), ('离合器 A', '离合器 B'), '压力 (MPa)')]
    lines = {}
    for ax, keys, labels, title in configs:
        ax.set_title(title, fontsize=12); ax.set_xlabel('时间 (s)'); ax.grid(alpha=.2)
        for index, (key, label) in enumerate(zip(keys, labels)):
            lines[key], = ax.plot([], [], label=label, color=('#187f78', '#c77928')[index],
                                  linestyle='--' if key.startswith('requested') else '-', linewidth=1.5)
        ax.legend(loc='upper left', fontsize=9)
    rows = deque(maxlen=600); start = time.monotonic(); stream = writer = None
    if args.record:
        args.record.parent.mkdir(parents=True, exist_ok=True); stream = args.record.open('x', newline='')

    def update(_):
        nonlocal writer
        for _ in range(32): rclpy.spin_once(node, timeout_sec=0)
        value = node.values()
        if value is None:
            heading.set_color('#9b4038'); summary.set_text('反馈未就绪或已过期；此窗口不发送控制指令。')
            return
        value['time'] = time.monotonic()-start; rows.append(value)
        if stream:
            if writer is None:
                writer = csv.DictWriter(stream, fieldnames=list(value)); writer.writeheader()
            writer.writerow(value); stream.flush()
        modes = ('停止', '动力换挡 / 手动', '动力换挡 / 自动', 'CVT / 自动', 'CVT / 手动速比')
        heading.set_color('#186b59' if not value['fault'] else '#9b4038')
        summary.set_text(f"{modes[value['mode']]}    实际挡位 {value['actual_gear']}    TCU挡位 {value['selected_gear']}    "
            f"换挡阶段 {('稳定', '充油', '交接')[value['shift_state']]}    故障 {value['fault']}\n"
            f"发动机 {value['engine_rpm']:.0f} rpm    速比 {value['ratio']:.1f}    "
            f"离合器滑差 {value['engine_rpm']-value['input_rpm']:.0f} rpm    "
            f"牵引负载 {value['load_force']:.0f} N    制动 {'是' if value['brake'] else '否'}\n"+
            (f"设置请求：转速 {value['requested_engine_rpm']:.0f} rpm    "
             f"换挡交接 {value['requested_shift_ms']:.0f} ms    CVT 速比变化 {value['requested_ratio_rate']:.0f}/s"
             if math.isfinite(value['requested_engine_rpm']) else '设置请求：无新指令'))
        times = [r['time'] for r in rows]
        for ax, keys, _, _ in configs:
            for key in keys: lines[key].set_data(times, [r[key] for r in rows])
            ax.relim(); ax.autoscale_view(); ax.set_xlim(max(0, times[-1]-45), max(10, times[-1]))

    animator = animation.FuncAnimation(figure, update, interval=100, cache_frame_data=False)
    try: plt.show()
    finally:
        if stream: stream.close()
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--record', type=Path)
    main(parser.parse_args())
