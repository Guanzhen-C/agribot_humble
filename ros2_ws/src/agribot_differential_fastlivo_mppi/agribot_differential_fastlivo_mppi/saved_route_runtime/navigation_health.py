"""Validity and diagnostic freshness of saved-path navigation inputs.

The caller holds the gateway state lock. Global map publication is deliberately
not a heartbeat: its size and display rate must not determine motion readiness.
"""

import math
import time


class LocalizationUnavailable(RuntimeError):
    """Recoverable loss of fusion readiness; keep the saved navigation task."""


class NavigationInputHealth:
    LIMITS = {"pose": 1.0, "local_costmap": 3.0}
    LABELS = {"pose": "融合定位", "local_costmap": "实时局部代价地图"}

    def __init__(self):
        self.inputs = {}

    def record(self, name, frame, source_age, now=None):
        now = time.monotonic() if now is None else now
        previous = self.inputs.get(name, {})
        gap = now - previous["received_at"] if previous else 0.0
        self.inputs[name] = {
            "frame": frame.lstrip("/"),
            "received_at": now,
            "source_age_at_receive": source_age,
            "samples": previous.get("samples", 0) + 1,
            "max_gap_sec": max(previous.get("max_gap_sec", 0.0), gap),
            "timeout_gaps": previous.get("timeout_gaps", 0) + int(gap > self.LIMITS[name]),
        }

    def snapshot(self, now=None):
        now = time.monotonic() if now is None else now
        result = {}
        for name, limit in self.LIMITS.items():
            sample = self.inputs.get(name)
            age = now - sample["received_at"] if sample else None
            source_age = sample["source_age_at_receive"] + age if sample else None
            reason = None
            if not sample:
                reason = "尚未收到"
            elif sample["frame"] != "map":
                reason = "坐标系不是map"
            elif not math.isfinite(source_age) or source_age < -0.5:
                reason = "测量时间戳异常"
            valid = reason is None
            if valid and age > limit:
                reason = f"接收中断{age:.2f}秒，门限{limit:g}秒"
            elif valid and source_age > limit:
                reason = f"测量数据已过期{source_age:.2f}秒，门限{limit:g}秒"
            result[name] = {
                "valid": valid,
                "ready": reason is None,
                "reason": reason,
                "receive_age_sec": age,
                "source_age_sec": source_age if source_age is None or math.isfinite(source_age) else None,
                "timeout_sec": limit,
                "samples": sample["samples"] if sample else 0,
                "max_gap_sec": sample["max_gap_sec"] if sample else 0.0,
                "timeout_gaps": sample["timeout_gaps"] if sample else 0,
            }
        return result

    def require(self, fusion_ready, now=None):
        # Age limits are diagnostics only. Do not abort a saved-path task on
        # these gateway-side receipt or measurement-age thresholds.
        reasons = [f"{self.LABELS[name]}：{item['reason']}"
                   for name, item in self.snapshot(now).items() if not item["valid"]]
        if reasons:
            raise RuntimeError("；".join(reasons) + "，停止保存路径任务")
        if not fusion_ready:
            raise LocalizationUnavailable("融合定位未就绪")
