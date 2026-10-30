"""Timestamped pose prediction and a continuity-preserving one-way handover."""

from dataclasses import dataclass
import math

import numpy as np
from scipy.spatial.transform import Rotation


@dataclass
class Sample:
    stamp: float
    received: float
    pose: np.ndarray
    velocity: np.ndarray
    angular: np.ndarray
    covariance: np.ndarray

    def at(self, target, maximum=0.35):
        dt = target - self.stamp
        if not 0.0 <= dt <= maximum:
            raise ValueError("No contemporaneous pose within the prediction horizon")
        pose = self.pose.copy()
        pose[:3, 3] += self.pose[:3, :3] @ self.velocity * dt
        pose[:3, :3] = self.pose[:3, :3] @ Rotation.from_rotvec(self.angular * dt).as_matrix()
        return pose


def finite_pose(pose):
    return (pose.shape == (4, 4) and np.isfinite(pose).all()
            and np.allclose(pose[:3, :3].T @ pose[:3, :3], np.eye(3), atol=1e-5)
            and np.linalg.det(pose[:3, :3]) > 0.999)


class Handover:
    """Stay on NavSat until independent fused poses pass a stable overlap test."""

    def __init__(self, stable_sec=2.0, max_distance=0.5, max_angle_deg=10.0):
        self.mode = "navsat"
        self.requested = False
        self.stable_since = None
        self.stable_sec = stable_sec
        self.max_distance = max_distance
        self.max_angle = math.radians(max_angle_deg)
        self.offset = np.eye(4)
        self.reason = "NavSat active"

    def request(self):
        first = not self.requested
        self.requested = True
        return first

    def consider(self, navsat_pose, fused_pose, healthy, now, last_output=None,
                 can_take_over=True, output_fused_pose=None):
        if self.mode == "fastlivo_rtk" or not self.requested:
            return False
        if not healthy or navsat_pose is None or fused_pose is None:
            self.stable_since = None
            self.reason = "Waiting for healthy overlapping localization"
            return False
        if not finite_pose(navsat_pose) or not finite_pose(fused_pose):
            self.stable_since = None
            self.reason = "Invalid overlap pose"
            return False
        error = np.linalg.inv(navsat_pose) @ fused_pose
        distance = np.linalg.norm(error[:3, 3])
        angle = Rotation.from_matrix(error[:3, :3]).magnitude()
        if distance > self.max_distance or angle > self.max_angle:
            self.stable_since = None
            self.reason = "Overlap position/orientation disagreement"
            return False
        if self.stable_since is None:
            self.stable_since = now
        if now - self.stable_since < self.stable_sec:
            self.reason = "Validating stable overlap"
            return False
        if not can_take_over:
            self.reason = "Waiting for a fresh fused frame to take over"
            return False
        # This transform is output alignment only, not an RTK factor fed into
        # FAST-LIVO2. It starts at exactly the last public pose.
        anchor = navsat_pose if last_output is None else last_output
        self.offset = anchor @ np.linalg.inv(fused_pose if output_fused_pose is None else output_fused_pose)
        self.mode = "fastlivo_rtk"
        self.reason = "FAST-LIVO2/RTK active"
        return True

    def corrected(self, fused_pose):
        return self.offset @ fused_pose

    def relax_offset(self, dt, reference=None, linear_rate=0.05, angular_rate=math.radians(2.0)):
        """Bound correction slew instead of jumping to the new global estimate."""
        dt = max(0.0, min(dt, .1))
        reference = np.eye(4) if reference is None else reference
        current = self.offset @ reference
        delta = reference[:3, 3] - current[:3, 3]
        length = np.linalg.norm(delta)
        if length:
            current[:3, 3] += delta * min(1.0, linear_rate*dt/length)
        vector = Rotation.from_matrix(current[:3, :3].T @ reference[:3, :3]).as_rotvec()
        angle = np.linalg.norm(vector)
        if angle:
            current[:3, :3] = current[:3, :3] @ Rotation.from_rotvec(
                vector * min(1.0, angular_rate*dt/angle)).as_matrix()
        self.offset = current @ np.linalg.inv(reference)
