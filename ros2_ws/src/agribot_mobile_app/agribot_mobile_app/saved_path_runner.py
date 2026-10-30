"""Follow saved paths; stop and request a short Smac detour only when necessary."""

import math
import threading
import time

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose, FollowPath
from nav_msgs.msg import Path
from rclpy.action import ActionClient

from .saved_path import arc_lengths, blocked_ahead, forward_index, rejoin_indices
from .navigation_health import LocalizationUnavailable


class SavedPathRunner:
    OBSTACLE_LOOKAHEAD_M = 1.0
    MAX_DETOURS = 6
    RECOVERY_STABLE_SEC = 1.0
    RECOVERY_POLL_SEC = 0.1

    def __init__(self, node, observe, update, publish, footprint, goal_tolerance=0.25,
                 callback_group=None, recovery_ready=None):
        self.node, self.observe, self.update, self.publish = node, observe, update, publish
        self.footprint = footprint
        self.goal_tolerance = goal_tolerance
        self.recovery_ready = recovery_ready or (lambda: True)
        self.on_progress = lambda index: None
        self.follow = ActionClient(node, FollowPath, "follow_path", callback_group=callback_group)
        self.plan = ActionClient(node, ComputePathToPose, "compute_path_to_pose", callback_group=callback_group)
        self.stop_event = threading.Event()
        self.thread = None

    @property
    def active(self):
        return self.thread is not None and self.thread.is_alive()

    def start(self, path, on_progress=None):
        if self.active:
            raise RuntimeError("已有保存路径任务")
        if not self.follow.wait_for_server(timeout_sec=0.2) or not self.plan.wait_for_server(timeout_sec=0.2):
            raise RuntimeError("Nav2跟踪或规划服务尚未就绪")
        self.stop_event.clear()
        self.on_progress = on_progress or (lambda index: None)
        self.thread = threading.Thread(target=self._run, args=(path,), daemon=True, name="saved-path")
        self.thread.start()

    def stop(self):
        self.stop_event.set()
        if self.thread:
            self.thread.join(timeout=12)
            if self.thread.is_alive():
                raise RuntimeError("保存路径取消尚未完成，禁止启动其他任务")

    def _wait(self, future, timeout=15):
        deadline = time.monotonic() + timeout
        while not future.done():
            if time.monotonic() > deadline:
                raise RuntimeError("Nav2服务应答超时")
            time.sleep(0.02)
        return future.result()

    def _cancel(self, handle):
        if handle and handle.accepted:
            result = handle.get_result_async()
            self._wait(handle.cancel_goal_async(), 5)
            self._wait(result, 5)

    def _request(self, client, goal):
        if self.stop_event.is_set():
            raise RuntimeError("任务已取消")
        future = client.send_goal_async(goal)
        try:
            handle = self._wait(future, 5)
        except Exception:
            # A late accepted request must not leave a controller running.
            future.add_done_callback(lambda f: f.result().cancel_goal_async()
                                     if f.result() and f.result().accepted else None)
            raise
        if not handle.accepted:
            raise RuntimeError("Nav2拒绝路径请求")
        if self.stop_event.is_set():
            self._cancel(handle)
            raise RuntimeError("任务已取消")
        return handle

    def message(self, poses):
        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = self.node.get_clock().now().to_msg()
        for pose in poses:
            item = PoseStamped()
            item.header = path.header
            item.pose.position.x, item.pose.position.y = pose["x"], pose["y"]
            item.pose.orientation.z, item.pose.orientation.w = math.sin(pose["yaw"] / 2), math.cos(pose["yaw"] / 2)
            path.poses.append(item)
        return path

    def _plan(self, pose, target):
        goal = ComputePathToPose.Goal()
        goal.start, goal.goal = self.message([pose, target]).poses
        goal.use_start, goal.planner_id = True, "GridBased"
        handle = self._request(self.plan, goal)
        future = handle.get_result_async()
        deadline = time.monotonic() + 12
        while not future.done():
            if self.stop_event.wait(0.05) or time.monotonic() > deadline:
                self._cancel(handle)
                raise RuntimeError("局部规划取消或超时")
            try:
                self.observe()
            except Exception:
                self._cancel(handle)
                raise
        response = future.result()
        self.observe()
        if response.status != GoalStatus.STATUS_SUCCEEDED or not response.result.path.poses:
            if not self.recovery_ready():
                raise LocalizationUnavailable("规划中断，等待定位数据恢复")
            return None
        return [{"x": p.pose.position.x, "y": p.pose.position.y,
                 "yaw": math.atan2(2*p.pose.orientation.w*p.pose.orientation.z,
                                   1-2*p.pose.orientation.z**2)} for p in response.result.path.poses]

    def _wait_for_localization(self):
        self.update("executing", "定位暂未就绪，已停车并保留路线，等待稳定恢复")
        stable_since = None
        while not self.stop_event.wait(self.RECOVERY_POLL_SEC):
            try:
                pose, grid = self.observe()
                ready = self.recovery_ready()
            except LocalizationUnavailable:
                ready = False
            if not ready:
                stable_since = None
                continue
            now = time.monotonic()
            if stable_since is None:
                stable_since = now
            if now - stable_since >= self.RECOVERY_STABLE_SEC:
                return pose, grid
        return None

    def _run(self, original):
        handle = None
        outcome = None
        path = original
        mapping = list(range(len(original)))
        index, attempts = 0, 0
        connect = True
        try:
            while not self.stop_event.is_set():
                try:
                    pose, grid = self.observe()
                    self.on_progress(mapping[index])
                    if len(path) == 1 and math.hypot(
                            pose["x"]-original[-1]["x"], pose["y"]-original[-1]["y"]) <= self.goal_tolerance:
                        outcome = ("succeeded", "已进入终点位置容差并停车")
                        return
                    if connect:
                        angle = math.atan2(math.sin(pose["yaw"]-path[0]["yaw"]), math.cos(pose["yaw"]-path[0]["yaw"]))
                        if (len(path) < 2 or math.hypot(pose["x"]-path[0]["x"], pose["y"]-path[0]["y"]) > 0.35
                                or abs(angle) > 0.4):
                            self.update("executing", "正在规划接入路径")
                            entry = self._plan(pose, path[0])
                            if not entry:
                                raise RuntimeError("无法安全接入所选起点")
                            path = entry + path[1:]
                            mapping = [mapping[0]] * len(entry) + mapping[1:]
                        connect = False
                    path_arc = arc_lengths(path)
                    self.publish(path)
                    goal = FollowPath.Goal()
                    goal.path = self.message(path)
                    goal.controller_id, goal.goal_checker_id = "FollowPath", "goal_checker"
                    self.observe()
                    handle = self._request(self.follow, goal)
                    result = handle.get_result_async()
                    index, blocked_since = 0, None
                    last_progress, last_index = time.monotonic(), 0
                    terminal_best = float("inf")
                    self.update("executing", "跟踪保存路径")
                    while not result.done() and not self.stop_event.wait(0.2):
                        pose, grid = self.observe()
                        index = forward_index(path, pose, index, arc=path_arc)
                        self.on_progress(mapping[index])
                        if (path_arc[-1]-path_arc[index] <= 1.0 and math.hypot(
                                pose["x"]-original[-1]["x"], pose["y"]-original[-1]["y"])
                                <= self.goal_tolerance):
                            self._cancel(handle)
                            handle = None
                            outcome = ("succeeded", "已进入终点位置容差并停车")
                            return
                        if path_arc[index]-path_arc[last_index] >= 0.05:
                            last_progress, last_index = time.monotonic(), index
                        # Final XY correction is progress even at the last index.
                        if path_arc[-1]-path_arc[index] <= 1.0:
                            remaining_xy = math.hypot(pose["x"]-original[-1]["x"],
                                                      pose["y"]-original[-1]["y"])
                            if terminal_best-remaining_xy >= 0.05:
                                terminal_best, last_progress = remaining_xy, time.monotonic()
                        blocked = blocked_ahead(path, index, grid, self.footprint, arc=path_arc,
                                                distance=self.OBSTACLE_LOOKAHEAD_M,
                                                outside_is_blocked=False)
                        blocked_since = (blocked_since or time.monotonic()) if blocked else None
                        if ((blocked_since is not None and time.monotonic()-blocked_since > 2)
                                or time.monotonic()-last_progress > 12):
                            break
                    if self.stop_event.is_set():
                        break
                    pose, grid = self.observe()
                    if result.done() and result.result().status == GoalStatus.STATUS_SUCCEEDED:
                        outcome = ("succeeded", "已到达路线终点")
                        return
                    # Finish cancellation before issuing a replacement request.
                    self._cancel(handle)
                    handle = None
                    # TF/pose loss can make Nav2 abort before the fusion flag's
                    # four-second deadline. Do not spend obstacle retries on it.
                    if result.done() and not self.recovery_ready():
                        raise LocalizationUnavailable("跟踪中断，等待定位数据恢复")
                    if attempts >= self.MAX_DETOURS:
                        raise RuntimeError("绕行次数达到上限，请检查障碍物后重新启动")
                    original_index = mapping[index]
                    replacement = None
                    self.update("executing", "停车，规划局部绕行路径")
                    for target in rejoin_indices(original, original_index):
                        # Smac validates rejoin points beyond the local window.
                        if blocked_ahead(original, target, grid, self.footprint, distance=0,
                                         outside_is_blocked=False):
                            continue
                        detour = self._plan(pose, original[target])
                        if detour:
                            replacement = detour + original[target+1:]
                            replacement_mapping = [original_index] * (len(detour)-1) + [target] + list(range(target+1, len(original)))
                            break
                    if replacement is None:
                        raise RuntimeError("没有安全绕行路径，车辆保持停止")
                    path, attempts = replacement, attempts + 1
                    mapping, index = replacement_mapping, 0
                except LocalizationUnavailable:
                    self._cancel(handle)
                    handle = None
                    recovered = self._wait_for_localization()
                    if recovered is None:
                        break
                    pose, grid = recovered
                    # Retain ordered progress rather than selecting a globally
                    # nearest point on an adjacent row or the return leg.
                    index = forward_index(path, pose, index)
                    path, mapping = path[index:], mapping[index:]
                    index, connect = 0, True
                    self.update("executing", "定位已稳定恢复，继续剩余路线")
        except Exception as error:
            outcome = ("canceled" if self.stop_event.is_set() else "failed", str(error))
        finally:
            try:
                self._cancel(handle)
            except Exception as error:
                self.node.get_logger().error(f"停止路径跟踪失败: {error}")
                outcome = ("failed", f"停止路径跟踪失败: {error}")
            self.update(*(outcome or ("canceled", "保存路径任务已取消")))
