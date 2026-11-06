"""Read-only prospective Smac route check; never sends a FollowPath goal."""
import argparse
import json
import math
from pathlib import Path

from action_msgs.msg import GoalStatus
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathThroughPoses
import rclpy

from .navigation import Navigator


def path_metrics(rows):
    directions = []
    distance = 0.
    for first, second in zip(rows, rows[1:]):
        dx, dy = second[0]-first[0], second[1]-first[1]
        distance += math.hypot(dx, dy)
        if math.hypot(dx, dy) < .001:
            continue
        yaw = 2*math.atan2(first[2], first[3])
        projection = dx*math.cos(yaw)+dy*math.sin(yaw)
        directions.append(1 if projection >= 0. else -1)
    return {'length_m': distance, 'direction_changes': sum(a != b for a, b in zip(directions, directions[1:])),
            'reverse_segments': directions.count(-1), 'forward_segments': directions.count(1)}


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--scenario', type=Path, default=Path.home()/'agribot_hil_ws/scenario')
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--pickup-index', type=int, help='Check a forward leg between consecutive pickup stops')
    parser.add_argument('--finish-yaw', type=float, help='Prospective final approach yaw only; no execution')
    args = parser.parse_args()
    manifest = json.loads((args.scenario/'mission.json').read_text())
    rclpy.init()
    node = Navigator()
    try:
        node.wait_active('planner_server')
        if not node.planner.wait_for_server(timeout_sec=5.):
            raise RuntimeError('Smac unavailable')
        goal = ComputePathThroughPoses.Goal(planner_id='GridBased', use_start=True)
        stops = manifest.get('pickup_stops', [{'park': manifest['park']}])
        start = stops[-1]['park']
        points = manifest['completion_via'] if 'completion_via' in manifest else manifest['return_via']
        if args.pickup_index is not None:
            if not 1 <= args.pickup_index < len(stops):
                raise ValueError('Pickup leg index must identify an existing consecutive pair')
            start, points = stops[args.pickup_index-1]['park'], [stops[args.pickup_index]['park']]
        if args.finish_yaw is not None:
            if not math.isfinite(args.finish_yaw):
                raise ValueError('Prospective final yaw must be finite')
            points = [list(point) for point in points]
            points[-1][2] = args.finish_yaw
        for index, point in enumerate([start]+points):
            p = PoseStamped()
            p.header.frame_id = 'map'
            p.header.stamp = node.get_clock().now().to_msg()
            p.pose.position.x, p.pose.position.y = point[:2]
            p.pose.orientation.z, p.pose.orientation.w = math.sin(point[2]/2), math.cos(point[2]/2)
            if index == 0:
                goal.start = p
            else:
                goal.goals.append(p)
        handle = node.wait(node.planner.send_goal_async(goal), 10.)
        if not handle.accepted:
            raise RuntimeError('Smac request rejected')
        result = node.wait(handle.get_result_async(), 90.)
        rows = [[p.pose.position.x, p.pose.position.y, p.pose.orientation.z,
                 p.pose.orientation.w] for p in result.result.path.poses]
        report = {'status': result.status, 'executed': False, 'start': start,
                  'goals': points, 'path': rows, **path_metrics(rows)}
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        print(json.dumps({k: v for k, v in report.items() if k != 'path'}), flush=True)
        if result.status != GoalStatus.STATUS_SUCCEEDED:
            raise RuntimeError('Prospective route not found')
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
