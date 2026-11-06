"""Check full-pose IK and collision-checked OMPL plans on the X5, without execution."""
import argparse
import json
import math
from pathlib import Path

from moveit_msgs.msg import Constraints, JointConstraint
from moveit_msgs.srv import GetMotionPlan, GetPositionFK, GetPositionIK, GetStateValidity
import numpy as np
import rclpy
from rclpy.node import Node
from scipy.spatial.transform import Rotation


JOINTS = [f'manipulator_joint{i}' for i in range(1, 7)]
FINGERS = ['manipulator_finger_joint1', 'manipulator_finger_joint2']
STOW = [0., 0., 1.57, 0., 0., 0., .0089, .0089]


def approach_orientation(target, shoulder, stem_axis=(0., 0., 1.)):
    direction = np.asarray(target) - np.asarray(shoulder)
    length = np.linalg.norm(direction)
    if length < 1e-8:
        raise ValueError('Grasp target coincides with the shoulder')
    direction /= length
    closing = np.cross(stem_axis, direction)
    length = np.linalg.norm(closing)
    if length < 1e-8:
        raise ValueError('A vertical approach does not define a horizontal jaw axis')
    closing /= length
    return Rotation.from_matrix(np.column_stack([
        np.cross(closing, direction), closing, direction])).as_quat()


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--mission', type=Path, required=True)
    parser.add_argument('--base-height', type=float, required=True,
                        help='Prospective physical base height, not the LIO map-Z estimate')
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    mission = json.loads(args.mission.read_text())
    rclpy.init()
    node = Node('hil_arm_preflight')
    report = {'executed': False, 'prospective_park': mission['park'],
              'base_height': args.base_height, 'stow': STOW, 'apples': []}

    def call(service, kind, request):
        client = node.create_client(kind, service)
        try:
            if not client.wait_for_service(timeout_sec=5.):
                raise RuntimeError(f'{service} unavailable')
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=20.)
            if not future.done() or future.result() is None:
                raise TimeoutError(service)
            return future.result()
        finally:
            node.destroy_client(client)

    try:
        state = GetStateValidity.Request()
        state.group_name = 'arm'
        state.robot_state.joint_state.name = JOINTS + FINGERS
        state.robot_state.joint_state.position = STOW
        report['stow_valid'] = call('/check_state_validity', GetStateValidity, state).valid
        if not report['stow_valid']:
            raise RuntimeError('Proposed initial assembly pose is in collision')
        fk = GetPositionFK.Request()
        fk.header.frame_id = 'base_link'
        fk.fk_link_names = ['manipulator_link1']
        fk.robot_state = state.robot_state
        result = call('/compute_fk', GetPositionFK, fk)
        if result.error_code.val != 1 or not result.pose_stamped:
            raise RuntimeError('Cannot obtain the actual mounted shoulder position')
        p = result.pose_stamped[0].pose.position
        shoulder = [p.x, p.y, p.z]
        x, y, yaw = mission['park']
        for index, apple in enumerate(mission['grasp_targets']):
            dx, dy = apple[0]-x, apple[1]-y
            position = [math.cos(yaw)*dx+math.sin(yaw)*dy,
                        -math.sin(yaw)*dx+math.cos(yaw)*dy,
                        apple[2]-args.base_height]
            orientation = approach_orientation(position, shoulder)
            request = GetPositionIK.Request()
            ik = request.ik_request
            ik.group_name, ik.ik_link_name = 'arm', 'manipulator_tcp'
            ik.robot_state = state.robot_state
            ik.pose_stamped.header.frame_id = 'base_link'
            pose = ik.pose_stamped.pose
            pose.position.x, pose.position.y, pose.position.z = position
            pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = map(float, orientation)
            ik.timeout.nanosec, ik.avoid_collisions = 500_000_000, True
            solution = call('/compute_ik', GetPositionIK, request)
            entry = {'apple': index+1, 'target_in_base': position,
                     'orientation_in_base': orientation.tolist(), 'ik_code': solution.error_code.val}
            report['apples'].append(entry)
            if solution.error_code.val != 1:
                raise RuntimeError(f'Apple {index+1} has no collision-free full-pose IK')
            request = GetMotionPlan.Request()
            motion = request.motion_plan_request
            motion.group_name, motion.pipeline_id = 'arm', 'ompl'
            motion.planner_id = 'RRTConnectkConfigDefault'
            motion.allowed_planning_time, motion.num_planning_attempts = 5., 3
            motion.max_velocity_scaling_factor = motion.max_acceleration_scaling_factor = .5
            motion.start_state = state.robot_state
            constraint = Constraints()
            for name, value in zip(solution.solution.joint_state.name, solution.solution.joint_state.position):
                if name in JOINTS:
                    constraint.joint_constraints.append(JointConstraint(
                        joint_name=name, position=value, tolerance_above=.005,
                        tolerance_below=.005, weight=1.))
            motion.goal_constraints = [constraint]
            plan = call('/plan_kinematic_path', GetMotionPlan, request).motion_plan_response
            entry.update(plan_code=plan.error_code.val, planning_time=plan.planning_time,
                         planned_points=len(plan.trajectory.joint_trajectory.points))
            print(json.dumps(entry), flush=True)
            if plan.error_code.val != 1 or not plan.trajectory.joint_trajectory.points:
                raise RuntimeError(f'Apple {index+1} OMPL planning failed')
    finally:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(report, indent=2)+'\n')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
