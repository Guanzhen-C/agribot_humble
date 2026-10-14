import importlib.util
from pathlib import Path

from nav_msgs.msg import Odometry


spec = importlib.util.spec_from_file_location(
    "simulation_initial_pose", Path(__file__).resolve().parents[1] / "scripts/simulation_initial_pose.py")
module = importlib.util.module_from_spec(spec)
spec.loader.exec_module(module)


def odometry(seconds, x=2.0):
    result = Odometry()
    result.header.stamp.sec = int(seconds)
    result.header.stamp.nanosec = round((seconds - int(seconds)) * 1e9)
    result.pose.pose.position.x = x
    result.pose.pose.orientation.x = 0.01
    result.pose.pose.orientation.z = -0.005
    result.pose.pose.orientation.w = 0.9999
    return result


def test_seed_is_synchronized_full_pose_and_one_shot():
    buffer = module.InitialPoseBuffer()
    truth = odometry(2.0)
    buffer.truth.extend([odometry(1.8, 9.0), truth])
    result = buffer.seed(odometry(2.02))
    assert result.pose.pose == truth.pose.pose
    assert result.header.frame_id == "map"
    assert result.header.stamp == odometry(2.02).header.stamp
    assert not buffer.truth
    buffer.truth.append(odometry(3.0, 90.0))
    assert buffer.seed(odometry(3.0)) is None


def test_stale_and_pre_settling_poses_are_not_used():
    buffer = module.InitialPoseBuffer()
    buffer.truth.append(odometry(0.2))
    assert buffer.seed(odometry(0.2)) is None
    assert buffer.seed(odometry(2.0)) is None
    assert not buffer.done


def test_truth_history_is_bounded():
    buffer = module.InitialPoseBuffer()
    buffer.truth.extend(odometry(2.0 + i * 0.01) for i in range(500))
    assert len(buffer.truth) == 100
