import math
import sys
from pathlib import Path

import numpy as np


PACKAGE_ROOT = Path(__file__).parents[1]
sys.path.insert(0, str(PACKAGE_ROOT / "scripts"))

from visual_place_recognizer import (  # noqa: E402
    aggregate_candidates,
    load_database,
    rank_descriptor,
)


def test_database_requires_base_link_pose_and_normalizes_descriptors(tmp_path):
    path = tmp_path / "index.npz"
    np.savez(
        path,
        descriptors=np.asarray([[2.0, 0.0], [0.0, 3.0]], dtype=np.float32),
        x=np.asarray([1.0, 2.0]),
        y=np.asarray([3.0, 4.0]),
        yaw_rad=np.asarray([0.1, 0.2]),
        pose_frame=np.asarray("base_link"),
    )
    descriptors, x, y, yaw = load_database(path)
    assert np.allclose(np.linalg.norm(descriptors, axis=1), 1.0)
    assert np.allclose(x, [1.0, 2.0])
    assert np.allclose(y, [3.0, 4.0])
    assert np.allclose(yaw, [0.1, 0.2])


def test_temporal_voting_returns_stable_ordered_pose_candidates():
    database = np.eye(4, dtype=np.float32)
    indices, scores = rank_descriptor(
        np.asarray([0.9, 0.85, 0.1, 0.0], dtype=np.float32)
        / np.linalg.norm([0.9, 0.85, 0.1, 0.0]),
        database,
        4,
    )
    observations = [(indices, scores)] * 5
    candidates = aggregate_candidates(
        observations,
        np.asarray([0.0, 0.4, 8.0, 12.0]),
        np.asarray([0.0, 0.1, 0.0, 0.0]),
        np.asarray([0.0, 0.05, math.pi, math.pi]),
        minimum_similarity=0.60,
        cluster_xy_radius=1.5,
        cluster_yaw_rad=math.radians(50.0),
        candidate_limit=3,
    )
    assert len(candidates) == 1
    assert 0.0 <= candidates[0].x <= 0.4
    assert candidates[0].support == 5
    assert abs(candidates[0].yaw) < 0.1


def test_low_similarity_never_publishes_a_candidate():
    candidates = aggregate_candidates(
        [(np.asarray([0]), np.asarray([0.5]))] * 5,
        np.asarray([1.0]),
        np.asarray([2.0]),
        np.asarray([0.0]),
        minimum_similarity=0.75,
        cluster_xy_radius=1.5,
        cluster_yaw_rad=math.radians(50.0),
        candidate_limit=3,
    )
    assert candidates == []
