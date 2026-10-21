import copy

import numpy as np
import pytest

from agribot_farm_sim import hitches
from agribot_vehicle_config_tools.model import ConfigError


@pytest.mark.parametrize("vehicle_id", hitches.HOSTS)
@pytest.mark.parametrize("tool", hitches.TOOLS)
def test_three_point_adapter_reaches_both_inspected_endpoints(vehicle_id, tool):
    vehicle = {"vehicleId": vehicle_id, "geometry": {"wheelbaseM": hitches.HOSTS[vehicle_id][0]}}
    before = copy.deepcopy(vehicle)
    hitch = hitches.layout(vehicle, tool, [-1.45, 0., -.55])
    assert vehicle == before
    assert hitch["pose"][2] >= -.55
    assert len(hitch["links"]) == 3
    for link in hitch["links"]:
        assert np.asarray(link["start"]) + hitch["pose"] == pytest.approx(link["hostPoint"])
        assert link["end"] == link["toolPoint"]
        assert link["hostPart"] and link["toolPart"]
    boxes = hitches.collision_boxes(hitch)
    assert len(boxes) == (4 if tool in hitches.RECEIVER_FRAMES else 3)
    for piece in hitches.geometry(hitch):
        assert np.isfinite(piece["vertices"]).all()
        assert piece["faces"]
        for point in piece["vertices"]:
            assert any(np.all(abs(point - box["center"]) <= np.asarray(box["size"]) / 2)
                       for box in boxes)


@pytest.mark.parametrize("tool", hitches.RECEIVER_FRAMES)
def test_receiver_is_a_connected_frame_attached_to_all_inspected_rails(tool):
    vehicle = {"vehicleId": "compact_tractor", "geometry": {"wheelbaseM": hitches.HOSTS["compact_tractor"][0]}}
    hitch = hitches.layout(vehicle, tool, [-.9, 0, -.44])
    frame = hitch["receiverFrame"]
    assert frame["footings"] == hitches.RECEIVER_FRAMES[tool]["footings"]
    assert len(frame["footings"]) == 4
    members = [(tuple(a), tuple(b)) for a, b in frame["members"]]
    reached = {tuple(frame["footings"][0][0])}
    while True:
        expanded = reached | {point for a, b in members if a in reached or b in reached for point in (a, b)}
        if expanded == reached:
            break
        reached = expanded
    expected = {tuple(p) for p, _ in frame["footings"]} | {tuple(l["toolPoint"]) for l in hitch["links"]}
    assert reached == expected
    frame["footings"].clear()
    assert len(hitches.RECEIVER_FRAMES[tool]["footings"]) == 4


def test_changed_scale_requires_reinspection():
    with pytest.raises(ConfigError, match="scale"):
        hitches.layout({"vehicleId": "unity_car4", "geometry": {"wheelbaseM": 1.}},
                       "compact_chisel_plow", [-1.3, 0., -.56])


def test_unknown_host_does_not_get_fabricated_anchors():
    assert hitches.layout({"vehicleId": "unknown"}, "compact_chisel_plow", [0, 0, 0]) is None


def test_append_keeps_source_cad_and_indices(tmp_path):
    mesh, material = tmp_path / "tool.obj", tmp_path / "tool.mtl"
    original = "mtllib tool.mtl\nv 0 0 0\nv 1 0 0\nv 0 1 0\nf 1 2 3\n"
    mesh.write_text(original)
    material.write_text("newmtl original\nKd .1 .1 .1\n")
    hitch = hitches.layout({"vehicleId": "unity_car4", "geometry": {"wheelbaseM": hitches.HOSTS["unity_car4"][0]}},
                           "compact_chisel_plow", [-1.3, 0., -.56])
    hitches.append_mesh(mesh, material, hitch)
    assert mesh.read_text().startswith(original)
    assert "mount_bridge" not in mesh.read_text()
    vertices = 0
    for line in mesh.read_text().splitlines():
        if line.startswith("v "):
            vertices += 1
        elif line.startswith("f "):
            indices = [int(x) for x in line.split()[1:]]
            assert all(0 <= (i - 1 if i > 0 else vertices + i) < vertices for i in indices)
