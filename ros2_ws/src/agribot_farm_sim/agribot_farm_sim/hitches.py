"""Inspected CAD pin locations and nominal rigid three-point transport adapters.

These are display/transport adapters, not certified real-machine hitch designs.
Coordinates are in each normalized export's frame, never its bounding-box frame.
"""

import math
import copy
from pathlib import Path

import numpy as np

from agribot_vehicle_config_tools.model import ConfigError


HOSTS = {
    "unity_car1": (2.8137142369, [
        ([-1.083, .404, .272], "NS_John_Deere_6195M_Rear_Holder_02"),
        ([-1.083, -.404, .272], "NS_John_Deere_6195M_Rear_Holder_02"),
        ([-.55, 0., .75], "NS_John_Deere_6195M_Rear_Holder_01")]),
    "unity_car2": (2.7586563347, [
        ([-1.054, .472, -.359], "Brack_Arm_Load"),
        ([-1.054, -.4704, -.359], "MirrorBrack_Arm_Load"),
        ([-.145, 0., .44], "Rear_Linkage_Arm")]),
    "unity_car4": (2.1851241913, [
        ([-.964, .4526, -.115], "0042_Lift_LowerArm"),
        ([-.964, -.4548, -.119], "0044_Lift_LowerArm"),
        ([-.26, 0., .36], "0055_Liftng_Block")]),
    "unity_car5": (1.6656807003, [
        ([-.91, .216, -.101], "0037_Product1_AllCATPart"),
        ([-.91, -.246, -.101], "0036_Product1_AllCATPart"),
        ([-.39, -.018, .12], "0049_Product1_AllCATPart")]),
    "compact_tractor": (1.7793064117, [
        ([-.452, .2925, -.13], "solid_10"),
        ([-.452, -.2925, -.13], "solid_18"),
        ([-.21, 0., .28], "solid_8")]),
    "cab_tractor": (3.0372819901, [
        ([-1.185, .517, -.417], "solid_94"),
        ([-1.185, -.517, -.417], "solid_93"),
        ([-.16, 0., .484], "solid_75")]),
}
TOOLS = {
    "compact_chisel_plow": [
        ([-.103, .384, .588], "solid_513"),
        ([-.103, -.384, .588], "solid_516"),
        ([-.118, 0., 1.213], "solid_525")],
    "mounted_disc_harrow": [
        ([-.06, .2913, .565], "solid_29"),
        ([-.06, -.2917, .565], "solid_34"),
        ([-.0623, -.0002, 1.0408], "solid_40")],
    "mounted_cultivator": [
        ([-.138, .410, .4458], "solid_137"),
        ([-.138, -.410, .4458], "solid_138"),
        ([-.035, .0005, 1.0141], "solid_139")],
    "forest_chain_crusher": [
        ([-.028, .404, .556], "solid_1222"),
        ([-.028, -.404, .557], "solid_1223"),
        ([-.069, 0., 1.269], "solid_1221")],
    "inter_row_milling_cutter": [
        ([-.106, .2824, .87], "solid_2641"),
        ([-.106, -.2824, .87], "solid_2638"),
        ([-.11, 0., 1.372], "solid_2580")],
    "groove_maker": [
        ([-.46, .36, .958], "solid_13"),
        ([-.46, -.36, .958], "solid_14"),
        ([-.204, -.0046, 1.5201], "solid_341")],
    "garlic_planter": [
        ([.045, .30, .485], "hitch_adapter_lower_left_lug"),
        ([.045, -.30, .485], "hitch_adapter_lower_right_lug"),
        ([.045, 0., 1.045], "hitch_adapter_top_lug")],
    "mulch_planter": [
        ([.06, .30, .54], "hitch_adapter_lower_left_lug"),
        ([.06, -.30, .54], "hitch_adapter_lower_right_lug"),
        ([.06, 0., 1.10], "hitch_adapter_top_lug")],
}

# These tools have frame rails, not inspected three-point pins. The receiver
# is an explicit nominal addition clamped to four locations on those rails.
RECEIVER_FRAMES = {
    "garlic_planter": {"footings": [
        ([-.18, .405, .4329], "solid_866"),
        ([-.18, -.435, .4346], "solid_864"),
        ([-.40, .405, .4329], "solid_866"),
        ([-.40, -.435, .4346], "solid_864")]},
    "mulch_planter": {"footings": [
        ([-.23, .4165, .4803], "solid_21"),
        ([-.23, -.3535, .4803], "solid_22"),
        ([-.45, .4165, .4803], "solid_21"),
        ([-.45, -.3535, .4803], "solid_22")]},
}


def layout(vehicle, kind, pose):
    """Raise to the lower hooks where possible, retaining minimum ground clearance."""
    if vehicle["vehicleId"] not in HOSTS or kind not in TOOLS:
        return None
    wheelbase, host = HOSTS[vehicle["vehicleId"]]
    if abs(vehicle["geometry"]["wheelbaseM"] - wheelbase) > .001:
        raise ConfigError("inspected hitch anchors do not match this vehicle scale")
    tool = TOOLS[kind]
    translation = np.asarray(pose, dtype=float).copy()
    translation[2] = max(translation[2], np.mean([p[0][2] for p in host[:2]]) -
                        np.mean([p[0][2] for p in tool[:2]]))
    links = []
    for name, (start, host_part), (end, tool_part) in zip(
            ("lower_left", "lower_right", "top_link"), host, tool):
        start_local = np.asarray(start) - translation
        length = float(np.linalg.norm(np.asarray(end) - start_local))
        if not .1 < length < 1.8:
            raise ConfigError("inspected transport hitch link is outside its geometry envelope")
        links.append(dict(name=name, hostPoint=start, hostPart=host_part,
                          toolPoint=end, toolPart=tool_part,
                          start=start_local.tolist(), end=end, lengthM=length))
    receiver = copy.deepcopy(RECEIVER_FRAMES.get(kind))
    if receiver:
        a, b, c = [point for point, _ in tool]
        feet = [point for point, _ in receiver["footings"]]
        receiver["members"] = [(a, b), (a, c), (b, c), (feet[0], a),
                               (feet[1], b), (feet[2], c), (feet[3], c)]
    return dict(pose=translation.tolist(), links=links, receiverFrame=receiver,
                basis="Inspected normalized CAD hooks/pins with nominal three-point adapter links; "
                      "frame-only tools use a separate nominal rail-clamped receiver. "
                      "Rigid raised transport only, not a lifting-capacity or hardware-fit certification.")


def _beam(name, a, b, width, depth):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    axis = b - a
    axis /= np.linalg.norm(axis)
    seed = np.array([0., 0., 1.]) if abs(axis[2]) < .9 else np.array([0., 1., 0.])
    u = np.cross(axis, seed)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    corners = [(-1, -1), (1, -1), (1, 1), (-1, 1)]
    vertices = [end + x * width / 2 * u + y * depth / 2 * v
                for end in (a, b) for x, y in corners]
    quads = [(0, 3, 2, 1), (4, 5, 6, 7), (0, 1, 5, 4),
             (1, 2, 6, 5), (2, 3, 7, 6), (3, 0, 4, 7)]
    return dict(name=name, vertices=np.asarray(vertices),
                faces=[face for q in quads for face in ((q[0], q[1], q[2]), (q[0], q[2], q[3]))],
                material="hitch_painted_steel")


def _lathe(name, a, b, profile, material, segments=16):
    a, b = np.asarray(a, dtype=float), np.asarray(b, dtype=float)
    axis = b - a
    axis /= np.linalg.norm(axis)
    seed = np.array([1., 0., 0.]) if abs(axis[0]) < .9 else np.array([0., 1., 0.])
    u = np.cross(axis, seed)
    u /= np.linalg.norm(u)
    v = np.cross(axis, u)
    vertices = [a + axial * axis + radius * (math.cos(theta) * u + math.sin(theta) * v)
                for axial, radius in profile
                for theta in np.linspace(0, 2 * math.pi, segments, endpoint=False)]
    faces = []
    for ring in range(len(profile)):
        for i in range(segments):
            quad = [ring * segments + i, ring * segments + (i + 1) % segments,
                    ((ring + 1) % len(profile)) * segments + (i + 1) % segments,
                    ((ring + 1) % len(profile)) * segments + i]
            for face in ((quad[0], quad[1], quad[2]), (quad[0], quad[2], quad[3])):
                p, q, r = [vertices[j] for j in face]
                if np.linalg.norm(np.cross(q - p, r - p)) > 1e-12:
                    faces.append(face)
    return dict(name=name, vertices=np.asarray(vertices), faces=faces, material=material)


def geometry(hitch):
    """Low-poly steel links, pin eyes, transverse pins and retaining collars."""
    pieces = []
    receiver = hitch.get("receiverFrame")
    if receiver:
        for i, (start, end) in enumerate(receiver["members"]):
            pieces.append(_beam(f"hitch_receiver_member_{i}", start, end, .04, .04))
        for i, (point, _) in enumerate(receiver["footings"]):
            point = np.asarray(point)
            pieces.append(_beam(f"hitch_receiver_clamp_{i}", point + [-.055, 0, .005],
                                point + [.055, 0, .005], .09, .012))
            for j, (x, y) in enumerate(((-.035, -.025), (-.035, .025), (.035, -.025), (.035, .025))):
                a, b = point + [x, y, -.015], point + [x, y, .025]
                pieces.append(_lathe(f"hitch_receiver_bolt_{i}_{j}", a, b,
                                     [(0, .006), (.028, .006), (.028, .011),
                                      (.04, .011), (.04, 0), (0, 0)], "hitch_pin_steel", segments=8))
    for link in hitch["links"]:
        name = "hitch_" + link["name"]
        a, b = np.asarray(link["start"]), np.asarray(link["end"])
        length = np.linalg.norm(b - a)
        pieces.append(_lathe(name + "_rod", a, b,
                            [(.025, .018), (length - .025, .018),
                             (length - .025, 0), (.025, 0)], "hitch_pin_steel"))
        pieces.append(_lathe(name + "_sleeve", a, b,
                            [(length * .22, .028), (length * .78, .028),
                             (length * .78, .018), (length * .22, .018)], "hitch_painted_steel"))
        for side, point in (("host", a), ("tool", b)):
            start, end = point + [0., -.014, 0.], point + [0., .014, 0.]
            pieces.append(_lathe(name + "_" + side + "_eye", start, end,
                                [(0, .042), (.028, .042), (.028, .015), (0, .015)],
                                "hitch_painted_steel"))
            start, end = point + [0., -.085, 0.], point + [0., .085, 0.]
            pieces.append(_lathe(name + "_" + side + "_pin", start, end,
                                [(0, .014), (.17, .014), (.17, 0), (0, 0)], "hitch_pin_steel"))
            for sign in (-1, 1):
                start = point + [0., sign * .077 - .003, 0.]
                pieces.append(_lathe(name + "_" + side + "_retainer_" + str(sign).replace("-", "m"),
                                    start, start + [0., .006, 0.],
                                    [(0, .024), (.006, .024), (.006, .014), (0, .014)], "hitch_pin_steel"))
    return pieces


def collision_boxes(hitch):
    boxes = []
    pieces = geometry(hitch)
    for link in hitch["links"]:
        name = "hitch_" + link["name"]
        vertices = np.concatenate([piece["vertices"] for piece in pieces
                                   if piece["name"].startswith(name + "_")])
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        boxes.append(dict(name=name, center=((low + high) / 2).tolist(),
                          size=(high - low + .002).tolist()))
    receiver = [piece["vertices"] for piece in pieces if piece["name"].startswith("hitch_receiver_")]
    if receiver:
        vertices = np.concatenate(receiver)
        low, high = vertices.min(axis=0), vertices.max(axis=0)
        boxes.append(dict(name="hitch_receiver_frame", center=((low + high) / 2).tolist(),
                          size=(high - low + .002).tolist()))
    return boxes


def append_mesh(mesh, material, hitch):
    # Relative face indices keep the original CAD indices and normals untouched.
    with Path(mesh).open("a") as stream:
        for piece in geometry(hitch):
            vertices = piece["vertices"]
            stream.write("\no " + piece["name"] + "\nusemtl " + piece["material"] + "\ns off\n")
            for point in vertices:
                stream.write("v " + " ".join(f"{x:.10g}" for x in point) + "\n")
            for face in piece["faces"]:
                stream.write("f " + " ".join(str(index - len(vertices)) for index in face) + "\n")
    with Path(material).open("a") as stream:
        stream.write("\nnewmtl hitch_painted_steel\nKa .16 .18 .19\nKd .16 .18 .19\n"
                     "\nnewmtl hitch_pin_steel\nKa .48 .50 .52\nKd .48 .50 .52\n")
