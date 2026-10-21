"""Closed-loop CAD tread animation, separate from vehicle contact physics."""

import bisect
import math
import shutil
import xml.etree.ElementTree as ET

from .visual_rotors import uri


class BeltPath:
    def __init__(self, points):
        self.points = [tuple(p) for p in points]
        if len(self.points) < 3 or any(len(p) != 2 or not all(math.isfinite(v) for v in p) for p in self.points):
            raise ValueError("A belt needs a finite closed XZ path")
        self.distances = [0.0]
        for a, b in zip(self.points, self.points[1:] + self.points[:1]):
            length = math.hypot(b[0]-a[0], b[1]-a[1])
            if length < 1e-8:
                raise ValueError("Belt path has a zero-length edge")
            self.distances.append(self.distances[-1] + length)
        self.length = self.distances[-1]

    def sample(self, distance):
        distance %= self.length
        index = min(bisect.bisect_right(self.distances, distance)-1, len(self.points)-1)
        a, b = self.points[index], self.points[(index+1) % len(self.points)]
        fraction = (distance-self.distances[index])/(self.distances[index+1]-self.distances[index])
        return a[0]+fraction*(b[0]-a[0]), a[1]+fraction*(b[1]-a[1]), -math.atan2(b[1]-a[1], b[0]-a[0])

    def project(self, x, z):
        best = (math.inf, 0.0)
        for index, (a, b) in enumerate(zip(self.points, self.points[1:] + self.points[:1])):
            dx, dz = b[0]-a[0], b[1]-a[1]
            t = max(0.0, min(1.0, ((x-a[0])*dx+(z-a[1])*dz)/(dx*dx+dz*dz)))
            distance = math.hypot(x-a[0]-t*dx, z-a[1]-t*dz)
            if distance < best[0]:
                best = distance, self.distances[index]+t*math.hypot(dx, dz)
        return best


def belts(config):
    items = config.get("metadata", {}).get("simulationVisualBelts", [])
    for item in items:
        if item["sourceWheel"] not in {w["id"] for w in config["geometry"]["wheelCenters"]}:
            raise ValueError("Unknown belt source wheel")
        BeltPath(item["points"])
        for part in item["segments"]:
            if part["meshFile"] != "belt_" + item["id"] + "_" + str(part["index"]) + ".obj":
                raise ValueError("Belt segment must be an export-local mesh")
    return items


def append_sdf(model, config):
    for item in belts(config):
        path = BeltPath(item["points"])
        for part in item["segments"]:
            x, z, angle = path.sample(part["distance"])
            visual = ET.SubElement(model.find("link[@name='base_link']"), "visual", name="belt_" + item["id"] + "_" + str(part["index"]))
            ET.SubElement(visual, "pose").text = f"{x} {item['centerY']} {z} 0 {angle} 0"
            ET.SubElement(ET.SubElement(ET.SubElement(visual, "geometry"), "mesh"), "uri").text = uri(config, part["meshFile"])
            plugin = ET.SubElement(visual, "plugin", name="belt_animation", filename="libagribot_simulation_visual_rotor.so")
            ET.SubElement(plugin, "source_link").text = item["sourceWheel"] + "_wheel_link"
            ET.SubElement(plugin, "ratio").text = str(item["sourceRadius"])
            ET.SubElement(plugin, "path").text = " ".join(str(v) for point in item["points"] for v in point)
            ET.SubElement(plugin, "distance").text = str(part["distance"])
            ET.SubElement(plugin, "center_y").text = str(item["centerY"])


def copy_assets(config, source, output):
    for item in belts(config):
        for part in item["segments"]:
            shutil.copy2(source / part["meshFile"], output / part["meshFile"])
