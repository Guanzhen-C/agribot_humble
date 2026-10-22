"""Preserve CAD display colours in Gazebo's non-PBR OBJ renderer."""

from pathlib import Path


def canonical_diffuse(name, diffuse):
    # These CAD rubber materials lost their texture and exported a white multiplier.
    if "细孔橡胶" in name and all(abs(float(value) - 1.) < 1e-6 for value in diffuse):
        return (.035, .040, .045)
    return tuple(float(value) for value in diffuse)


def normalize_ambient(path):
    path = Path(path)
    groups, current = [], []
    for line in path.read_text(encoding="utf-8").splitlines(keepends=True):
        if line.startswith("newmtl ") and current:
            groups.append(current)
            current = []
        current.append(line)
    groups.append(current)
    result = []
    for group in groups:
        diffuse = next((line.split()[1:4] for line in group if line.startswith("Kd ")), None)
        if diffuse is not None:
            name = next((line[7:].strip() for line in group if line.startswith("newmtl ")), "")
            corrected = canonical_diffuse(name, diffuse)
            if corrected != tuple(float(value) for value in diffuse):
                diffuse = [format(value, ".6g") for value in corrected]
                group = [("Kd " + " ".join(diffuse) + "\n") if line.startswith("Kd ") else line
                         for line in group]
            # Blender exports white Ka even for black rubber. Gazebo uses it
            # independently from Kd, so the scene ambient washes out the model.
            group = [("Ka " + " ".join(diffuse) + "\n") if line.startswith("Ka ") else line
                     for line in group]
        result.extend(group)
    path.write_text("".join(result), encoding="utf-8")
