from glob import glob
from pathlib import Path
from setuptools import find_packages, setup

name = "agribot_manipulator_sim"
setup(
    name=name, version="0.1.0", packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + name]),
        ("share/" + name, ["package.xml", "README.md"]),
        ("share/" + name + "/models/lite6", [path for path in glob("models/lite6/*") if Path(path).is_file()]),
        ("share/" + name + "/models/lite6/meshes", glob("models/lite6/meshes/*")),
        ("share/" + name + "/rviz", glob("rviz/*")),
    ],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="cgz", maintainer_email="cgz@example.com", license="MIT AND BSD-3-Clause",
    description="Optional vehicle-mounted arm movement demonstration.",
    entry_points={"console_scripts": ["arm_demo = agribot_manipulator_sim.arm_demo:main"]},
)
