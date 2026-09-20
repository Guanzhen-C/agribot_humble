from glob import glob
import os

from setuptools import find_packages, setup


package_name = "agribot_visual_perception"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="cgz",
    maintainer_email="cgz@example.com",
    description=(
        "Independent OpenCV visual perception algorithms that do not affect "
        "the vehicle motion-control chain."
    ),
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "visual_perception_node = "
            "agribot_visual_perception.visual_perception_node:main",
        ],
    },
)
