from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "agribot_bev"
ROOT = Path(__file__).parent


setup(
    name=PACKAGE_NAME,
    version="0.1.0",
    packages=find_packages(exclude=("test",)),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{PACKAGE_NAME}"],
        ),
        (f"share/{PACKAGE_NAME}", ["package.xml", "README.md"]),
        (
            f"share/{PACKAGE_NAME}/config",
            [str(path) for path in sorted((ROOT / "config").glob("*.yaml"))],
        ),
        (
            f"share/{PACKAGE_NAME}/launch",
            [str(path) for path in sorted((ROOT / "launch").glob("*.launch.py"))],
        ),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="cgz",
    maintainer_email="cgz@example.com",
    description="Four-camera ground-plane BEV and local occupancy grid for Agribot.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "surround_bev = agribot_bev.surround_bev_node:main",
        ]
    },
)
