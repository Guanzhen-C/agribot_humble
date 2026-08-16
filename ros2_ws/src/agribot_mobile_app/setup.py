from pathlib import Path

from setuptools import find_packages, setup


PACKAGE_NAME = "agribot_mobile_app"
ROOT = Path(__file__).parent


def installed_tree(source, destination):
    entries = []
    for path in sorted((ROOT / source).rglob("*")):
        if path.is_file():
            relative_parent = path.relative_to(ROOT / source).parent
            entries.append(
                (str(Path(destination) / relative_parent), [str(path)])
            )
    return entries


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
        *installed_tree("launch", f"share/{PACKAGE_NAME}/launch"),
        *installed_tree("config", f"share/{PACKAGE_NAME}/config"),
        *installed_tree("systemd", f"share/{PACKAGE_NAME}/systemd"),
        *installed_tree("web/dist", f"share/{PACKAGE_NAME}/web"),
    ],
    install_requires=["setuptools"],
    zip_safe=False,
    maintainer="cgz",
    maintainer_email="cgz@example.com",
    description=(
        "Installable mobile PWA and guarded ROS 2 gateway for Agribot operations."
    ),
    license="GPL-3.0-only",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "mobile_gateway = agribot_mobile_app.gateway_node:main",
        ]
    },
)
