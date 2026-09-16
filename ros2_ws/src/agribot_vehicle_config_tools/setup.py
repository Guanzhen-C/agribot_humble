from setuptools import find_packages, setup


package_name = "agribot_vehicle_config_tools"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/schema", ["schema/vehicle_config.schema.json"]),
    ],
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="cgz",
    maintainer_email="cgz@example.com",
    description=(
        "Validate Unity vehicle exports and generate consistent ROS 2 vehicle "
        "configuration bundles."
    ),
    license="MIT",
    entry_points={
        "console_scripts": [
            "agribot_vehicle_config = agribot_vehicle_config_tools.cli:main",
        ],
    },
)
