from glob import glob
from setuptools import setup

setup(
    name="agribot_k2_sim", version="0.1.0", packages=["agribot_k2_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agribot_k2_sim"]),
        ("share/agribot_k2_sim", ["package.xml", "README.md"]),
        ("share/agribot_k2_sim/launch", glob("launch/*.launch.py")),
        ("share/agribot_k2_sim/plugins", glob("plugins/*.cpp")),
    ],
    entry_points={"console_scripts": [
        "prepare_k2_scene=agribot_k2_sim.prepare:main",
        "verify_running_k2=agribot_k2_sim.validate:main",
        "check_k2_sensors=agribot_k2_sim.sensor_health:main",
    ]},
)
