from glob import glob
from pathlib import Path
from setuptools import setup

setup(
    name="agribot_k2_sim", version="0.1.0", packages=["agribot_k2_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agribot_k2_sim"]),
        ("share/agribot_k2_sim", ["package.xml", "README.md"]),
        ("share/agribot_k2_sim/launch", glob("launch/*.launch.py")),
        ("share/agribot_k2_sim/plugins", glob("plugins/*.cpp")),
    ] + [("share/agribot_k2_sim/" + str(folder), [str(p) for p in folder.iterdir() if p.is_file()])
         for folder in [Path("vendor"), *Path("vendor").rglob("*")] if folder.is_dir()],
    entry_points={"console_scripts": [
        "prepare_k2_scene=agribot_k2_sim.prepare:main",
        "verify_running_k2=agribot_k2_sim.validate:main",
        "check_k2_sensors=agribot_k2_sim.sensor_health:main",
        "import_scanned_scene=agribot_k2_sim.import_scene:main",
    ]},
)
