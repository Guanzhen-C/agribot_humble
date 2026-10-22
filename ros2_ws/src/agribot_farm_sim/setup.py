from glob import glob
from pathlib import Path
from setuptools import setup

setup(
    name="agribot_farm_sim", version="0.1.0", packages=["agribot_farm_sim"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/agribot_farm_sim"]),
        ("share/agribot_farm_sim", ["package.xml"]),
        ("share/agribot_farm_sim/config", glob("config/*.json") + glob("config/*.txt")),
    ] + [("share/agribot_farm_sim/" + str(directory),
           [str(path) for path in directory.iterdir() if path.is_file()])
          for directory in Path("assets").rglob("*") if directory.is_dir()],
    install_requires=["setuptools"], zip_safe=True,
    maintainer="cgz", maintainer_email="cgz@example.com", license="Apache-2.0",
    description="Agricultural simulation scenarios and local CAD inventory",
    entry_points={"console_scripts": [
        "farm_resources = agribot_farm_sim.cli:main",
        "farm_compatibility = agribot_farm_sim.compatibility:main",
    ]},
)
