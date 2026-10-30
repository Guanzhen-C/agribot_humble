from glob import glob
from setuptools import setup

name = "agribot_localization_switch"
setup(name=name, version="0.1.0", packages=[name],
      data_files=[("share/ament_index/resource_index/packages", ["resource/"+name]),
                  ("share/"+name, ["package.xml", "README.md"]),
                  ("share/"+name+"/launch", glob("launch/*.launch.py"))],
      install_requires=["setuptools"], zip_safe=False,
      maintainer="cgz", maintainer_email="cgz@example.com", license="GPL-3.0-only",
      description="Waypoint-controlled localization handover",
      entry_points={"console_scripts": ["localization_switch = agribot_localization_switch.node:main"]})
