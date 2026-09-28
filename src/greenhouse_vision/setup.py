import os
from glob import glob

from setuptools import find_packages, setup

package_name = "greenhouse_vision"

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
    maintainer="delcri",
    maintainer_email="delcast2210@gmail.com",
    description="Color/depth dock-target detection and lateral alignment controller for greenhouse table docking",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "dock_target_detector = greenhouse_vision.dock_target_detector:main",
            "table_alignment_controller = greenhouse_vision.table_alignment_controller:main",
        ],
    },
)
