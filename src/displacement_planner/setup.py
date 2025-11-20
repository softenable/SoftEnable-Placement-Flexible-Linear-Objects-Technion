import os
from glob import glob

from setuptools import find_packages, setup

package_name = "displacement_planner"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*launch.[pxy][yma]*")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "config"), glob("config/*.urdf")),
        (os.path.join("share", package_name, "scripts"), glob("scripts/*.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Itay Grinberg",
    maintainer_email="itaygrinberg@campus.technion.ac.il",
    description="ROS 2 service for planning flexible object displacement trajectories",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "planner = displacement_planner.planner:main",
            "example_client = displacement_planner.example_client:main",
            "displacement_calculator = displacement_planner.displacement.main:main",
        ],
    },
)
