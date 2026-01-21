from setuptools import setup, find_packages
import os
from glob import glob

package_name = "gps_autonomous_navigation"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "launch"), glob("launch/*.py")),
        (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
        (os.path.join("share", package_name, "urdf"), glob("urdf/*.urdf")),
        (os.path.join("share", package_name, "rviz"), glob("rviz/*.rviz")),
        (os.path.join("share", package_name, "maps"), glob("maps/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="kavya",
    maintainer_email="kavya@todo.todo",
    description="GPS Navigation package with SLAM Toolbox and Nav2",
    license="Apache-2.0",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "gps_goal_sender = gps_autonomous_navigation.gps_goal_sender:main",
            "gps_to_local = gps_autonomous_navigation.gps_to_local:main",
            "goal_pose_publisher = gps_autonomous_navigation.goal_pose_publisher:main",
            "laser_odom = gps_autonomous_navigation.laser_odom:main",
        ],
    },
)

