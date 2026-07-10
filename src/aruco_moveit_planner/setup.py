import glob

from setuptools import find_packages, setup

PACKAGE = "aruco_moveit_planner"

setup(
    name=PACKAGE,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + PACKAGE],
        ),
        ("share/" + PACKAGE, ["package.xml"]),
        ("share/" + PACKAGE + "/launch", ["launch/plan_to_aruco.launch.py"]),
        ("share/" + PACKAGE + "/config", ["config/sample_aruco_pose.json"]),
        ("share/" + PACKAGE + "/calibrations", glob.glob("../../calibrations/*.calib")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rosi",
    maintainer_email="start27.11.25@gmail.com",
    description=(
        "Plan-only MoveIt 2 trajectory to an ArUco marker for the left UR10e arm."
    ),
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "plan_to_aruco = aruco_moveit_planner.main:main",
            "arm_joint_state_publisher = aruco_moveit_planner.arm_joint_state_publisher:main",
        ],
    },
)
