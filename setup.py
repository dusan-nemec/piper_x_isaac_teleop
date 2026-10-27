from glob import glob
from setuptools import setup

package_name = "piper_x_isaac_teleop"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml", "README.md"]),
        (f"share/{package_name}/launch", glob("launch/*.py")),
        (f"share/{package_name}/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Dusan",
    description="Isaac Sim bring-up and joystick Cartesian teleop for PiPER-X",
    license="BSD-3-Clause",
    entry_points={
        "console_scripts": [
            "isaac_bridge = piper_x_isaac_teleop.isaac_bridge:main",
            "joy_teleop = piper_x_isaac_teleop.joy_teleop:main",
        ],
    },
)
