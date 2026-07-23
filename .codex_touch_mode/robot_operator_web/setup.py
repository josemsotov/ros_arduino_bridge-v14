from glob import glob
import os

from setuptools import find_packages, setup

package_name = "robot_operator_web"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        (os.path.join("share", package_name, "static"), glob("static/*")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="josemsotov",
    description="Web operator interface for the Smart Trolley ROS 2 stack.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "web_server = robot_operator_web.web_server:main",
        ],
    },
)
