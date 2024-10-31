
import os
from glob import glob
from setuptools import setup

package_name = 'turtles_bringup'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            ["resource/" + package_name],
        ),
        ("share/" + package_name, ["package.xml"]),
        (
            os.path.join("share", package_name, "launch"),
            glob(os.path.join("launch", "*launch.[pxy][yma]*")),
        ),
    ],
    install_requires=['setuptools', 'llm_config'],
    zip_safe=True,
    maintainer='Nisarga',
    maintainer_email='nisarga.nilavadi@utn.de',
    description='ros2-ai bringup package for turtles',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
