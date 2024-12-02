from setuptools import setup

package_name = 'llm_robot'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'llm_config'],
    zip_safe=True,
    maintainer='Nisarga Nilavadi',
    maintainer_email='nisarga.nilavadi@utn.de',
    description='The llm_robot package publishes action commands for real world robot',
    license="TODO",
    entry_points={
        'console_scripts': [
            "turtle7 = llm_robot.turtle7:main",
            "arm_robot = llm_robot.arx5_arm_robot:main",
            "multi_robot = llm_robot.multi_robot:main",
        ],
    },
)
