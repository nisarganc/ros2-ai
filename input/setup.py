from setuptools import setup

package_name = 'input'

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
    maintainer='Nisarga',
    maintainer_email='nisarga.nilavadi@utn.de',
    description='The input package contains input nodes for the ROS2-AI.',
    license="Apache-2.0",
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "motion_control = input.motion_control:main",
            "oak_input = input.oak_input:main",
            "llm_text_input = input.llm_text_input:main",
        ],
    },
)
