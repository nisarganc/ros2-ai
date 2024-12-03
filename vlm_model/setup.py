from setuptools import setup

package_name = "vlm_model"

setup(
    name=package_name,
    version="0.0.1",
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),        
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools", "vlm_config", "msgs_interfaces"],
    zip_safe=True,
    maintainer="Nisarga Nilavadi",
    maintainer_email="nisarga.nilavadi@utn.de",
    description="The vlm package for ros2-ai",
    license="TODO",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "chatgpt = vlm_model.chatgpt:main",
            "gpt = vlm_model.gpt:main",
            "multi_robot = vlm_model.multi_robot:main",
        ],
    },
)
