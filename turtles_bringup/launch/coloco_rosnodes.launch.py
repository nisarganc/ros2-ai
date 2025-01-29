#!~/ros2-ai/objectpushing/bin python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="optimizer",
                executable="ros_nodes",
                name="ros_nodes",
                output="screen",
            )
        ]
    )
