#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="input",
                executable="oak_input",
                name="oak_input",
                output="screen",
                parameters=[{'turtlebot_name': 'turtle2'}]
            ),
            Node(
                package="input",
                executable="oak_input",
                name="oak_input",
                output="screen",
                parameters=[{'turtlebot_name': 'turtle4'}]
            ),
            Node(
                package="input",
                executable="oak_input",
                name="oak_input",
                output="screen",
                parameters=[{'turtlebot_name': 'turtle6'}]
            )      
        ]
    )
