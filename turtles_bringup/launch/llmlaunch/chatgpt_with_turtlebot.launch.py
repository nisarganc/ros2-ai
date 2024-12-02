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
                executable="llm_text_input",
                name="llm_text_input",
                output="screen",
            ),
            Node(
                package="llm_model",
                executable="chatgpt",
                name="chatgpt",
                output="screen",
            ),
            Node(
                package="llm_robot",
                executable="turtle7",
                name="turtle7",
                output="screen",
            )
        ]
    )
