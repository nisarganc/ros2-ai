#!~/ros2-ai/objectpushing/bin python3
# -*- coding: utf-8 -*-

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="pose_estimation",
                executable="aruco_pose_node",
                name="aruco_pose_node",
                output="screen",
            ),
            Node(
                package="mrcap",
                executable="mrcap_node",
                name="mrcap_node",
                output="screen",
            ),
            Node(
                package="vlm_model",
                executable="GPT_node",
                name="GPT_node",
                output="screen",
            )
            # Node(
            #     package="input",
            #     executable="llm_text_input_local",
            #     name="llm_text_input_local",
            #     output="screen",
            # ),
            # Node(
            #     package="vlm_model",
            #     executable="gpt",
            #     name="gpt",
            #     output="screen",
            # )
        ]
    )
