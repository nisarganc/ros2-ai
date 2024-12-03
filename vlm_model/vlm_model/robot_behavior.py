#!/usr/bin/env python3
# -*- coding: utf-8 -*-

robot_functions_list_turtlebot4 = [
    {
        "name": "publish_cmd_vel",
        "description": "Publish cmd_vel message to control the movement of turtlebot4, including translation and rotation velocities",
        "parameters": {
            "type": "object",
            "properties": {
                "linear_x": {
                    "type": "number",
                    "description": "The linear velocity along the x-axis",
                },
                "linear_y": {
                    "type": "number",
                    "description": "The linear velocity along the y-axis",
                },
                "linear_z": {
                    "type": "number",
                    "description": "The linear velocity along the z-axis",
                },
                "angular_x": {
                    "type": "number",
                    "description": "The angular velocity around the x-axis",
                },
                "angular_y": {
                    "type": "number",
                    "description": "The angular velocity around the y-axis",
                },
                "angular_z": {
                    "type": "number",
                    "description": "The angular velocity around the z-axis",
                },
            },
            "required": [
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ],
        },
    },
    {
        "name": "publish_target_pose",
        "description": "Publish target pose message to control the movement of arm robot, including x, y, z, roll, pitch, yaw",
        "parameters": {
            "type": "object",
            "properties": {
                "x": {
                    "type": "number",
                    "description": "The x position of the target pose",
                },
                "y": {
                    "type": "number",
                    "description": "The y position of the target pose",
                },
                "z": {
                    "type": "number",
                    "description": "The z position of the target pose",
                },
                "roll": {
                    "type": "number",
                    "description": "The roll of the target pose",
                },
                "pitch": {
                    "type": "number",
                    "description": "The pitch of the target pose",
                },
                "yaw": {
                    "type": "number",
                    "description": "The yaw of the target pose",
                },
            },
            "required": [
                "x",
                "y",
                "z",
                "roll",
                "pitch",
                "yaw",
            ],
        },
    }
]

robot_functions_list_multi_robot = [
    {
        "name": "publish_cmd_vel",
        "description": "Publish cmd_vel message to control the movement and rotation of turtlesim. This function is only compatible with turtlesim and not for robotic arm.",
        "parameters": {
            "type": "object",
            "properties": {
                "robot_name": {
                    "type": "string",
                    "description": "Name of the robot instance that should be controlled. Valid robot names are 'turtle1','turtle2','minipupper', when no specific robot name is specified, robot_name=''",
                },
                "duration": {
                    "type": "number",
                    "description": "Duration of time (in seconds) for which the movement should be performed.",
                },
                "linear_x": {
                    "type": "number",
                    "description": "Linear velocity along the x-axis for the robot.",
                },
                "linear_y": {
                    "type": "number",
                    "description": "Linear velocity along the y-axis for the robot.",
                },
                "linear_z": {
                    "type": "number",
                    "description": "Linear velocity along the z-axis for the robot.",
                },
                "angular_x": {
                    "type": "number",
                    "description": "Angular velocity around the x-axis for the robot.",
                },
                "angular_y": {
                    "type": "number",
                    "description": "Angular velocity around the y-axis for the robot.",
                },
                "angular_z": {
                    "type": "number",
                    "description": "Angular velocity around the z-axis for the robot.",
                },
            },
            "required": [
                "robot_name",
                "duration",
                "linear_x",
                "linear_y",
                "linear_z",
                "angular_x",
                "angular_y",
                "angular_z",
            ],
        },
    },
]


class RobotBehavior:

    def __init__(self):
        self.robot_functions_list = robot_functions_list_turtlebot4


if __name__ == "__main__":
    pass
