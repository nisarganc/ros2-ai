#!/usr/bin/env python3
# -*- coding: utf-8 -*-

response_list = [
    {
        "name": "publish_cmd_vel",
        "description": "Publish cmd_vel message to control the movement of a turtlebot4 with marker-id, including translation and rotation velocities",
        "parameters": {
            "type": "object",
            "properties": {
                "marker_id": {
                    "type": "number",
                    "description": "The Aruco id on top of the robot",
                },
                "linear_x": {
                    "type": "number",
                    "description": "The linear velocity along the x-axis",
                },
                "angular_z": {
                    "type": "number",
                    "description": "The angular velocity around the z-axis",
                },
            },
            "required": [
                "robot_id",
                "linear_x",
                "angular_z",
            ],
        },
    },
]


class ResponseBehavior:

    def __init__(self):
        self.response_functions_list = response_list


if __name__ == "__main__":
    pass
