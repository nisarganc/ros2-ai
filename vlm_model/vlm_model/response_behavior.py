# -*- coding: utf-8 -*-

response_list = [
    {
        "name": "publish_cmd_vel",
        "description": "Publish cmd_vel message to control the movement of a turtlebots with marker-ids, including translation and rotation velocities",
        "parameters": {
            "type": "object",
            "properties": {
                "marker_id1": {
                    "type": "number",
                    "description": "The Aruco id on top of one of the robots",
                },
                "linear_x1": {
                    "type": "number",
                    "description": "The linear velocity of robot with marker_id1 along the x-axis",
                },
                "angular_z1": {
                    "type": "number",
                    "description": "The angular velocity of robot with marker_id1 around the z-axis",
                },
                "marker_id2": {
                    "type": "number",
                    "description": "The Aruco id on top of one of the robots",
                },
                "linear_x2": {
                    "type": "number",
                    "description": "The linear velocity of robot with marker_id2 along the x-axis",
                },
                "angular_z2": {
                    "type": "number",
                    "description": "The angular velocity of robot with marker_id2 around the z-axis",
                },
                "marker_id3": {
                    "type": "number",
                    "description": "The Aruco id on top of one of the robots",
                },
                "linear_x3": {
                    "type": "number",
                    "description": "The linear velocity of robot with marker_id3 along the x-axis",
                },
                "angular_z3": {
                    "type": "number",
                    "description": "The angular velocity of robot with marker_id3 around the z-axis",
                },                                
            },
            "required": [
                "marker_id1",
                "linear_x1",
                "angular_z1",
                "marker_id2",
                "linear_x2",
                "angular_z2",
                "marker_id3",
                "linear_x3",
                "angular_z3"

            ]
                
        }
    }
]


class ResponseBehavior:

    def __init__(self):
        self.response_functions_list = response_list


if __name__ == "__main__":
    pass
