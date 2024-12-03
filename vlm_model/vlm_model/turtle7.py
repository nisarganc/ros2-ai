#!~/ros2-ai/objectpushing/bin python3
# -*- coding: utf-8 -*-

# ROS related
import rclpy
from rclpy.node import Node
from msgs_interfaces.srv import ChatGPT
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty

# LLM related
import json
from openai_config import UserConfig

# Global Initialization
config = UserConfig()

class turtle7(Node):
    def __init__(self):
        super().__init__("turtle7")

        # Publisher for cmd_vel
        self.publisher_ = self.create_publisher(Twist, "/turtle7/cmd_vel", 10)
        
        # Server for ChatGPT function call
        self.function_call_server = self.create_service(
            ChatGPT, "/ChatGPT_function_call_service", self.function_call_callback
        )

    def function_call_callback(self, request, response):
        function_name = request.request_name
        function_args = json.loads(request.request_text)
        func_obj = getattr(self, function_name)
        try:
            function_execution_result = func_obj(**function_args)
        except Exception as error:
            self.get_logger().info(f"Failed to call function: {error}")
            response.response_text = str(error)
        else:
            response.response_text = str(function_execution_result)
        return response

    def publish_cmd_vel(self, **kwargs):
        """
        Publishes cmd_vel message to control the movement of turtlebot4
        """
        linear_x = kwargs.get("linear_x", 0.0)
        linear_y = kwargs.get("linear_y", 0.0)
        linear_z = kwargs.get("linear_z", 0.0)
        angular_x = kwargs.get("angular_x", 0.0)
        angular_y = kwargs.get("angular_y", 0.0)
        angular_z = kwargs.get("angular_z", 0.0)
        response = {
            "linear_x": linear_x,
            "linear_y": linear_y,
            "linear_z": linear_z,
            "angular_x": angular_x,
            "angular_y": angular_y,
            "angular_z": angular_z,
        }

        twist_msg = Twist()
        twist_msg.linear.x = float(linear_x)
        twist_msg.linear.y = float(linear_y)
        twist_msg.linear.z = float(linear_z)
        twist_msg.angular.x = float(angular_x)
        twist_msg.angular.y = float(angular_y)
        twist_msg.angular.z = float(angular_z)
        self.publisher_.publish(twist_msg)

        self.get_logger().info(f"Publishing cmd_vel message successfully: {twist_msg}")
        return response

def main():
    rclpy.init()
    turtle_robot = turtle7()
    rclpy.spin(turtle_robot)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
