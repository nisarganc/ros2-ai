#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import random

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from irobot_create_msgs.action import Undock
from geometry_msgs.msg import Twist
from rclpy.action import ActionClient

# Global Initialization
from llm_config.user_config import UserConfig
config = UserConfig()


class MotionControl(Node):
    def __init__(self):
        super().__init__("motion_control")

        self.declare_parameter('turtlebot_name', '/turtle2')
        self.turtlebot_name = self.get_parameter('turtlebot_name').get_parameter_value().string_value

        self.undock_action_client = ActionClient(self, Undock, '/'+self.turtlebot_name+'/undock')
        self.undock_callback()

        self.is_turning = False
        self.cmd_publisher = self.create_publisher(Twist, '/'+self.turtlebot_name+'/cmd_vel', 0)
        self.create_timer(1, self.move_callback)

    def move_callback(self):
        msg = Twist()

        if self.is_turning:
            msg.angular.z = random.uniform(-0.5, 0.5)  
            msg.linear.x = 0.0
        else:
            msg.linear.x = 0.1 
            msg.angular.z = 0.0  

        self.is_turning = not self.is_turning
    
        self.cmd_publisher.publish(msg)
            

    def undock_callback(self):

        goal_msg = Undock.Goal()
        self.undock_action_client.wait_for_server()
        goal_future = self.undock_action_client.send_goal_async(goal_msg)

        rclpy.spin_until_future_complete(self, goal_future)

        self.undock_goal_handle = goal_future.result()

        if not self.undock_goal_handle.accepted:
            self.error('Undock goal rejected')
            return

        self.undock_result_future = self.undock_goal_handle.get_result_async()    

def main(args=None):
    rclpy.init(args=args)

    motion_control = MotionControl()
    rclpy.spin(motion_control)

    motion_control.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    