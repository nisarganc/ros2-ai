#!/usr/bin/env python3
# -*- coding: utf-8 -*-
# flake8: noqa
#
# Copyright 2023 Herman Ye @Auromix
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Description:
#
# Node test Method:
# ros2 run llm_input llm_audio_input_local
# ros2 topic echo /llm_input_audio_to_text
# ros2 topic pub /llm_state std_msgs/msg/String "data: 'listening'" -1
#
# Author: Herman Ye @Auromix

import random


# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

from geometry_msgs.msg import Twist
from rclpy.action import ActionClient

from irobot_create_msgs.action import Undock


# OpenCV related
from cv_bridge import CvBridge
import cv2

# Global Initialization
from llm_config.user_config import UserConfig
config = UserConfig()



class OakInput(Node):
    def __init__(self):
        super().__init__("oak_input_0")

        self.turtlebot_name = 'turtle2'

        self.undock_action_client = ActionClient(self, Undock, '/'+self.turtlebot_name+'_do_not_use/undock')
        self.undock_callback()

        self.is_turning = False
        self.cmd_publisher = self.create_publisher(Twist, '/'+self.turtlebot_name+'_do_not_use/cmd_vel', 0)
        self.create_timer(1, self.move_callback)

        # self.oak_message_subscriber = self.create_subscription(
        #     Image, '/'+self.turtlebot_name+'/oakd/rgb/preview/image_raw', self.image_callback, 0
        # )
        # self.oak_message_subscriber_id = self.create_subscription( 
        #     CameraInfo, '/'+self.turtlebot_name+'/oakd/rgb/preview/camera_info', self.image_info, 0)
        
        # self.bridge = CvBridge()


    def image_callback(self, msg):        
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # save the image 
            cv2.imwrite('oak_image_'+self.turtlebot_name+'.jpg', cv_image)
        
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')

    def image_info(self, msg):
        self.get_logger().info(f' {self.turtlebot_name} camera Info: {msg.header.stamp}')


    def publish_string(self, string_to_send, publisher_to_use):

        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )

    def move_callback(self):
        msg = Twist()

        if self.is_turning:
            msg.angular.z = random.uniform(-1, 1)  
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

    oak_input = OakInput()
    rclpy.spin(oak_input)

    oak_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
    