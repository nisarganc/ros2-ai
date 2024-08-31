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

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()


class TextInput(Node):
    def __init__(self):
        super().__init__("llm_text_input_local")

        # LLM initialization publisher
        self.initialization_publisher = self.create_publisher(
            String, "/llm_initialization", 0
        )

        # LLM state listener
        self.llm_message_subscriber = self.create_subscription(
            String, "/llm_input_message", self.state_listener_callback, 0
        )

        # LLM message publisher
        self.llm_message_publisher = self.create_publisher(
            String, "/message_from_user", 0
        )

        # Initialization ready
        self.publish_string("llm input node initialized!", self.initialization_publisher)

    def state_listener_callback(self, msg):
        if msg.data != "":
            self.get_logger().info(f"User Message Received: {msg.data}")
            self.publish_string(msg.data, self.llm_message_publisher)
        else:
            self.get_logger().info("Empty Message Received!")

    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )


def main(args=None):
    rclpy.init(args=args)

    text_input = TextInput()

    rclpy.spin(text_input)

    text_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()