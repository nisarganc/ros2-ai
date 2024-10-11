#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# Global Initialization
from llm_config.user_config import UserConfig

config = UserConfig()


class TextInput(Node):
    def __init__(self):
        super().__init__("llm_text_input")

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