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
# This code defines a ROS node called ChatGPTNode
# The node interacts with the ChatGPT service to implement conversational interactions
# The node implements the ChatGPT service callback function "llm_callback"
# The node also includes a client function "function_call_client" and a publisher "output_publisher"
# It also includes a function called "add_message_to_history" to update chat history records
# The code generates a chat response using the OpenAI API
# It extracts response information from the response data
# The code writes chat history records to a JSON file using Python's JSON library
# The code calls other functions using ROS Service
#
# Node test Method:
# ros2 run llm_model chatgpt
# ros2 topic echo /llm_feedback_to_user
# ros2 topic pub /llm_input_audio_to_text std_msgs/msg/String "data: 'Hello,tell me a joke'" -1
#
# Author: Herman Ye @Auromix

# ROS related
import rclpy
from rclpy.node import Node
from llm_interfaces.srv import ChatGPT
from std_msgs.msg import String

# LLM related
import json
import os
import time
from openai import OpenAI
from llm_config.user_config import UserConfig


# Global Initialization
config = UserConfig()
client = OpenAI(
    api_key=config.openai_api_key
    
)

class ChatGPTNode(Node):
    def __init__(self):
        super().__init__("ChatGPT_node")

        # LLM input listener
        self.llm_input_subscriber = self.create_subscription(
            String, "/message_from_user", self.llm_callback, 0
        )

        # LLM feedback for user publisher
        self.llm_feedback_publisher = self.create_publisher(
            String, "/llm_feedback_to_user", 0
        )

        # client of ChatGPT service
        self.function_call_client = self.create_client(
            ChatGPT, "/ChatGPT_function_call_service"
        )

        # Wait for function call server to be ready
        while not self.function_call_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(
                "ChatGPT Function Call Server from turtle7 not available, waiting again..."
            )

        # request to be sent to ChatGPT service
        self.function_call_request = ChatGPT.Request()

        self.start_timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.chat_history_file = os.path.join(
            config.chat_history_path, f"chat_history_{self.start_timestamp}.json"
        )
        self.write_chat_history_to_json()
        self.get_logger().info(f"Chat history saved to {self.chat_history_file}")

        # Function name
        self.function_name = "null"


    ##################################### CHATGPT RESPONSE ############################################
    def llm_callback(self, msg):
        """
        llm_callback is the main function of the ChatGPT node.
        """

        user_prompt = msg.data
        self.add_message_to_history("user", user_prompt)
        chatgpt_response = self.generate_chatgpt_response(config.chat_history)
        content_text, function_flag, function_call_arg, function_call_name = self.get_response_information(
            chatgpt_response
        )

        if function_flag == 1:
            self.function_call(function_call_arg, function_call_name)
        else:
            self.publish_string(content_text, self.llm_feedback_publisher)
            self.add_message_to_history(
            role="assistant", content=content_text.strip("\n").strip()
            )
        self.write_chat_history_to_json()

    def generate_chatgpt_response(self, chat_history):
        """
        Generates a chatgpt response based on the input messages provided.
        """

        response = client.chat.completions.create(
            model=config.openai_model,
            messages=chat_history,
            functions=config.robot_functions_list,
            function_call="auto",
            temperature=config.openai_temperature,
            top_p=config.openai_top_p,
            n=config.openai_n,
            stream=config.openai_stream,
            stop=config.openai_stop,
            max_tokens=config.openai_max_tokens,
            presence_penalty=config.openai_presence_penalty,
            frequency_penalty=config.openai_frequency_penalty,
        )
        self.get_logger().info(f"OpenAI response: {response}")
        return response


    def get_response_information(self, chatgpt_response):
        """
        Returns the response information from the chatgpt response.
        The response information includes the message, text, function call, and function flag.
        """

        message = chatgpt_response.choices[0].message
        content = message.content

        # Initializing function flag, 0: no function call and text context, 1: function call and None content
        function_flag = 0

        if content is not None:
            function_flag = 0
            function_call_name = None
            function_call_arg = None
        else:
            function_flag = 1
            function_call_name = message.function_call.name
            function_call_arg = message.function_call.arguments

        # self.get_logger().info(
        #     f"Content from OpenAI: {content}, type: {type(content)}"
        # )
        # self.get_logger().info( 
        #     f"Function call name from OpenAI: {function_call_name}, type: {type(function_call_name)}"
        # )
        # self.get_logger().info(
        #     f"Function call from OpenAI: {function_call_arg}, type: {type(function_call_arg)}"
        #)

        return content, function_flag, function_call_arg, function_call_name


    def function_call(self, function_call_args, function_call_name):
        """
        Sends a function call request with the given input and waits for the response.
        When the response is received, the function call response callback is called.
        """
        function_call_text = function_call_args
        self.function_name = function_call_name

        # Send function call request
        self.function_call_request.request_text = function_call_text
        self.function_call_request.request_name = function_call_name
        future = self.function_call_client.call_async(self.function_call_request)
        future.add_done_callback(self.function_call_response_callback)

    def function_call_response_callback(self, future):
        """
        The function call response callback is called when the function call response is received.
        the function_call_response_callback will call the gpt service again
        to get the text response to user
        """
        try:
            response = future.result()
            self.get_logger().info(
                f"Response from ChatGPT_function_call_service: {response}"
            )

        except Exception as e:
            self.get_logger().info(f"ChatGPT function call service failed {e}")

        response_text = response.response_text
        
        self.add_message_to_history(
            role="function",
            content=response_text.strip("\n").strip(),
            name=self.function_name
        )
        
        # Generate chat completion
        second_chatgpt_response = self.generate_chatgpt_response(config.chat_history)
        content_text, function_flag, function_call_arg, function_call_name = self.get_response_information(
            second_chatgpt_response
        )
        if function_flag != 1:
            self.add_message_to_history(
                role="assistant", content=content_text.strip("\n").strip()
            )
            self.publish_string(content_text, self.llm_feedback_publisher)

    ####################################### HELPER FUNCTIONS ############################################

    def add_message_to_history(
        self, role, content="null", name=None
    ):
        """
        Add a new message_element_object to the chat history
        with the given role, content, and function call information.
        The message_element_object dictionary contains
        the key-value pairs for "role", "content", and "function_call".
        If the chat history exceeds the maximum allowed length,
        the oldest message_element_object will be removed.
        Returns the updated chat history list.
        """

        # Adding message_element_object to chat history
        message_element_object = {
            "role": role,
            "content": content,
        }
        if name is not None:
            message_element_object["name"] = name
        
        config.chat_history.append(message_element_object)

        if len(config.chat_history) > config.chat_history_max_length: 
            self.get_logger().info(
                f"Chat history is too long, popping out the oldest message: {config.chat_history[0]}"
            )
        config.chat_history.pop(0)

        # Returning updated chat history dictionary
        return config.chat_history


    def write_chat_history_to_json(self):
        """
        Write the chat history to a JSON file.
        """
        try:
            # Converting chat history to JSON string
            json_data = json.dumps(config.chat_history)

            # Writing JSON to file
            with open(self.chat_history_file, "w", encoding="utf-8") as file:
                file.write(json_data)

            return True

        except IOError as error:
            self.get_logger().error(f"Error writing chat history to JSON: {error}")
            return False


    def publish_string(self, string_to_send, publisher_to_use):
        msg = String()
        msg.data = string_to_send

        publisher_to_use.publish(msg)
        self.get_logger().info(
            f"Topic: {publisher_to_use.topic_name}\nMessage published: {msg.data}"
        )

def main(args=None):
    rclpy.init(args=args)
    chatgpt = ChatGPTNode()
    rclpy.spin(chatgpt)
    rclpy.shutdown()


if __name__ == "__main__":
    main()