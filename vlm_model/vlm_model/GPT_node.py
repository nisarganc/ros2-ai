#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS related
import rclpy
from rclpy.node import Node
from msgs_interfaces.srv import GPT

# LLM related
import json
import os
import time
from openai import OpenAI
from .openai_config import OpenAIConfig

import cv2
from cv_bridge import CvBridge
import base64

# Global Initialization
config = OpenAIConfig()
client = OpenAI(
    api_key=config.openai_api_key
    
)

class GPTNode(Node):
    def __init__(self):
        super().__init__("GPT_node")

        # Server for GPT response: (Poses, Image) -> Twist
        self.function_call_server = self.create_service(
            GPT, "GPT_service", self.gpt_callback
        )
        
        self.bridge = CvBridge()

        self.start_timestamp = time.strftime("%Y-%m-%d-%H-%M-%S", time.localtime())
        self.chat_history_file = os.path.join(
            config.chat_history_path, f"multi_robot_{self.start_timestamp}.json"
        )
        self.write_chat_history_to_json()
        self.get_logger().info(f"Chat history saved to {self.chat_history_file}")

        # Function name
        self.function_name = "null"


    ##################################### CHATGPT RESPONSE ############################################
    def gpt_callback(self, request, response):

        self.add_message_to_history("user", content=request.poses_text, image=request.image)     
        gpt_response = self.generate_gpt_response(config.chat_history)
        content_text, function_flag, function_call_name, function_call_arg = self.get_response_information(gpt_response)

        if function_flag == 1:
            response = function_call_arg
        else:
            response = "Function call not found in GPT response"

        self.add_message_to_history(
            role="assistant",
            content=content_text,
            image=None,
            func_name=function_call_name,
            func_arg=function_call_arg,
        )
        self.write_chat_history_to_json()

        self.get_logger().info(f"Response: {response}")

        return response
        

    def generate_gpt_response(self, chat_history):
        response = client.chat.completions.create(
            model=config.openai_model,
            messages=chat_history,
            functions=config.response_functions_list,
            function_call="auto",            
            # temperature=config.openai_temperature,
            # top_p=config.openai_top_p,
            # n=config.openai_n,
            # stream=config.openai_stream,
            # stop=config.openai_stop,
            # max_tokens=config.openai_max_tokens,
            # presence_penalty=config.openai_presence_penalty,
            # frequency_penalty=config.openai_frequency_penalty,
        )
        # self.get_logger().info(f"OpenAI response: {response}")
        return response


    def get_response_information(self, gpt_response):

        message = gpt_response.choices[0].message
        content = message.content.strip("\n").strip()
        self.get_logger().info(f"MESSAGE: {content}")

        # Initializing function flag, 0: no function call and text context, 1: function call and None content
        function_flag = 0

        if message.function_call is None:
            function_flag = 0
            function_call_name = None
            function_call_arg = None
        else:
            function_flag = 1
            function_call_name = message.function_call.name
            function_call_arg = message.function_call.arguments

        return content, function_flag, function_call_name, function_call_arg
        

    ####################################### HELPER FUNCTIONS ############################################

    def add_message_to_history(self, role, content="null", image=None, func_name=None, func_arg=None):

        if role == "user":
            cv_image = self.bridge.imgmsg_to_cv2(image, "bgr8")
            _, buffer = cv2.imencode('.png', cv_image)

            encoded_image = base64.b64encode(buffer.tobytes()).decode('utf-8')
        
            message_element_object = {
                "role": role,
                "content": [
                            {"type": "text", "text": content},
                            {
                                "type": "image_url",
                                "image_url": {"url": f"data:image/png;base64,{encoded_image}"},
                            },
                ],
            }
        else:
            message_element_object = {
                "role": role,
                "content": [
                            {"type": "text", "text": content},
                            {"type": "function", "function_name": func_name, "function_arg": func_arg},
                ],
            }
        
        config.chat_history.append(message_element_object)

        if len(config.chat_history) > config.chat_history_max_length: 
            self.get_logger().info(
                f"Chat history is too long, popping out the oldest message: {config.chat_history[0]}"
            )
        config.chat_history.pop(0)

        return config.chat_history


    def write_chat_history_to_json(self):
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

def main(args=None):
    rclpy.init(args=args)
    chatgpt = GPTNode()
    rclpy.spin(chatgpt)
    rclpy.shutdown()


if __name__ == "__main__":
    main()