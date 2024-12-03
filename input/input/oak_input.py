#!~/ros2-ai/objectpushing/bin python3
# -*- coding: utf-8 -*-

import random

# ROS related
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo

# OpenCV related
from cv_bridge import CvBridge
import cv2




class OakInput(Node):
    def __init__(self):
        super().__init__("oak_input")

        self.declare_parameter('turtlebot_name', '/turtle2')
        self.turtlebot_name = self.get_parameter('turtlebot_name').get_parameter_value().string_value

        self.oak_message_subscriber = self.create_subscription(
            Image, '/'+self.turtlebot_name+'/oakd/rgb/preview/image_raw', self.image_callback, 0
        )
        self.oak_message_subscriber_id = self.create_subscription( 
            CameraInfo, '/'+self.turtlebot_name+'/oakd/rgb/preview/camera_info', self.image_info, 0)
        
        self.bridge = CvBridge()


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

def main(args=None):
    rclpy.init(args=args)

    oak_input = OakInput()
    rclpy.spin(oak_input)

    oak_input.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()