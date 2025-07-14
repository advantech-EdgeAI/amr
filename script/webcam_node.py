#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np


class WebcamNode(Node):

    def __init__(self):

        super().__init__('webcam_node')
        self.publisher_ = self.create_publisher(Image, "/webcam_image", 1)
        # video file
        # self.video_path = "./IMG_3688.mp4"
        # camera device
        self.video_path = 0
        self.video_cap = cv2.VideoCapture(self.video_path)

        #timer_period = 1/self.video_cap.get(cv2.CAP_PROP_FPS)  # seconds
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.bridge = CvBridge()
        self.count = 0
        
        ret, cv2_im = self.video_cap.read()
        if ret:
            self.get_logger().info('Webcam Node is ready.')

       
    def timer_callback(self):
        
        ret, cv2_im = self.video_cap.read()
        if ret:
            self.publisher_.publish(self.bridge.cv2_to_imgmsg(np.array(cv2_im), "bgr8"))
            # self.get_logger().info('Publishing an image')
            
def main(args=None):

    rclpy.init(args=args)
    webcam_node = WebcamNode()
    rclpy.spin(webcam_node)
    webcam_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()