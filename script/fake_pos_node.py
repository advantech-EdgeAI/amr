#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
# from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import cv2
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
import time


class SimulationPoseNode(Node):

    def __init__(self):

        super().__init__('simulation_pose_node')
        self.pose_publisher = self.create_publisher(PoseWithCovarianceStamped, 'amcl_pose', 1)
        timer_period = 1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.counter = 0
        self.get_logger().info('Simulation Pose Node is ready.')
        
    def timer_callback(self):

        # Create an instance of PoseWithCovarianceStamped
        pose_msg = PoseWithCovarianceStamped()

        # Fill the header
        pose_msg.header.stamp = self.get_clock().now().to_msg()  # Use your node's clock
        pose_msg.header.frame_id = "map"  # Frame of reference

        pose_msg.pose.pose.position.x = float(self.counter)
        pose_msg.pose.pose.position.y = float(self.counter)
        pose_msg.pose.pose.position.z = 0.0

        pose_msg.pose.pose.orientation.x = 0.0
        pose_msg.pose.pose.orientation.y = 0.0
        pose_msg.pose.pose.orientation.z = 0.707
        pose_msg.pose.pose.orientation.w = 0.707  # Quaternion (identity rotation)

        self.pose_publisher.publish(pose_msg)
        self.counter += 1

def main(args=None):

    rclpy.init(args=args)
    simulation_pose_node = SimulationPoseNode()
    rclpy.spin(simulation_pose_node)
    simulation_pose_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()