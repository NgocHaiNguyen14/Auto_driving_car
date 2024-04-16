#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__("camera_subscriber")
        self.subscription = self.create_subscription(
            Yolov8Inference,
            '/Yolov8_Inference',
            self.yolo_callback,
            10)

    def yolo_callback(self, msg: Yolov8Inference):
        self.get_logger().info(str(msg))

def main(args=None):
    rclpy.init(args=args)
    node = Camera_subscriber()
    rclpy.spin(node)
    rclpy.shutdown()