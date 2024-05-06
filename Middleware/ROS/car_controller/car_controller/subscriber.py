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

class LiDAR_subscriber(Node):
    def __init__(self):
        super().__init__("lidar_subscriber")
        self.subscription = self.create_subscription(
            String,
            '/scan',
            self.lidar_callback,
            10)
        self.file_path = "lidar_data.txt"  # Path to the text file

    def lidar_callback(self, msg: String):
        self.get_logger().info(str(msg))
        self.write_to_file(str(msg))

    def write_to_file(self, data):
        with open(self.file_path, "a") as file:
            file.write(data + "\n")

def main(args=None):
    rclpy.init(args=args)
    cam_node = Camera_subscriber()
    lidar_node = LiDAR_subscriber()
    rclpy.spin(cam_node)
    rclpy.spin(lidar_node)
    rclpy.shutdown()
