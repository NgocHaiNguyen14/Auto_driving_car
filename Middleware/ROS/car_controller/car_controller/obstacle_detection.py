from ultralytics import YOLO
import rclpy
from rclpy.node import Node
import cv2
import cvzone
import math

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference



classNames = ["person", "bicycle", "car", "motorbike", "aeroplane", "bus", "train", "truck", "boat",
              "traffic light", "fire hydrant", "stop sign", "parking meter", "bench", "bird", "cat",
              "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra", "giraffe", "backpack", "umbrella",
              "handbag", "tie", "suitcase", "frisbee", "skis", "snowboard", "sports ball", "kite", "baseball bat",
              "baseball glove", "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
              "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange", "broccoli",
              "carrot", "hot dog", "pizza", "donut", "cake", "chair", "sofa", "pottedplant", "bed",
              "diningtable", "toilet", "tvmonitor", "laptop", "mouse", "remote", "keyboard", "cell phone",
              "microwave", "oven", "toaster", "sink", "refrigerator", "book", "clock", "vase", "scissors",
              "teddy bear", "hair drier", "toothbrush"
              ]

class Camera_publisher(Node):

    def __init__(self):
        super().__init__('camera_publisher')

        self.cap = cv2.VideoCapture(0)  # 0 for the default camera, you may need to adjust this if you have multiple cameras
        if not self.cap.isOpened():
            print("Error: Unable to open camera")
            return

        self.model = YOLO('src/car_controller/yolov8n.pt')

        self.yolov8_inference = Yolov8Inference()

        self.timer_period = 0.1
        self.create_timer(self.timer_period, self.camera_callback)

        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 10)

    def camera_callback(self):

        ret, frame = self.cap.read()
        if not ret:
            print("Error: Unable to capture frame")
            return

        results = self.model(frame, stream=True)

        for r in results:
            boxes = r.boxes
            for box in boxes:
                self.inference_result = InferenceResult()
                # Bounding Box
                x1, y1, x2, y2 = box.xyxy[0]
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                # cv2.rectangle(img,(x1,y1),(x2,y2),(255,0,255),3)
                w, h = x2 - x1, y2 - y1
                cvzone.cornerRect(frame, (x1, y1, w, h))
                # Confidence
                conf = math.ceil((box.conf[0] * 100)) / 100
                # Class Name
                cls = int(box.cls[0])

                cvzone.putTextRect(frame, f'{classNames[cls]} {conf}', (max(0, x1), max(35, y1)), scale=1, thickness=1)
                self.inference_result.class_name = self.model.names[cls]
                self.inference_result.top = x1
                self.inference_result.left = y1
                self.inference_result.bottom = x2
                self.inference_result.right = y2
                self.yolov8_inference.yolov8_inference.append(self.inference_result)


            #camera_publisher.get_logger().info(f"{self.yolov8_inference}")
            # Display the frame in a window
            cv2.imshow("Camera Feed", frame)
            cv2.waitKey(1)  # Display each frame for 1 millisecond
        
        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

def main(args=None):
    rclpy.init(args=args)
    camera_publisher = Camera_publisher()
    rclpy.spin(camera_publisher)
    rclpy.shutdown()
