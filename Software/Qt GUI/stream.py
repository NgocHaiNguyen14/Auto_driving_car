import cv2
import cvzone
from PySide6.QtWidgets import QWidget, QLabel,  QVBoxLayout
from PySide6.QtGui import QPixmap
from PySide6.QtCore import QTimer
import qimage2ndarray
from ultralytics import YOLO
import math


model = YOLO("D:\Desktop\Qt GUI\yolov8l.pt")

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

class Stream(QWidget):
    def __init__(self):
        super().__init__()

        self.setGeometry(100, 100, 640, 480)

        # OpenCV
        self.cap = cv2.VideoCapture(0)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 320)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 240)

        # Create a QLabel for displaying the video feed
        self.label = QLabel('No Camera Feed')

        # Create a timer to update the video feed
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.displayFrame)
        self.timer.start(60)

        # Create a layout for the widgets
        layout = QVBoxLayout()
        layout.addWidget(self.label)

        self.setLayout(layout)

    def displayFrame(self):
        ret, frame = self.cap.read()
        results = model(frame, stream = True)
        for r in results:
            boxes = r.boxes
            for box in boxes:
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

        if ret:
            frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            image = qimage2ndarray.array2qimage(frame)
            self.label.setPixmap(QPixmap.fromImage(image))

        
    