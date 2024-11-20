import cv2
import numpy as np
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8m_seg/weights/best.pt')

# Initialize webcam capture (camera index 0, can change if you have multiple cameras)
cap = cv2.VideoCapture(0)

# Check if camera opened successfully
if not cap.isOpened():
    print("Error: Could not open camera.")
    exit()

while True:
    # Read a frame from the camera
    ret, frame = cap.read()
    if not ret:
        print("Error: Failed to capture image.")
        break
    
    # Perform inference on the frame
    results = model(frame)

    # Extract the bounding boxes and class predictions
    boxes = results[0].boxes
    class_preds = boxes.cls.cpu().numpy()
    confidences = boxes.conf.cpu().numpy()
    xywh = boxes.xywh.cpu().numpy()  # Get the center, width, and height of the bounding boxes

    # Define class names (adjust based on your model's class list)
    class_names = ['Vehicle', 'Marking', 'Lane']  # Replace with your own classes if needed

    # Draw bounding boxes and labels on the frame
    for i, box in enumerate(xywh):
        x_center, y_center, width, height = box
        x1, y1 = int((x_center - width / 2) * frame.shape[1]), int((y_center - height / 2) * frame.shape[0])
        x2, y2 = int((x_center + width / 2) * frame.shape[1]), int((y_center + height / 2) * frame.shape[0])
        
        # Get the class name and confidence score
        class_id = int(class_preds[i])
        label = f'{class_names[class_id]} {confidences[i]:.2f}'

        # Draw the bounding box and label
        color = (0, 255, 0)  # Green for bounding box, you can customize it
        cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
        cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # Show the frame with detections
    cv2.imshow('Object Detection', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()
