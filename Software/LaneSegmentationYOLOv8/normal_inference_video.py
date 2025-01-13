import cv2
import numpy as np
from ultralytics import YOLO

# Load the YOLOv8 segmentation model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8m_seg/weights/best.pt')

# Initialize video capture from the file
video_path = '/home/sefas/Desktop/Data_Dec18_2024/test.mp4'
cap = cv2.VideoCapture(video_path)

# Check if video file opened successfully
if not cap.isOpened():
    print(f"Error: Could not open video file {video_path}.")
    exit()

# Define class names (adjust based on your model's class list)
class_names = ['Lane', 'Marking', 'Vehicle', 'Obstacle']  # Replace with your own classes if needed
colors = [(255, 0, 0), (0, 255, 0), (0, 0, 255), (255, 255, 0)]  # Colors for each class

while True:
    # Read a frame from the video
    ret, frame = cap.read()
    if not ret:
        print("End of video or error reading frame.")
        break

    # Perform inference on the frame
    results = model(frame)

    # Handle segmentation masks if they exist
    masks = getattr(results[0].masks, 'data', None)  # Safely access the masks
    if masks is not None:
        masks = masks.cpu().numpy()  # Convert to NumPy array
        mask_img = np.zeros_like(frame, dtype=np.uint8)  # Create a blank mask image
        for i, mask in enumerate(masks):
            class_id = int(results[0].boxes.cls[i].cpu().numpy())
            color = colors[class_id] if class_id < len(colors) else (255, 255, 255)  # Default to white if class_id is out of bounds
            mask = mask.astype(bool)  # Convert to boolean mask
            mask_img[mask] = color  # Apply color to the mask

        # Blend the mask image with the original frame
        alpha = 0.5  # Transparency for the masks
        frame = cv2.addWeighted(frame, 1 - alpha, mask_img, alpha, 0)

    # Draw bounding boxes and labels on the frame if any detections exist
    boxes = results[0].boxes
    if boxes is not None:
        for i, box in enumerate(boxes.xywh.cpu().numpy()):
            x_center, y_center, width, height = box
            x1, y1 = int((x_center - width / 2)), int((y_center - height / 2))
            x2, y2 = int((x_center + width / 2)), int((y_center + height / 2))

            # Get the class name and confidence score
            class_id = int(boxes.cls[i].cpu().numpy())
            confidence = boxes.conf[i].cpu().numpy()
            label = f'{class_names[class_id]} {confidence:.2f}'

            # Draw the bounding box and label
            color = (0, 255, 0)  # Green for bounding box, customize as needed
            cv2.rectangle(frame, (x1, y1), (x2, y2), color, 2)
            cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

    # Show the frame with detections and segmentation masks
    cv2.imshow('Segmentation and Detection', frame)

    # Break the loop if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the capture and close any OpenCV windows
cap.release()
cv2.destroyAllWindows()

