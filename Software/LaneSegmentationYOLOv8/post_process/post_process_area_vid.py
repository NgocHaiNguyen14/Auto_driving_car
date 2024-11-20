import cv2
import numpy as np
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8m_seg/weights/best.pt')

# Define the polygon points for drawing on the image
polygon1_points = np.array([[220, 320], [364, 320], [338, 285], [246, 285]], dtype=np.int32)  # Red
polygon2_points = np.array([[338, 285], [246, 285], [272, 250], [312, 250]], dtype=np.int32)  # Green
polygon3_points = np.array([[313, 250], [332, 250], [384, 320], [365, 320]], dtype=np.int32)  # Yellow
polygon4_points = np.array([[271, 250], [252, 250], [200, 320], [221, 320]], dtype=np.int32)  # Yellow

# Load the video
cap = cv2.VideoCapture('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/VID_370.MOV')

# Get video properties for setting up the VideoWriter
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = int(cap.get(cv2.CAP_PROP_FPS))

# Resize dimensions
output_width, output_height = 640, 384

# Video writers for lane and marking masks
lane_writer = cv2.VideoWriter('lane_masks_video.avi', cv2.VideoWriter_fourcc(*'XVID'), fps, (output_width, output_height), isColor=False)
vehicles_writer = cv2.VideoWriter('vehicles_masks_video.avi', cv2.VideoWriter_fourcc(*'XVID'), fps, (output_width, output_height), isColor=False)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Resize the frame to 640x384 for consistent processing
    frame_resized = cv2.resize(frame, (output_width, output_height))

    # Perform inference on the frame
    results = model(frame_resized)

    # Extract the segmentation masks
    masks = results[0].masks.data.cpu().numpy()

    # Extract class predictions from the bounding boxes
    class_preds = results[0].boxes.cls.cpu().numpy()

    # Initialize masks for lane and marking (class 2 and class 1)
    lane_mask = np.zeros((output_height, output_width), dtype=np.uint8)
    vehicles_mask = np.zeros((output_height, output_width), dtype=np.uint8)

    # Iterate through masks and add pixels of the lane class (class 2) to the lane_mask
    for i, cls in enumerate(class_preds):
        if cls == 2:  # Class 2 is the lane
            lane_mask = np.maximum(lane_mask, masks[i])
        elif cls == 0:  # Class 1 is the marking
            vehicles_mask = np.maximum(vehicles_mask, masks[i])

    # Convert masks to 8-bit format for video saving
    lane_mask = (lane_mask * 255).astype(np.uint8)
    vehicles_mask = (vehicles_mask * 255).astype(np.uint8)

    # Write the masks to the video files
    lane_writer.write(lane_mask)
    vehicles_writer.write(vehicles_mask)

    # Optionally, display the resized frame with the polygons (comment out if not needed)
    # cv2.imshow('Frame with Polygons', frame_resized)

    # Press 'q' to exit the loop early
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture and writers
cap.release()
lane_writer.release()
vehicles_writer.release()
cv2.destroyAllWindows()

