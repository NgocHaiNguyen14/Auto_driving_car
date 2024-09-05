import cv2
import time
from ultralytics import YOLO

# Paths to the model and video file
#model_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8//models/yolov8l_seg/weights/best.pt'
#model_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8//models/yolov8m_seg/weights/best.pt'
model_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8s_seg/weights/best.pt'
video_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8//VID_370.MOV'

# Load the YOLOv8 segmentation model
model = YOLO(model_path)

# Open the video file
cap = cv2.VideoCapture(video_path)

# Check if video opened successfully
if not cap.isOpened():
    print("Error: Could not open video.")
    exit()

# Initialize variables for FPS calculation
frame_count = 0
start_time = time.time()
total_inference_time = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    frame_count += 1

    # Convert frame to RGB for YOLOv8
    rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    
    # Perform segmentation inference and measure time
    inference_start_time = time.time()
    results = model(rgb_frame)
    inference_end_time = time.time()
    
    # Calculate inference time for this frame
    inference_time = inference_end_time - inference_start_time
    total_inference_time += inference_time
    
    # Iterate over the results
    annotated_frame = rgb_frame
    for result in results:
        # Render results on the frame
        annotated_frame = result.plot()  # Plot on the frame

    # Display the frame
    #cv2.imshow('Segmented Video', annotated_frame)
    
    # Calculate and display FPS and inference time
    end_time = time.time()
    elapsed_time = end_time - start_time
    fps = frame_count / elapsed_time
    average_inference_time = total_inference_time / frame_count if frame_count > 0 else 0
    print(f'FPS: {fps:.2f}, Average Inference Time per Frame: {average_inference_time:.4f} seconds')

    # Exit on 'q' key press
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release video capture and close windows
#cap.release()
#cv2.destroyAllWindows()
