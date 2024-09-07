import cv2
import os
from ultralytics import YOLO

# Load the trained YOLOv8 model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8l_seg/weights/best.pt')

# Path to input video
video_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/VID_370.MOV'

# Open the video file
cap = cv2.VideoCapture(video_path)

# Get video properties
frame_width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
frame_height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
fps = cap.get(cv2.CAP_PROP_FPS)

# Create output directories for YOLO-format files and images
label_output_dir = "/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/gen_annotations/yolo_output"
image_output_dir = "/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/gen_annotations/yolo_output_images"
os.makedirs(label_output_dir, exist_ok=True)
os.makedirs(image_output_dir, exist_ok=True)

frame_count = 0

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference on the frame
    results = model(frame)

    # Save the frame as an image
    image_file_path = os.path.join(image_output_dir, f"frame_{frame_count:06d}.png")
    cv2.imwrite(image_file_path, frame)  # Save the current frame

    # Check if results is a list and get the first result
    if isinstance(results, list):
        results = results[0]  # Access the first result

    # Ensure the 'masks' attribute exists
    if hasattr(results, 'masks') and results.masks is not None:
        # Get the predicted masks and corresponding classes
        masks = results.masks
        classes = results.boxes.cls.cpu().numpy()  # Predicted class indices

        # Open a new file to write predictions in YOLO format (per frame)
        label_file_path = os.path.join(label_output_dir, f"frame_{frame_count:06d}.txt")
        with open(label_file_path, 'w') as f:
            for i, mask in enumerate(masks.data.cpu().numpy()):
                cls = int(classes[i])  # Get the class index
                
                # Extract the contours (polygon points) from the mask
                contours, _ = cv2.findContours(mask.astype('uint8'), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                
                # Go through each contour (there could be multiple in one mask)
                for contour in contours:
                    if len(contour) >= 3:  # Ensure there are at least 3 points to form a polygon
                        polygon_points = []
                        for point in contour:
                            x, y = point[0]
                            
                            # Normalize the coordinates (0-1) based on video frame dimensions
                            norm_x = x / frame_width
                            norm_y = y / frame_height
                            
                            polygon_points.extend([norm_x, norm_y])  # Flatten the list

                        # Write the class label and polygon points in YOLO format
                        f.write(f"{cls} " + " ".join([f"{pt:.6f}" for pt in polygon_points]) + "\n")

        print(f"Predicted polygons and corresponding image saved for frame {frame_count}.")
    else:
        print(f"No masks were detected in the result for frame {frame_count}.")

    frame_count += 1

# Release the video capture
cap.release()

print("Video processing and saving completed.")
