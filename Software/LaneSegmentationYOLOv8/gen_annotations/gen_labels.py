import cv2
import os
from ultralytics import YOLO

# Load the trained YOLOv8 model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8l_seg/weights/best.pt')

# Load the image
image_path = 'gen_annotations/01545.png'
image = cv2.imread(image_path)

# Get image dimensions
image_height, image_width = image.shape[:2]

# Create output directory for YOLO-format files
output_dir = "/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/gen_annotations/yolo_output"
os.makedirs(output_dir, exist_ok=True)

# Run inference on the image
results = model(image)


# Check if results is a list and get the first result
if isinstance(results, list):
    results = results[0]  # Access the first result

# Ensure the 'masks' attribute exists
if hasattr(results, 'masks') and results.masks is not None:
    # Get the predicted masks and corresponding classes
    masks = results.masks
    classes = results.boxes.cls.cpu().numpy()  # Predicted class indices

    # Open a new file to write predictions in YOLO format (per image)
    output_file_path = os.path.join(output_dir, f"{os.path.basename(image_path).split('.')[0]}.txt")
    with open(output_file_path, 'w') as f:
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
                        
                        # Normalize the coordinates (0-1) based on image dimensions
                        norm_x = x / 640 #adjust hereeeeeeee
                        norm_y = y / 384
                        
                        polygon_points.extend([norm_x, norm_y])  # Flatten the list

                    # Write the class label and polygon points in YOLO format
                    f.write(f"{cls} " + " ".join([f"{pt:.6f}" for pt in polygon_points]) + "\n")

    print(f"Predicted polygons saved in YOLO format at {output_file_path}")
else:
    print("No masks were detected in the result.")
