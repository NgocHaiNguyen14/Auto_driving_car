import cv2
import numpy as np
import matplotlib.pyplot as plt

# File paths
image_path = 'gen_annotations/02233.png'
label_path = 'gen_annotations/yolo_output/02233.txt'

# Colors for each class: lane, marking, vehicle
colors = {
    0: (255, 0, 0),  # Red for lane
    1: (0, 255, 0),  # Green for marking
    2: (0, 0, 255)   # Blue for vehicle
}

# Read the image
image = cv2.imread(image_path)
if image is None:
    raise FileNotFoundError(f"Image {image_path} not found")

# Get image dimensions
height, width, _ = image.shape

# Read label file and parse objects
with open(label_path, 'r') as file:
    labels = file.readlines()

# Loop over each object in the label file
for label in labels:
    label = label.strip().split()
    class_id = int(label[0])  # Class ID
    
    # Polygon points: pairs of (x, y) after the class ID
    polygon_points = list(map(float, label[1:]))
    
    # Denormalize the polygon points (from normalized [0, 1] to pixel values)
    polygon = []
    for i in range(0, len(polygon_points), 2):
        x = int(polygon_points[i] * width)
        y = int(polygon_points[i + 1] * height)
        polygon.append([x, y])
    
    # Convert the polygon to a numpy array
    polygon = np.array(polygon, np.int32)
    polygon = polygon.reshape((-1, 1, 2))  # OpenCV expects a specific shape
    
    # Draw the polygon on the image using the corresponding color
    cv2.polylines(image, [polygon], isClosed=True, color=colors[class_id], thickness=2)

# Convert BGR to RGB for Matplotlib
image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Display the image with Matplotlib
plt.figure(figsize=(10, 10))
plt.imshow(image_rgb)
plt.axis('off')  # Hide axis
plt.show()
