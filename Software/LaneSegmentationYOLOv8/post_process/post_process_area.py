import cv2
import numpy as np
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('epoch100.pt')

# Perform inference on the image
results = model('test.png')

# Extract the segmentation masks
masks = results[0].masks.data.cpu().numpy()

# Extract class predictions from the bounding boxes
class_preds = results[0].boxes.cls.cpu().numpy()

# Initialize a mask for the lane (class 0)
lane_mask = np.zeros(masks.shape[1:], dtype=np.uint8)
marking_mask = np.zeros(masks.shape[1:], dtype=np.uint8)

# Iterate through masks and add pixels of the lane class (class 2) to the lane_mask
for i, cls in enumerate(class_preds):
    if cls == 2:  # Class 2 is the lane
        lane_mask = np.maximum(lane_mask, masks[i])
    elif cls == 1:  # Class 1 is the marking
        marking_mask = np.maximum(marking_mask, masks[i])

# Define the polygon points for drawing on the image
polygon1_points = np.array([[220, 320], [364, 320], [338, 285], [246, 285]], dtype=np.int32)  # Red
polygon2_points = np.array([[338, 285], [246, 285], [272, 250], [312, 250]], dtype=np.int32)  # Green
polygon3_points = np.array([[313, 250], [332, 250], [384, 320], [365, 320]], dtype=np.int32)  # Yellow
polygon4_points = np.array([[271, 250], [252, 250], [200, 320], [221, 320]], dtype=np.int32)  # Yellow

# Load the original image
image = cv2.imread('test.png')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for display

# Resize the image to 640x384
image_resized = cv2.resize(image, (640, 384))

# Draw the first polygon on the resized image with red lines
cv2.polylines(image_resized, [polygon1_points], isClosed=True, color=(255, 0, 0), thickness=2)

# Draw the second polygon on the resized image with green lines
cv2.polylines(image_resized, [polygon2_points], isClosed=True, color=(0, 255, 0), thickness=2)

# Draw the third polygon on the resized image with yellow lines
cv2.polylines(image_resized, [polygon3_points], isClosed=True, color=(255, 255, 0), thickness=2)

# Draw the fourth polygon on the resized image with yellow lines
cv2.polylines(image_resized, [polygon4_points], isClosed=True, color=(255, 255, 0), thickness=2)

# Visualize the resized image with the polygons
plt.figure(figsize=(15, 5))

# Plot the resized image with the polygons
plt.subplot(1, 3, 1)
plt.imshow(image_resized)
plt.title("Resized Image with Polygons (640x384)")
plt.axis('off')

# Plot the lane mask
plt.subplot(1, 3, 2)
plt.imshow(lane_mask, cmap='gray')
plt.title("Lane Mask (Class 2)")
plt.axis('off')

# Plot the marking mask
plt.subplot(1, 3, 3)
plt.imshow(marking_mask, cmap='gray')
plt.title("Marking Mask (Class 1)")
plt.axis('off')

plt.show()
