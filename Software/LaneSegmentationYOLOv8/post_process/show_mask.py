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

# Iterate through masks and add pixels of the lane class (class 0) to the lane_mask
for i, cls in enumerate(class_preds):
    if cls == 2:  # Class 0 is the lane
        lane_mask = np.maximum(lane_mask, masks[i])

# Load the original image
image = cv2.imread('test.png')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for display

# Visualize the lane mask
plt.figure(figsize=(10, 5))

# Plot the original image
plt.subplot(1, 2, 1)
plt.imshow(image)
plt.title("Original Image")
plt.axis('off')

# Plot the lane mask
plt.subplot(1, 2, 2)
plt.imshow(lane_mask, cmap='gray')
plt.title("Lane Mask")
plt.axis('off')

plt.show()
