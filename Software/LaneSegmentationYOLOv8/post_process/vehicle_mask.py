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

# Initialize masks for lane, marking, and vehicle
lane_mask = np.zeros(masks.shape[1:], dtype=np.uint8)
marking_mask = np.zeros(masks.shape[1:], dtype=np.uint8)
vehicle_mask = np.zeros(masks.shape[1:], dtype=np.uint8)

# Iterate through masks and add pixels to the respective mask based on class
for i, cls in enumerate(class_preds):
    if cls == 2:  # Class 2 is the lane
        lane_mask = np.maximum(lane_mask, masks[i])
    elif cls == 1:  # Class 1 is the marking
        marking_mask = np.maximum(marking_mask, masks[i])
    elif cls == 0:  # Class 0 is the vehicle
        vehicle_mask = np.maximum(vehicle_mask, masks[i])

# Define the polygon points for drawing on the image
front1_points = np.array([[220, 320], [364, 320], [338, 285], [246, 285]], dtype=np.int32)  # Red
front2_points = np.array([[338, 285], [246, 285], [272, 250], [312, 250]], dtype=np.int32)  # Green
right_points = np.array([[313, 250], [332, 250], [384, 320], [365, 320]], dtype=np.int32)  # Yellow
left_points = np.array([[271, 250], [252, 250], [200, 320], [221, 320]], dtype=np.int32)  # Yellow

# Load the original image
image = cv2.imread('test.png')
image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for display

# Resize the image to 640x384
image_resized = cv2.resize(image, (640, 384))

# Draw the polygons on the resized image
cv2.polylines(image_resized, [front1_points], isClosed=True, color=(255, 0, 0), thickness=2)
cv2.polylines(image_resized, [front2_points], isClosed=True, color=(0, 255, 0), thickness=2)
cv2.polylines(image_resized, [right_points], isClosed=True, color=(255, 255, 0), thickness=2)
cv2.polylines(image_resized, [left_points], isClosed=True, color=(255, 255, 0), thickness=2)

# Create masks for each polygon
front1_mask = np.zeros_like(lane_mask)
front2_mask = np.zeros_like(lane_mask)
right_mask = np.zeros_like(lane_mask)
left_mask = np.zeros_like(lane_mask)

cv2.fillPoly(front1_mask, [front1_points], 1)
cv2.fillPoly(front2_mask, [front2_points], 1)
cv2.fillPoly(right_mask, [right_points], 1)
cv2.fillPoly(left_mask, [left_points], 1)

# Count the number of pixels for lane and marking in each polygon
front1_lane_count = np.sum(lane_mask * front1_mask)
front1_marking_count = np.sum(marking_mask * front1_mask)
front1_vehicle_count = np.sum(vehicle_mask * front1_mask)

front2_lane_count = np.sum(lane_mask * front2_mask)
front2_marking_count = np.sum(marking_mask * front2_mask)
front2_vehicle_count = np.sum(vehicle_mask * front2_mask)

right_lane_count = np.sum(lane_mask * right_mask)
right_marking_count = np.sum(marking_mask * right_mask)
right_vehicle_count = np.sum(vehicle_mask * right_mask)

left_lane_count = np.sum(lane_mask * left_mask)
left_marking_count = np.sum(marking_mask * left_mask)
left_vehicle_count = np.sum(vehicle_mask * left_mask)

# Print the results
print(f"Front1 - Lane pixels: {front1_lane_count}, Marking pixels: {front1_marking_count}, Vehicle pixels: {front1_vehicle_count}")
print(f"Front2 - Lane pixels: {front2_lane_count}, Marking pixels: {front2_marking_count}, Vehicle pixels: {front2_vehicle_count}")
print(f"Right - Lane pixels: {right_lane_count}, Marking pixels: {right_marking_count}, Vehicle pixels: {right_vehicle_count}")
print(f"Left - Lane pixels: {left_lane_count}, Marking pixels: {left_marking_count}, Vehicle pixels: {left_vehicle_count}")

# Visualize the resized image with the polygons and vehicle mask
plt.figure(figsize=(15, 5))

# Plot the resized image with the polygons
plt.subplot(1, 3, 1)
plt.imshow(image_resized)
plt.title("Resized Image with Polygons (640x384)")
plt.axis('off')

# Plot the lane mask
plt.subplot(1, 3, 2)
plt.imshow(vehicle_mask, cmap='gray')
plt.title("Vehicle Mask (Class 0)")
plt.axis('off')

# Plot the marking mask
plt.subplot(1, 3, 3)
plt.imshow(lane_mask, cmap='gray')
plt.title("Lane Mask (Class 2)")
plt.axis('off')

plt.show()
