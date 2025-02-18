import cv2
import numpy as np
import matplotlib.pyplot as plt
from ultralytics import YOLO

# Load the YOLOv8 model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8m_seg/weights/best.pt')

# Perform inference on the image
results = model('frame_00013.jpg')
image = cv2.imread('frame_00013.jpg')
# Extract segmentation masks and class IDs
masks = results[0].masks.data.cpu().numpy()
class_preds = results[0].boxes.cls.cpu().numpy()

# Load the original image

image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Convert to RGB for visualization
orig_h, orig_w, _ = image.shape  # Get original image dimensions

# Create a blank overlay
overlay = np.zeros_like(image, dtype=np.uint8)

# Define colors for each class (BGR format for OpenCV)
colors = {
    0: (255, 0, 0),    # Red for Vehicle
    1: (0, 255, 0),    # Green for Marking
    2: (0, 0, 255)     # Blue for Lane
}

# Iterate through masks and overlay them
for i, cls in enumerate(class_preds):
    mask = (masks[i] > 0.5).astype(np.uint8)  # Threshold mask
    mask = cv2.resize(mask, (orig_w, orig_h), interpolation=cv2.INTER_CUBIC)  # Resize to original image size
    color = colors.get(cls, (255, 255, 255))  # Default white if class not found
    
    for c in range(3):  # Apply mask color to each channel
        overlay[:, :, c] = np.where(mask == 1, color[c], overlay[:, :, c])

# Blend the overlay with the original image
alpha = 0.5  # Transparency factor
masked_image = cv2.addWeighted(image, 1, overlay, alpha, 0)

# Save the result
cv2.imwrite('masked_01375.png', cv2.cvtColor(masked_image, cv2.COLOR_RGB2BGR))

# Display the result
plt.figure(figsize=(10, 5))
plt.imshow(masked_image)
plt.title("Segmented Image with Masks")
plt.axis('off')
plt.show()
