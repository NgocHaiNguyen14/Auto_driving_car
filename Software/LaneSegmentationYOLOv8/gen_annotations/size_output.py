from ultralytics import YOLO
import torch

# Load the YOLOv8 large segmentation model
model = YOLO('/home/sefas/Auto_driving_car/Software/LaneSegmentationYOLOv8/models/yolov8l_seg/weights/best.pt')

# Create a sample input tensor with a batch size of 1, 3 color channels, and a 640x640 image
sample_input = torch.randn(1, 3, 640, 640)

# Run a forward pass through the model to get the output
output = model(sample_input)

# Print the size of the output
print("Output size:", output.shape)
