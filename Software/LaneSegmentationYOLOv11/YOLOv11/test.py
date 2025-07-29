import argparse
from ultralytics import YOLO

# Parse command-line arguments
parser = argparse.ArgumentParser(description="Run YOLOv8 instance segmentation on an image.")
parser.add_argument('--input', type=str, required=True, help='Path to input image')
args = parser.parse_args()

# Load the model
model = YOLO('/home/sefas/Documents/Hai/YOLOv8_train_AutonomousVehicle/YOLOv11/runs/segment/train/weights/best.pt')

# Run inference
results = model(args.input, save=True)

# Show the result
results[0].show()

