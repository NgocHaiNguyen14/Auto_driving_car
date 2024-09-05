from ultralytics import YOLO
import yaml
import json

# Load the model
model_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentation/models/yolov8s_seg/weights/best.pt'
model = YOLO(model_path)

# Load dataset configuration
data_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentation/dataset/YOLODataset/dataset.yaml'
with open(data_path, 'r') as file:
    data_config = yaml.safe_load(file)

# Evaluate the model
results = model.val(data=data_path)

# Extract performance metrics
metrics = {
    'maps': results.maps,
    'fitness': results.fitness,
    'results_dict': results.results_dict,
    'speed': results.speed,
}

# Save metrics to a JSON file
with open('performance_metrics.json', 'w') as f:
    json.dump(metrics, f, indent=4)

print("Evaluation complete. Metrics saved to performance_metrics.json.")
