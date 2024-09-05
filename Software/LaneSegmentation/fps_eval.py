from ultralytics import YOLO
import matplotlib.pyplot as plt

# Paths to your trained models and datasets
models = {
    'YOLOv8 Large': '/home/sefas/Auto_driving_car/Software/LaneSegmentation/models/yolov8l_seg/weights/best.pt',
    'YOLOv8 Medium': '/home/sefas/Auto_driving_car/Software/LaneSegmentation/models/yolov8m_seg/weights/best.pt',
    'YOLOv8 Small': '/home/sefas/Auto_driving_car/Software/LaneSegmentation/models/yolov8s_seg/weights/best.pt'
}

# Path to your validation dataset (YOLO format)
data_path = '/home/sefas/Auto_driving_car/Software/LaneSegmentation/dataset/YOLODataset/dataset.yaml'

# Initialize empty lists to store metrics
model_names = []
precisions = []
recalls = []
maps_50 = []
maps_50_95 = []
fps_list = []

# Evaluate each model
for name, model_path in models.items():
    model = YOLO(model_path)  # Load the model
    results = model.val(data=data_path)  # Validate the model on the dataset

    # Extract metrics
    precisions.append(results.metrics.precision)
    recalls.append(results.metrics.recall)
    maps_50.append(results.metrics.map50)
    maps_50_95.append(results.metrics.map)  # This may vary; ensure it aligns with mAP@0.5:0.95
    fps_list.append(results.speed['inference'])  # If speed is a dictionary, adjust accordingly


# Plotting Precision, Recall, mAP@0.5, mAP@0.5:0.95
plt.figure(figsize=(14, 8))

plt.subplot(2, 2, 1)
plt.bar(model_names, precisions, color='blue')
plt.title('Precision')

plt.subplot(2, 2, 2)
plt.bar(model_names, recalls, color='green')
plt.title('Recall')

plt.subplot(2, 2, 3)
plt.bar(model_names, maps_50, color='orange')
plt.title('mAP@0.5')

plt.subplot(2, 2, 4)
plt.bar(model_names, maps_50_95, color='red')
plt.title('mAP@0.5:0.95')

plt.tight_layout()
plt.show()

# Plotting FPS
plt.figure(figsize=(6, 4))
plt.bar(model_names, fps_list, color='purple')
plt.title('FPS')
plt.xlabel('Model')
plt.ylabel('Frames Per Second')
plt.show()
