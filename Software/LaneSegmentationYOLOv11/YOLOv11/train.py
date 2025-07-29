

from ultralytics import YOLO



# Load a model
model = YOLO("yolo11n-seg.pt")

# Train the model
train_results = model.train(
    data="/home/sefas/Documents/Hai/YOLOv8_train_AutonomousVehicle/LabeledData/YOLODataset/dataset.yaml",  # path to dataset YAML
    epochs=100,  # number of training epochs
    imgsz=640,  # training image size
    device=0,  # device to run on, i.e. device=0 or device=0,1,2,3 or device=cpu
)


