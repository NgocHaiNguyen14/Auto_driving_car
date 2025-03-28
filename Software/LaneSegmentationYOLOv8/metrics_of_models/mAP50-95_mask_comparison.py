import pandas as pd
import matplotlib
matplotlib.use('Agg')  # Use a non-interactive backend
import matplotlib.pyplot as plt

# File paths for the CSV files
file1 = 'metrics_of_models/model_metrics_large.csv'
file2 = 'metrics_of_models/model_metrics_medium.csv'
file3 = 'metrics_of_models/model_metrics_small.csv'

# Read the CSV files into DataFrames
df1 = pd.read_csv(file1)
df2 = pd.read_csv(file2)
df3 = pd.read_csv(file3)

# Extract Mask(P) values for each DataFrame
mask_p1 = df1['Mask(mAP50-95)']
mask_p2 = df2['Mask(mAP50-95)']
mask_p3 = df3['Mask(mAP50-95)']
mask_p4 = [0.585, 0.531, 0.501, 0.723] 

# Extract classes to align the Mask(P) values for comparison
classes = df1['Class']  # assuming all files have the same classes in the same order

# Plot the comparison
plt.figure(figsize=(12, 6))

plt.plot(classes, mask_p1, marker='o', linestyle='-', label='yolov8-large')
plt.plot(classes, mask_p2, marker='s', linestyle='--', label='yolov8-medium')
plt.plot(classes, mask_p3, marker='^', linestyle='-.', label='yolov8-small')
plt.plot(classes, mask_p4, marker='x', linestyle='-.', label='Mask_RCNN')

plt.xlabel('Class')
plt.ylabel('Mask(mAP50-95)')
plt.title('Comparison of Mask(mAP50-95) Across Models')
plt.xticks(rotation=45)
plt.legend()
plt.grid(True)

plt.tight_layout()

# Save the plot to a file
plt.savefig('mask_mAP50-95_comparison.png')
