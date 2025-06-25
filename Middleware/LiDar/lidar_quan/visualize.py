import pandas as pd
import matplotlib.pyplot as plt

# Load the CSV file (update 'your_file.csv' with the actual filename)
df = pd.read_csv('lidar/sensor_data/pointcloud_20250307-112805.csv', delimiter='\t')  # Use '\t' if tab-separated, ',' if comma-separated
df.columns = df.columns.str.strip()  # Remove any leading/trailing spaces
print(df)
# Plot the data
plt.figure(figsize=(8, 6))
plt.scatter(df['x'], df['y'], s=5, label="LiDAR Points", color='blue')

# Labels and title
plt.xlabel("X Coordinate")
plt.ylabel("Y Coordinate")
plt.title("LiDAR Data Points")
plt.legend()
plt.grid(True)

# Show the plot
plt.show()