import matplotlib.pyplot as plt
import numpy as np
import ast

# Read the data file
with open('lidar_datatxt', 'r') as f:
    content = f.read()

# Extract angle and range info
angle_min = float(content.split("angle_min:")[1].split("\n")[0].strip())
angle_max = float(content.split("angle_max:")[1].split("\n")[0].strip())
angle_increment = float(content.split("angle_increment:")[1].split("\n")[0].strip())

ranges_text = content.split("ranges:")[1].split("intensities:")[0].strip().rstrip(',')
ranges = ast.literal_eval(ranges_text)

range_min = float(content.split("range_min:")[1].split("\n")[0].strip())
range_max = float(content.split("range_max:")[1].split("\n")[0].strip())
ranges = np.array(ranges)
ranges[(ranges < range_min) | (ranges > range_max)] = np.nan

# Compute angles and Cartesian coordinates
angles = angle_min + np.arange(len(ranges)) * angle_increment
x = ranges * np.cos(angles)
y = ranges * np.sin(angles)

# Plot setup
plt.figure(figsize=(8, 8))
plt.title("LiDAR 2D Scan - Rays to Points")
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.axis('equal')
plt.grid(True)

# Draw red center point
plt.plot(0, 0, 'ro', markersize=5)

# Draw rays from center to each valid point
for xi, yi in zip(x, y):
    if not np.isnan(xi) and not np.isnan(yi):
        plt.plot([0, xi], [0, yi], color='blue', linewidth=0.3)

plt.show()

