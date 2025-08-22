#!/usr/bin/env python3
import numpy as np
import matplotlib.pyplot as plt
import sys

if len(sys.argv) != 2:
    print(f"Usage: {sys.argv[0]} lidar_file.npy")
    sys.exit(1)

lidar_data = np.load(sys.argv[1])  # Shape: (N, 2) for X, Y

plt.figure(figsize=(6, 6))
plt.scatter(lidar_data[:, 0], lidar_data[:, 1], c='b', s=2)
plt.scatter(0, 0, c='r', s=50, label='LiDAR Origin')
plt.axis('equal')
plt.xlabel("X (meters)")
plt.ylabel("Y (meters)")
plt.title("Saved LiDAR Points")
plt.legend()
plt.show()

