import numpy as np
import struct
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

class PointCloud2:
    def __init__(self, data, fields, width, height, point_step, row_step, is_bigendian):
        self.data = data
        self.fields = fields
        self.width = width
        self.height = height
        self.point_step = point_step
        self.row_step = row_step
        self.is_bigendian = is_bigendian

    def read_points(self):
        fmt = '<fff'  # little-endian, 3 floats (x, y, z)
        points = []
        for i in range(0, len(self.data), self.point_step):
            point = struct.unpack(fmt, self.data[i:i+12])
            points.append(point)
        return np.array(points)

def visualize(points):
    fig = plt.figure()
    ax = fig.add_subplot(111, projection='3d')
    ax.scatter(points[:, 0], points[:, 1], points[:, 2], c='r', marker='o')
    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_zlabel('Z Label')
    plt.show()

def read_data_from_file(file_path):
    with open(file_path, 'r') as file:
        data = file.read()
    # Remove commas and other non-numeric characters, then split by spaces
    data = data.replace(',', ' ').split()
    # Convert to integers
    data = [int(x) for x in data if x.isdigit()]
    data = bytes(data)
    return data

# Read data from the text file
file_path = 'D:\Desktop\lidar_data_cloud.txt'
data = read_data_from_file(file_path)

# PointCloud2 parameters
fields = ["x", "y", "z"]
width = 541
height = 1
point_step = 16
row_step = 8656
is_bigendian = False

# Create a PointCloud2 instance
pc = PointCloud2(data=data, fields=fields, width=width, height=height, point_step=point_step, row_step=row_step, is_bigendian=is_bigendian)

# Read points from the point cloud
points = pc.read_points()

# Visualize the points
visualize(points)


