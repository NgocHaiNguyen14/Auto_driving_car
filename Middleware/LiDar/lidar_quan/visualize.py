#!/usr/bin/env python

import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import matplotlib.pyplot as plt
from queue import Queue
import numpy as np
import time
# Initialize matplotlib
plt.ion()  # Enable interactive mode
fig, ax = plt.subplots()  # Create a 2D plot

# Create a queue to hold the point cloud data
data_queue = Queue()

def cloud_callback(data):
    # Convert PointCloud2 data to a list of points (x, y)
    points = list(pc2.read_points(data, field_names=("x", "y"), skip_nans=True))
    
    if points:  # Only process if there are points
        # Store the points in the queue
        data_queue.put(points)

def cloud_listener():
    rospy.init_node('cloud_listener', anonymous=True)
    
    # Register subscriber to listen to the /cloud topic
    rospy.Subscriber("/cloud", PointCloud2, cloud_callback)

    while not rospy.is_shutdown():
        if not data_queue.empty():
            # Process and plot a batch of points
            points = data_queue.get()
            if points:
                start = time.time()
                points = np.array(points)  # Convert to numpy array for faster processing
                ax.cla()  # Clear the previous points
                ax.scatter(points[:, 0], points[:, 1], c='b', marker='o', s=1)  # Plot only x and y
                ax.set_xlabel('x')
                ax.set_ylabel('y')
                ax.set_title('2D Point Cloud Visualization')
                plt.xlim(points[:, 0].min(), points[:, 0].max())  # Set x limits
                plt.ylim(points[:, 1].min(), points[:, 1].max())  # Set y limits
                plt.draw()
                plt.pause(0.0001)  # Adjust the pause duration as necessary
                print(f"Time elapsed:{time.time()-start:.6f}")

if __name__ == '__main__':
    try:
        cloud_listener()
    except rospy.ROSInterruptException:
        pass
