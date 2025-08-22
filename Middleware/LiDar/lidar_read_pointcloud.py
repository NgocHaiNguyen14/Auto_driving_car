#!/usr/bin/env python3
import rospy
import numpy as np
import matplotlib.pyplot as plt
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
import threading

# Shared data
latest_points = None
lock = threading.Lock()

def cloud_callback(msg):
    global latest_points
    points_list = []

    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points_list.append([p[0], p[1]])  # Only X and Y for 2D plot

    with lock:
        latest_points = np.array(points_list)

def main():
    global latest_points
    rospy.init_node('lidar_realtime_visualizer_2d_rays', anonymous=True)
    rospy.Subscriber("/cloud", PointCloud2, cloud_callback)
    rospy.loginfo("Listening to /cloud and plotting 2D LiDAR rays in real-time...")

    plt.ion()
    fig, ax = plt.subplots()

    while not rospy.is_shutdown():
        with lock:
            if latest_points is not None:
                ax.clear()

                # Draw rays from origin to each point
                for x, y in latest_points:
                    ax.plot([0, x], [0, y], color='blue', linewidth=0.5, alpha=0.5)

                # Draw all points
                ax.scatter(latest_points[:, 0], latest_points[:, 1], c='b', s=2)

                # Draw LiDAR center
                ax.scatter(0, 0, c='yellow', s=50, edgecolors='black', zorder=5, label='LiDAR Origin')

                ax.set_xlabel("X (meters)")
                ax.set_ylabel("Y (meters)")
                ax.set_title("LiDAR 2D Scan - Rays to Points")
                ax.axis("equal")
                ax.legend(loc="upper right")

        plt.draw()
        plt.pause(0.01)

if __name__ == '__main__':
    main()

