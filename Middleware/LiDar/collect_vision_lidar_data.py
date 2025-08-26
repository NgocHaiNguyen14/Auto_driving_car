#!/usr/bin/env python3
import rospy
import numpy as np
import cv2
import os
import sys
import select
import threading
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2

# Shared data
latest_points = None
lock_lidar = threading.Lock()

save_dir = "/home/sefas/Documents/Hai/Fusion/data1"
file_counter = 1

# Ensure directory exists
os.makedirs(save_dir, exist_ok=True)

# Open camera
cap = cv2.VideoCapture(0)  # Change index if needed
if not cap.isOpened():
    raise RuntimeError("Cannot open camera")

def cloud_callback(msg):
    global latest_points
    points_list = []
    for p in pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True):
        points_list.append([p[0], p[1]])
    with lock_lidar:
        latest_points = np.array(points_list)

def save_data():
    global file_counter, latest_points

    # Capture image from webcam
    ret, img = cap.read()
    if not ret:
        rospy.logwarn("Failed to capture image from camera")
        return

    with lock_lidar:
        lidar_copy = None if latest_points is None else latest_points.copy()

    if lidar_copy is None:
        rospy.logwarn("No LiDAR data to save yet.")
        return

    img_path = os.path.join(save_dir, f"pic{file_counter}.jpg")
    lidar_path = os.path.join(save_dir, f"lidar{file_counter}.npy")

    cv2.imwrite(img_path, img)
    np.save(lidar_path, lidar_copy)

    rospy.loginfo(f"Saved {img_path} and {lidar_path}")

    # --- Extra: check LiDAR rays < 0.2 m ---
    distances = np.sqrt(lidar_copy[:,0]**2 + lidar_copy[:,1]**2)
    close_idx = np.where(distances < 0.2)[0]
    if close_idx.size > 0:
        rospy.loginfo("LiDAR rays < 0.2 m:")
        for i in close_idx:
            rospy.loginfo(f"Ray {i}: ({lidar_copy[i,0]:.3f}, {lidar_copy[i,1]:.3f}), Dist={distances[i]:.3f} m")
    else:
        rospy.loginfo("No LiDAR rays closer than 0.2 m")

    file_counter += 1


def keyboard_listener():
    while not rospy.is_shutdown():
        if sys.stdin in select.select([sys.stdin], [], [], 0)[0]:
            key = sys.stdin.readline().strip()
            if key.lower() == 'd':
                save_data()

if __name__ == "__main__":
    rospy.init_node("fusion_data_capture_cv", anonymous=True)
    rospy.Subscriber("/cloud", PointCloud2, cloud_callback)

    kb_thread = threading.Thread(target=keyboard_listener)
    kb_thread.daemon = True
    kb_thread.start()

    rospy.loginfo("Press 'd' then Enter to save camera image + lidar data")

    rospy.spin()

    cap.release()

