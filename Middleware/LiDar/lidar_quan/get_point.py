import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from queue import Queue
import time  # Import time module for measuring time
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
    points = []
    while not rospy.is_shutdown():
        start_time = time.time()
        while data_queue.empty():
            pass
        if not data_queue.empty():
            points = data_queue.get()
            if points:
                end_time = time.time()  # End time
                elapsed_time = end_time - start_time
                print(f"Time taken to get data: {elapsed_time:.6f} seconds")
if __name__ == '__main__':
    try:
        cloud_listener()
    except rospy.ROSInterruptException:
        pass

