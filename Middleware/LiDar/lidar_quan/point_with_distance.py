import rospy
from sensor_msgs.msg import PointCloud2
import sensor_msgs.point_cloud2 as pc2
from queue import Queue
import time  # Import time module for measuring time
import numpy as np

# Tạo hàng đợi để lưu trữ dữ liệu
data_queue = Queue()

def cloud_callback(data):
    # Chuyển đổi dữ liệu PointCloud2 thành danh sách các điểm (x, y)
    points = list(pc2.read_points(data, field_names=("x", "y"), skip_nans=True))
    if points:  # Chỉ xử lý nếu có điểm
        start_time = time.time()
        points_array = np.array(points)  # Mảng (N, 2) với N là số lượng điểm

        # Tính toán cột sqrt(x^2 + y^2)
        distances = np.sqrt(points_array[:, 0]**2 + points_array[:, 1]**2)

        # Kết hợp cột mới vào mảng điểm
        points_with_distances = np.column_stack((points_array, distances))

        # Lưu hoặc xử lý dữ liệu mới theo nhu cầu
        data_queue.put(points_with_distances)
        end_time = time.time()  # Thời gian kết thúc
        elapsed_time = end_time - start_time
        #print(f"Time taken to get distance: {elapsed_time:.6f} seconds")

def cloud_listener():
    rospy.init_node('cloud_listener', anonymous=True)
    # Đăng ký subscriber để lắng nghe topic /cloud
    rospy.Subscriber("/cloud", PointCloud2, cloud_callback)

    while not rospy.is_shutdown():
        start_time = time.time()
        while data_queue.empty():
            time.sleep(0.01)  # Thêm thời gian chờ để tránh sử dụng CPU quá mức

        if not data_queue.empty():
            points = data_queue.get()
            if points is not None:
                print(points[270][2])
                end_time = time.time()  # Thời gian kết thúc
                elapsed_time = end_time - start_time
                #print(f"Time taken to get data: {elapsed_time:.6f} seconds")
                # Bạn có thể xử lý dữ liệu tại đây nếu cần

if __name__ == '__main__':
    try:
        cloud_listener()
    except rospy.ROSInterruptException:
        pass
