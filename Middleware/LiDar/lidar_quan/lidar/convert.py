from ultralytics import YOLO
import cv2
import math
import visualize
# Hàm tính góc từ bounding box
def bbox_to_angle2(x_min, x_max, W, HFOV):
    theta1 = 271 + HFOV - 2 * x_min * HFOV / W
    theta2 = 271 + HFOV - 2 * x_max * HFOV / W
    return theta1, theta2

def bbox_to_angle(x_min, x_max, W = 800, HFOV = 55):
    # Đổi HFOV sang radian
    HFOV_radians = math.radians(HFOV)

    # Tính tiêu cự
    f = (W / 2.0) / math.tan(HFOV_radians / 2.0)

    # Tâm ảnh
    center_x = W / 2.0

    # Tính toạ độ x từ tâm
    x1 = x_min - center_x
    x2 = x_max - center_x

    # Tính góc
    theta1 = 271 - 2*math.degrees(math.atan(x1 / f))  # Đổi từ radian sang độ
    theta2 = 271 - 2*math.degrees(math.atan(x2 / f))  # Đổi từ radian sang độ

    return theta1, theta2
import math

def point_to_distance(x, y, image_width, image_height, hfov_deg, vfov_deg, camera_height):

    # Chuyển đổi góc nhìn từ độ sang radian
    hfov_rad = math.radians(hfov_deg)
    vfov_rad = math.radians(vfov_deg)

    # Tính toán góc ngang (theta_x) và góc dọc (theta_y)
    theta_x = math.atan((x - image_width / 2) / (image_width / 2 / math.tan(hfov_rad / 2)))
    theta_y = math.atan((y - image_height / 2) / (image_height / 2 / math.tan(vfov_rad / 2)))

    # Tính khoảng cách ngang (D_x) và khoảng cách dọc (D_y)
    D_x = camera_height * math.tan(theta_x)
    D_y = camera_height * math.tan(theta_y)

    # Tính khoảng cách thực tế (D) bằng công thức Pythagoras
    D = math.sqrt(D_x**2 + D_y**2)

    return D