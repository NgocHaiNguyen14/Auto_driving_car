from ultralytics import YOLO
import cv2
import math

# Hàm tính góc từ bounding box
def bbox_to_angle2(x_min, x_max, W, HFOV):
    theta1 = 271 + HFOV - 2 * x_min * HFOV / W
    theta2 = 271 + HFOV - 2 * x_max * HFOV / W
    return theta1, theta2

def bbox_to_angle(x_min, x_max, W, HFOV):
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
W = 1920
HFOV = 66.7 

model = YOLO("yolov8n.pt").to("cuda")

image_path = "20_11_2.jpg"
image = cv2.imread(image_path)

results = model(image)

if len(results):
    for box in results[0].boxes.xyxy:
        # Lấy tọa độ bounding box (x_min, y_min, x_max, y_max)
        x_min, y_min, x_max, y_max = map(int, box.cpu().numpy())

        # Vẽ bounding box
        cv2.rectangle(image, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

        theta1, theta2 = bbox_to_angle(x_min, x_max, W, HFOV)

        # Ghi thông tin lên bounding box
        text = f"Angle: {theta1:.2f}, {theta2:.2f}"
        cv2.putText(image, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Hiển thị ảnh
    cv2.imshow("Detected Image", image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Không tìm thấy bounding box nào.")
