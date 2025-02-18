from ultralytics import YOLO
import cv2
import math

# Hàm tính góc từ bounding box
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
    theta1 = 271 - 2 * math.degrees(math.atan(x1 / f))  # Đổi từ radian sang độ
    theta2 = 271 - 2 * math.degrees(math.atan(x2 / f))  # Đổi từ radian sang độ

    return theta1, theta2

# Khởi tạo thông số
Height = 1080
Width = 1920
HFOV = 66.7

# Tải mô hình YOLO
model = YOLO("yolov8n.pt").to("cuda")

# Sử dụng webcam (ID 0 là webcam mặc định)
cap = cv2.VideoCapture(0)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, Width)  # Set width to 1280
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, Height)  # Set height to 720
if not cap.isOpened():
    print("Không thể mở webcam.")
    exit()

while True:
    # Đọc frame từ webcam
    ret, frame = cap.read()
    if not ret:
        print("Không thể đọc frame từ webcam.")
        break
    resized_frame = cv2.resize(frame, (960, 540))

    # Dự đoán bằng YOLO
    results = model(frame)
    if len(results):
        for box in results[0].boxes.xyxy:
            # Lấy tọa độ bounding box (x_min, y_min, x_max, y_max)
            x_min, y_min, x_max, y_max = map(int, box.cpu().numpy())

            # Vẽ bounding box
            cv2.rectangle(resized_frame, (x_min, y_min), (x_max, y_max), (0, 255, 0), 2)

            # Tính góc từ bounding box
            theta1, theta2 = bbox_to_angle(x_min, x_max, Width, HFOV)

            # Ghi thông tin góc lên bounding box
            text = f"Angle: {theta1:.2f}, {theta2:.2f}"
            cv2.putText(resized_frame, text, (x_min, y_min - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # Hiển thị khung hình
    cv2.imshow("Real-Time Detection", resized_frame)

    # Thoát khi nhấn phím 'q'
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Giải phóng tài nguyên
cap.release()
cv2.destroyAllWindows()
