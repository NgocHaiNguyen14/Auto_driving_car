from PySide6.QtWidgets import  QLabel, QWidget, QGridLayout
from PySide6.QtCore import Qt

class Sensor(QWidget):
    def __init__(self, stat):
        super().__init__()
        self.setWindowTitle("Bảng trạng thái của các loại cảm biến")

        grid_layout = QGridLayout()

        header = QLabel("Thông số cảm biến")
        
        battery_stat = QLabel("Pin : ")
        bat_value = QLabel(str(stat[0]) + "%")

        battery_temp = QLabel("Nhiệt độ Pin : ")
        bat_temp_value = QLabel(str(stat[1]) + "°C")

        system_stat = QLabel("Nhiệt độ trung bình hệ thống : ")
        system_value = QLabel(str(stat[2]) + "°C")

        tire_stat = QLabel("Áp suất lốp : ")
        tire_value = QLabel(str(stat[3]) + " Pa")

        steering_stat = QLabel("Góc lái : ")
        steer_value = QLabel(str(stat[4]) + "°")

        front_dist = QLabel("Khoảng cách vật cản trước : ")
        front_value = QLabel(str(stat[5]) + " cm")

        rear_dist = QLabel("Khoảng cách vật cản bên : ")
        rear_value = QLabel(str(stat[6]) + " cm")

        grid_layout = QGridLayout()
        
        header.setAlignment(Qt.AlignCenter)
        grid_layout.addWidget(header,0,1,1,4) #Take up 1rows and 4 column
        
        grid_layout.addWidget(battery_stat,1,0,1,3)
        grid_layout.addWidget(bat_value,1,5)

        grid_layout.addWidget(battery_temp,2,0,1,3)
        grid_layout.addWidget(bat_temp_value,2,5)

        grid_layout.addWidget(system_stat,3,0,1,3)
        grid_layout.addWidget(system_value,3,5)

        grid_layout.addWidget(tire_stat,4,0,1,3)
        grid_layout.addWidget(tire_value,4,5)

        grid_layout.addWidget(steering_stat,5,0,1,3)
        grid_layout.addWidget(steer_value,5,5)

        grid_layout.addWidget(front_dist,6,0,1,3)
        grid_layout.addWidget(front_value,6,5)

        grid_layout.addWidget(rear_dist,7,0,1,3)
        grid_layout.addWidget(rear_value,7,5)

        self.setLayout(grid_layout)
        