from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QGridLayout, QWidget, QLabel
from PySide6.QtGui import QPixmap
from stream import Stream
from map import Map
from lidar import Obstacle

class SecondWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Monitor Window")
        self.setGeometry(100, 100, 400, 200)

        self.grid_layout = QGridLayout()

        # Video source
        self.stream = Stream()
        
        # Show the path to avoid obstacles
        self.obs = Obstacle()
        
        # Show map
        self.map = QLabel()
        # self.map.setPixmap(QPixmap("D:\Desktop\path_map.html").scaled(320, 240))
        self.map.setGeometry(100, 100, 640, 480)

        self.search = Map()
        self.search.setGeometry(100, 100, 640, 480)

        #Lay out
        self.grid_layout.addWidget(self.stream, 0, 0, 3, 3)
        self.grid_layout.addWidget(self.obs, 0, 3, 3, 3)
        self.grid_layout.addWidget(self.search, 3, 0)
        self.grid_layout.addWidget(self.map, 5, 0, 3, 3)

        central_widget = QWidget()
        central_widget.setLayout(self.grid_layout)
        self.setCentralWidget(central_widget)
        