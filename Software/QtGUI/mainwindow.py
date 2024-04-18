from PySide6.QtWidgets import QApplication, QMainWindow, QPushButton, QGridLayout, QWidget, QLabel
from sensor import Sensor
import time

def read_text(file_path):
        with open(file_path, 'r') as file:
            lines = file.readlines()
        
        while True:

            last_line = lines[-1].split()  # Split the last line into columns
            time.sleep(1)

            return last_line

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Main Window")
        self.setGeometry(100, 100, 400, 200)

        self.grid_layout = QGridLayout()


        # Create a button to show stat
        self.stat_button = QPushButton("Thông số")
        self.stat_button.clicked.connect(self.stat_table)

        # Create a button to quit the application
        self.quit_button = QPushButton("Quit")
        self.quit_button.clicked.connect(self.quitApplication)

        #Lay out
        self.grid_layout.addWidget(self.stat_button, 0, 0, 2, 2)
        self.grid_layout.addWidget(self.quit_button, 2, 0, 2, 2)

        central_widget = QWidget()
        central_widget.setLayout(self.grid_layout)
        self.setCentralWidget(central_widget)

    def stat_table(self):
        stat_file = "D:/Desktop/Qt GUI/StatTest.txt"
        stat = read_text(stat_file)
        self.sensor = Sensor(stat)
        self.sensor.show()
    
    def quitApplication(self):
        QApplication.quit()
        