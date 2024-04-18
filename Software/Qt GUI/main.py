from PySide6.QtWidgets import QApplication
from mainwindow import MainWindow
from secondwindow import SecondWindow
import sys

app = QApplication(sys.argv)

main_window = MainWindow()
main_window.setGeometry(100, 100, 200, 200)
main_window.setWindowTitle("Main Window")

main_window.show()

second_window = SecondWindow()
second_window.setGeometry(500, 200, 1000, 600)
second_window.setWindowTitle("Monitor Window")

second_window.show()

#Start event loop
app.exec()
