from PySide6.QtWidgets import QVBoxLayout, QWidget, QPushButton
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
import matplotlib.pyplot as plt
import numpy as np
import networkx as nx

def start_end(i, geo_path):
    start = geo_path[i]
    end = geo_path[i + 1]
    i += 1
    return start, end, i

def read_geo(filepath):
    with open(filepath, 'r') as file:
        lines = file.read().splitlines()

    geo_path = []
    for i in range(1, len(lines)):
        x, y = map(float, lines[i].split(","))
        geo_path.append((x, y))

    return geo_path

class Obstacle(QWidget):
    def __init__(self):
        super().__init__()

        self.fig = plt.figure()
        self.canvas = FigureCanvas(self.fig)
        self.ax = self.fig.add_subplot(111, projection='3d')
        self.button = QPushButton("Hiển thị vật cản")

        layout = QVBoxLayout()
        layout.addWidget(self.canvas)
        layout.addWidget(self.button)
        self.setLayout(layout)
        self.button.clicked.connect(self.update_and_generate_graph)
        # self.button.clicked.connect(self.generate_graph)

        self.count = 10
        self.geo_path = read_geo(r"D:/Desktop/Qt GUI/coordinates.txt")
        self.start, self.end, self.count = start_end(self.count, self.geo_path)

    def update_and_generate_graph(self):
        # Increment count to move to the next pair of points in geo_path
        self.count += 1

        # Check if there are more points in geo_path
        if self.count + 1 < len(self.geo_path):
            # Update start, end based on the new pair of points
            self.start, self.end, _ = start_end(self.count, self.geo_path)

            # Call generate_graph to update the visualization
            self.generate_graph()
        else:
            print("End of geo_path reached.")

    def distance(self, point1, point2):
        x1, y1 = point1
        x2, y2 = point2
        distance = ((x2 - x1) ** 2 + (y2 - y1) ** 2) ** (1 / 2)
        return distance

    def generate_graph(self):
            #Create surrounding matrix between 2 points
            self.ax.clear()
            self.decimal_places = 7
            self.rows, self.cols = round(abs((self.end[1] - self.start[1])) * (10 ** self.decimal_places) + 1), round(abs((self.end[0] - self.start[0])) * (10 ** self.decimal_places) + 1)
            self.matrix = np.ones((int(self.rows), int(self.cols)))

            # Randomly create blocks and assume them as obstacles
            self.blocks = []
            self.block_width = 2  # safety dist
            for _ in range(10):  # Adjusted the loop range
                x = round(np.random.uniform(0, self.cols-1), 0)
                y = round(np.random.uniform(0, self.rows-1), 0)
                node = (int(x), int(y))
                if self.distance(node, (0,0)) >= 2 and self.distance(node, (self.cols-1, self.rows-1)) >= 2:
                    height = np.random.uniform(0.1, 0.3)
                    self.ax.bar3d(x - self.block_width, y - self.block_width, 0, 2 * self.block_width,
                                2 * self.block_width, height, color='orange')

                    self.blocks.append((x, y))
                
            print(self.blocks)

            #combine safety distance to make no-touch zone
            for block in self.blocks:
                for block_x in range(max(int(block[0]) - 1, 0), min(int(block[0]) + 1, int(self.cols-1)) + 1):
                    for block_y in range(max(int(block[1]) - 1, 0),min(int(block[1]) + 1, int(self.rows-1)) + 1):
                        self.matrix[block_y, block_x] = 0

            # Initialize graph and nodes
            G = nx.Graph()
            A = (0, 0)
            G.add_node(A)
            B = (self.cols-1, self.rows-1)
            G.add_node(B)

            self.ax.scatter(A[0], A[1], 0, color='r', s=100, label='start')
            self.ax.scatter(B[0], B[1], 0, color='g', s=100, label='end')

            #Connect movable edges
            one_indices = np.argwhere(self.matrix == 1)
            for idx in one_indices:
                i, j = idx
                if i < self.rows - 1 and self.matrix[i + 1, j] == 1:
                    G.add_edge((j, i), (j, i + 1))
                if j < self.cols - 1 and self.matrix[i, j + 1] == 1:
                    G.add_edge((j, i), (j + 1, i))
                if i < self.rows - 1 and j < self.cols - 1 and self.matrix[i + 1, j + 1] == 1:
                    G.add_edge((j, i), (j + 1, i + 1))
                if 0 < i < self.rows and j < self.cols - 1 and self.matrix[i - 1, j + 1] == 1:
                    G.add_edge((j, i), (j + 1, i - 1))

             
            #Find path
            try:
                path = nx.shortest_path(G, source=A, target=B)
                path_x, path_y = zip(*path)
                path_z = np.zeros_like(path_x)
                self.ax.plot(path_x, path_y, path_z, color='purple', linewidth=1, label='Đường đi')
            except nx.NetworkXNoPath:
                print("No path")
            
            self.ax.legend()
            self.canvas.draw()
