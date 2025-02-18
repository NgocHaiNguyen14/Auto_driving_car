import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parse data entries
data_list = []

with open("lidar20_11_2.txt", "r") as file:
    data_entry = {}
    for line in file:
        line = line.strip()
        if line.startswith("header"):
            data_entry = {"header": line}
        elif line.startswith("data:"):
            data_values = line.split("data:")[1].strip().strip("[]")
            data_entry["data"] = list(map(int, data_values.split(',')))
        elif line.startswith("is_dense:"):
            data_entry["is_dense"] = line.split("is_dense:")[1].strip() == "True"
        elif line == "---":
            data_list.append(data_entry)

# Create figure and axis for animation
fig, ax = plt.subplots()

# Invert the y-axis once for the entire animation
ax.invert_yaxis()
scatter = ax.scatter([], [], c='blue', marker='.')
ax.set_xlabel('Y')
ax.set_ylabel('X')
ax.set_title("2D Point Cloud Animation")

def update(frame, first_point=248, last_point=270):
    ax.clear()
    ax.invert_xaxis()
    data = data_list[frame] 
    data_bytes = np.array(data["data"], dtype=np.uint8).tobytes()
    num_points = len(data["data"]) // 16
    point_data = np.frombuffer(data_bytes, dtype=np.float32).reshape(num_points, 4)

    # Slice data to include only the specified range
    x = point_data[first_point:last_point, 0]
    print(x.size)
    y = point_data[first_point:last_point, 1]
    for xi, yi in zip(x, y):
        ax.plot([0, yi], [0, xi], color='gray', linewidth=0.5)
    # Scatter plot for current frame
    ax.scatter(y, x, c= 'red', marker='o', s=1)
    ax.set_xlabel('Y')
    ax.set_ylabel('X')
    ax.set_title(f"2D Point Cloud - Frame {frame}")
ani = FuncAnimation(fig, update, frames=len(data_list), interval=20)

plt.show()
