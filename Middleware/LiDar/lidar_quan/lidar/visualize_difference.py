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
ax.set_xlabel('Index')
ax.set_ylabel('Distance Difference')
ax.set_title("Distance Difference Over Time")
ax.grid(True)

line, = ax.plot([], [], 'b-', lw=1)  # Line for distance differences

def compute_distance_diff(frame, first_point=248, last_point=270):
    """ Compute distances and their differences for a given frame """
    data = data_list[frame]
    data_bytes = np.array(data["data"], dtype=np.uint8).tobytes()
    num_points = len(data["data"]) // 16
    point_data = np.frombuffer(data_bytes, dtype=np.float32).reshape(num_points, 4)

    # Extract (x, y) in the given range
    x = point_data[first_point:last_point, 0]
    y = point_data[first_point:last_point, 1]

    # Compute distances
    distances = np.sqrt(x**2 + y**2)

    # Compute difference between consecutive distances
    distance_diff = np.diff(distances)

    return distance_diff

def update(frame):
    """ Update animation frame """
    ax.clear()
    ax.set_xlabel('Index')
    ax.set_ylabel('Distance Difference')
    ax.set_title(f"Distance Difference - Frame {frame}")
    ax.grid(True)

    # Compute distance differences
    distance_diff = compute_distance_diff(frame)

    # Plot differences
    ax.plot(range(len(distance_diff)), distance_diff, 'b-', lw=1)

ani = FuncAnimation(fig, update, frames=len(data_list), interval=20)

plt.show()
