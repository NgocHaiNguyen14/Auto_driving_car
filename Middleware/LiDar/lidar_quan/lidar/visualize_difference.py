import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# Parse data entries
data_list = []
with open("/home/sefas/Auto_driving_car/Middleware/LiDar/lidar_quan/lidar/lidar20_11_2.txt", "r") as file:
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
#fig, ax = plt.subplots()
#ax.set_xlabel('Index')
#ax.set_ylabel('Distance Difference')
#ax.set_title("Distance Difference Over Time")
#ax.grid(True)

#line, = ax.plot([], [], 'b-', lw=1)  # Line for distance differences

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
"""
def update(frame):
    ax.clear()
    ax.set_xlabel('Index')
    ax.set_ylabel('Distance Difference')
    ax.set_title(f"Distance Difference - Frame {frame}")
    ax.grid(True)

    # Compute distance differences
    distance_diff = compute_distance_diff(frame)

    # Plot differences
    ax.plot(range(len(distance_diff)), distance_diff, 'b-', lw=1)
    """

#ani = FuncAnimation(fig, update, frames=len(data_list), interval=20)

#plt.show()

def find_obstacle_edges(frame, first_point=248, last_point=270, threshold=0.5):
    """ 
    Find the first left and right angles where distance differences exceed a threshold.

    :param frame: The frame index from `data_list`.
    :param first_point: The starting index of the points to analyze.
    :param last_point: The ending index of the points to analyze.
    :param threshold: Minimum difference to detect an obstacle edge.
    :return: Tuple (left_angle, right_angle) or (None, None) if no edges found.
    """
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
    distance_diff = np.abs(np.diff(distances))

    # Find the first index from the left where diff > threshold
    left_index = np.argmax(distance_diff > threshold)  # First occurrence
    left_angle = first_point + left_index if distance_diff[left_index] > threshold else None

    # Find the first index from the right where diff > threshold
    right_index = np.argmax(distance_diff[::-1] > threshold)  # Reverse search
    right_angle = last_point - right_index - 1 if distance_diff[-(right_index + 1)] > threshold else None

    return left_angle, right_angle

# Example usage:
frame_index = 0  # Select a frame to analyze
thresh = 0.5  # Set a threshold for significant change

left_edge, right_edge = find_obstacle_edges(frame_index, threshold=thresh)
print(f"Left Edge Angle: {left_edge}, Right Edge Angle: {right_edge}")

