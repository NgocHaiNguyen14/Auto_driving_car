import numpy as np

def lane_value_function(num_lanes, dist, target_lane, current_lane, weight_obstacle=1.0, weight_target=1.0, weight_pen=2.0, weight_step=10.0, weight_stay=3.0, threshold=3):
    """
    Compute lane values focusing only on the current lane and the two adjacent lanes.

    Parameters:
    - num_lanes (int): Number of lanes.
    - dist (list): List of distances to obstacles for each lane.
    - target_lane (int): The preferred target lane (0-indexed).
    - current_lane (int): The current lane (0-indexed).
    - weight_obstacle (float): Weight for obstacle avoidance.
    - weight_target (float): Weight for preferring the target lane.
    - weight_pen (float): Penalty for moving further from the target lane.
    - weight_step (float): Bonus for moving 1 step closer to target.
    - weight_stay (float): Bonus for staying in the target lane.
    - threshold (float): Minimum normalized distance to consider staying in the target lane.

    Returns:
    - values (list): List of computed values for each lane.
    """
    lanes = np.array([current_lane])  # Start with the current lane
    if current_lane > 0:
        lanes = np.append(lanes, current_lane - 1)  # Add left lane if available
    if current_lane < num_lanes - 1:
        lanes = np.append(lanes, current_lane + 1)  # Add right lane if available

    # Initialize values (default: -inf for lanes we don't consider)
    values = np.full(num_lanes, -np.inf)

    for lane in lanes:
        # Obstacle avoidance (larger distance to obstacle, higher value)
        obstacle_value = weight_obstacle * (dist[lane] ** 2)

        # Target lane preference (higher for closer lanes to target)
        target_value = weight_target * ((num_lanes - 1 - np.abs(lane - target_lane)) ** 2)

        # Penalty for moving away from the target lane
        penalty = weight_pen * max(0, np.abs(lane - target_lane) - np.abs(current_lane - target_lane))

        step_bonus = 0
        if np.abs(lane - target_lane) < np.abs(current_lane - target_lane) and dist[lane] > threshold:
            # **Activate step_bonus when distance to obstacle is larger than threshold**
            step_bonus += weight_step  # Add bonus if lane distance to obstacle is larger than threshold
            print(step_bonus)


        # Bonus for staying in the target lane if obstacle is far enough
        stay_bonus = weight_stay * (num_lanes ** 2) if lane == target_lane else 0

        # Compute total lane value
        values[lane] = obstacle_value + target_value - penalty + step_bonus + stay_bonus

    return values.tolist()



num_lanes = 5
dist = [5, 4, 10, 2, 1]  # Distances to obstacles
target_lane = 3
current_lane = 0

values = lane_value_function(num_lanes, dist, target_lane, current_lane)
print(values)

