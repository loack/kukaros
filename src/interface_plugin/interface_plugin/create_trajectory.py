import json
import math

def create_circle_trajectory(center, radius, num_points=100, z_height=None):
    """
    Generate a list of [x, y, z] points forming a circle trajectory.
    center: [x, y, z] - center of the circle
    radius: float - radius of the circle
    num_points: int - number of points in the trajectory
    z_height: float or None - if set, overrides center[2] for all points
    """
    trajectory = []
    for i in range(num_points):
        angle = 2 * math.pi * i / num_points
        x = center[0] + radius * math.cos(angle)
        y = center[1] + radius * math.sin(angle)
        z = z_height if z_height is not None else center[2]
        trajectory.append([x, y, z])
    return trajectory

if __name__ == "__main__":
    # Example: center at (0, 0, 0.5), radius 0.2, 100 points
    center = [1, 1, 0.5]
    radius = 0.5
    num_points = 500
    trajectory = create_circle_trajectory(center, radius, num_points)

    # Save to trajectory.json
    with open("./src/trajectory.json", "w") as f:
        json.dump(trajectory, f, indent=2)