import math


def get_job_path(rect, blade_ef_diam, overlap, turn_radius):
    x_min, y_min, x_max, y_max = rect
    lane_spacing = blade_ef_diam * (1 - overlap)

    path = []
    y = y_min
    direction = 1  # 1 = east, -1 = west

    while y <= y_max:
        # straight segment
        if direction == 1:
            path.append((x_min, y, 0))  # start pose
            path.append((x_max, y, 0))  # end pose
        else:
            path.append((x_max, y, math.pi))  # start west
            path.append((x_min, y, math.pi))  # end west

        # prepare next lane
        y += lane_spacing
        if y > y_max:
            break

        # add a U-turn arc (simplified: just connect waypoints)
        if direction == 1:
            path.append((x_max, y, math.pi))  # turned back
        else:
            path.append((x_min, y, 0))

        # flip direction
        direction *= -1

    return path


# Example usage
rect = (0, 0, 10, 5)  # 10m wide, 5m tall area
blade_ef_diam = 1.0   # 1 meter wide blade
overlap = 0.2         # 20% overlap
turn_radius = 0.5     # 0.5 m turning radius (not used yet)

path = get_job_path(rect, blade_ef_diam, overlap, turn_radius)

for pose in path:
    print(pose)
