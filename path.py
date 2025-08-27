import math
import numpy as np


def arc_points(xc, yc, r, start_angle, end_angle, n=10):
    """Generate n points along a circular arc"""
    angles = np.linspace(start_angle, end_angle, n)
    return [(xc + r*math.cos(a), yc + r*math.sin(a), a) for a in angles]


def get_job_path(rect, blade_ef_diam, overlap, turn_radius, arc_res=10):
    x_min, y_min, x_max, y_max = rect
    lane_spacing = blade_ef_diam * (1 - overlap)

    path = []
    y = y_min
    direction = 1  # 1 = east, -1 = west

    while y <= y_max:
        # Straight row
        if direction == 1:
            path.append((x_min, y, 0))
            path.append((x_max, y, 0))
            x_end = x_max
        else:
            path.append((x_max, y, math.pi))
            path.append((x_min, y, math.pi))
            x_end = x_min

        # Next row
        y_next = y + lane_spacing
        if y_next > y_max:
            break

        # U-turn arc
        if direction == 1:
            # Turning from east-going to west-going
            # Center of arc is to the right of the path
            arc = arc_points(
                x_end, y + turn_radius,
                turn_radius,
                -math.pi/2, math.pi/2,
                arc_res
            )
            # Shift arc to end at the new lane
            arc = [(x, y_next, theta+math.pi) if abs(y_next-y - lane_spacing) < 1e-6 else (x, y_arc, theta)
                   for (x, y_arc, theta) in arc]
            path.extend(arc)

        else:
            # Turning from west-going to east-going
            arc = arc_points(
                x_end, y + turn_radius,
                turn_radius,
                math.pi/2, 3*math.pi/2,
                arc_res
            )
            arc = [(x, y_next, theta) if abs(y_next-y - lane_spacing) < 1e-6 else (x, y_arc, theta)
                   for (x, y_arc, theta) in arc]
            path.extend(arc)

        # Move to next lane
        y = y_next
        direction *= -1

    return path
