import math


def pure_pursuit_control(x, y, theta, path, lookahead=0.5, wheelbase=0.5):
    """
    x,y,theta: current cart pose
    path: list of (x,y) waypoints
    lookahead: distance ahead to target (m)
    wheelbase: vehicle wheelbase (m)

    returns: steering angle (rad)
    """
    # Find closest waypoint ahead at lookahead distance
    for px, py in path:
        dist = math.hypot(px - x, py - y)
        if dist >= lookahead:
            target_x, target_y = px, py
            break
    else:
        target_x, target_y = path[-1]

    # Transform target into vehicle coordinates
    dx = target_x - x
    dy = target_y - y
    local_x = math.cos(-theta)*dx - math.sin(-theta)*dy
    local_y = math.sin(-theta)*dx + math.cos(-theta)*dy

    # Pure pursuit steering law
    if abs(local_y) < 1e-6:
        return 0.0
    radius = (local_x**2 + local_y**2) / (2*local_y)
    steering = math.atan2(wheelbase, radius)
    return steering
