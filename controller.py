import math


class ClosedLoopController:
    def __init__(self, lookahead=0.5, wheelbase=0.5):
        self.lookahead = lookahead
        self.wheelbase = wheelbase

    def set_params(self, lookahead=0.5, wheelbase=0.5):
        self.lookahead = lookahead
        self.wheelbase = wheelbase

    def pure_pursuit(self, cart_pose, path):
        """
        cart_pose: (x, y, theta in degrees)
        path: list of (x, y) waypoints
        returns: steering angle (rad)
        """
        x, y, theta_deg = cart_pose
        theta = math.radians(theta_deg)
        # Find closest waypoint ahead at lookahead distance
        for px, py in path:
            dist = math.hypot(px - x, py - y)
            if dist >= self.lookahead:
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
        steering = math.atan2(self.wheelbase, radius)
        return steering
