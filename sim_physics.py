import math


# def update_cart_position(x, y, heading, wheel_angle, speed, wheel_base):
#     """
#     Update cart position and heading using a simple bicycle model.
#     Returns new_x, new_y, new_heading.
#     """
#     # Update heading
#     if speed != 0:
#         turning_radius = wheel_base / \
#             math.tan(math.radians(wheel_angle)
#                      ) if wheel_angle != 0 else float('inf')
#         angular_velocity = speed / \
#             turning_radius if turning_radius != float('inf') else 0
#         heading += math.degrees(angular_velocity)
#     # Update position
#     angle_rad = math.radians(heading)
#     dx = speed * math.cos(angle_rad)
#     dy = speed * math.sin(angle_rad)
#     return x + dx, y + dy, heading


class Thing:
    def __init__(self, wheel_base=40):
        self.world_rect = None  # [(x0, y0), (x1, y1), (x2, y2), (x3, y3)]
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0  # degrees
        self.speed = 0.0
        self.wheel_angle = 0.0
        self.wheel_base = wheel_base

    def set_world_space(self, rect):
        """rect: list of 4 vertices [(x0, y0), ..., (x3, y3)]"""
        self.world_rect = rect

    def set_initial_pose(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def update_pose(self, dt=1.0):
        """Update pose using bicycle model. dt is time step."""
        if self.speed != 0:
            turning_radius = self.wheel_base / \
                math.tan(math.radians(self.wheel_angle)
                         ) if self.wheel_angle != 0 else float('inf')
            angular_velocity = self.speed / \
                turning_radius if turning_radius != float('inf') else 0
            self.angle += math.degrees(angular_velocity * dt)
        angle_rad = math.radians(self.angle)
        dx = self.speed * math.cos(angle_rad) * dt
        dy = self.speed * math.sin(angle_rad) * dt
        self.x += dx
        self.y += dy
        return self.x, self.y, self.angle

    def get_pose(self):
        return self.x, self.y, self.angle
