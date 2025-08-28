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
    def __init__(self, wheel_base=40, blade_diameter=1):
        self.world_rect = None
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0  # degrees
        self.speed = 0.0
        self.wheel_angle = 0.0
        self.wheel_base = wheel_base
        self.blade_diameter = blade_diameter
        self.walked_path = []

    def set_world_space(self, rect):
        self.world_rect = rect

    def set_initial_pose(self, x, y, angle):
        self.x = x
        self.y = y
        self.angle = angle

    def update_pose(self, dt=1.0):
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
        # Store walked path
        self.walked_path.append((self.x, self.y))
        return self.x, self.y, self.angle

    def get_pose(self):
        return self.x, self.y, self.angle

    def get_walked_path(self):
        return self.walked_path

    def get_blade_diameter(self):
        return self.blade_diameter

    def get_wheel_angle(self):
        return self.wheel_angle
