import math
from controller import ClosedLoopController


class UserController:
    def __init__(self, angle_step=20, speed_step=0.002):
        self.angle_step = angle_step
        self.speed_step = speed_step

    def set_params(self, angle_step, speed_step):
        self.angle_step = angle_step
        self.speed_step = speed_step

    def update(self, pygame, cart):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_a]:
            cart.angle -= self.angle_step
        if keys[pygame.K_d]:
            cart.angle += self.angle_step
        if keys[pygame.K_w]:
            cart.speed += self.speed_step
        if keys[pygame.K_s]:
            cart.speed -= self.speed_step


class Thing:
    def __init__(self, wheel_base=40, blade_diameter=1, angle_step=20, speed_step=0.002):
        self.world_rect = None
        self.x = 0.0
        self.y = 0.0
        self.angle = 0.0  # degrees
        self.speed = 0.0
        self.wheel_angle = 0.0
        self.wheel_base = wheel_base
        self.blade_diameter = blade_diameter
        self.walked_path = []
        self.user_controller = UserController(angle_step, speed_step)
        self.closed_loop_controller = ClosedLoopController(
            lookahead=0.5, wheelbase=self.wheel_base)

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
