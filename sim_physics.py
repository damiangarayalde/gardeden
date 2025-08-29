import math
import yaml
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
        else:
            cart.angle = 0.0


class Thing:

    def __init__(self, initial_config="config.yml"):

        # Load config from YAML
        with open(initial_config, "r") as f:
            config = yaml.safe_load(f)

        self.rect = config["cart"]["rect"]
        self.world_rect = config["world"]["rect"]
        self.set_initial_pose(pose=config["cart"]["initial_pose"])
        self.speed = 0.0
        self.wheel_angle = 0.0
        self.wheel_base = config["cart"]["wheel_base"]
        self.blade_pos = tuple(config["cart"]["blade_pos"])
        self.blade_diameter = config["cart"]["blade_diameter"]
        self.walked_path = []
        self.user_controller = UserController(
            config["cart"]["angle_step"], config["cart"]["speed_step"])
        self.job_path = None
        self.closed_loop_controller = ClosedLoopController(self)
        self.turn_radius = config["cart"].get("turn_radius", 0.2)
        self.overlap = config["cart"].get("overlap", 0.2)

    def set_world_space(self, rect):
        self.world_rect = rect

    def set_initial_pose(self, pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def update_pose(self, dt=1.0):
        if self.speed != 0:
            turning_radius = self.wheel_base / \
                math.tan(self.wheel_angle
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

    def set_job_path(self, job_path):
        self.job_path = job_path
