import math
import yaml
# from controller import ClosedLoopController


def normalize_angle(angle_deg: float) -> float:
    """
    Normalize an angle to the range [-180, 180) degrees.
    """
    # First wrap to [0, 360)
    wrapped = angle_deg % 360
    # Shift to [-180, 180)
    if wrapped >= 180:
        wrapped -= 360
    return wrapped


class UserController:
    def __init__(self, angle_step=0.02, speed_step=0.002):
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
        # self.closed_loop_controller = ClosedLoopController(self)
        self.turn_radius = config["cart"].get("turn_radius", 0.2)
        self.overlap = config["cart"].get("overlap", 0.2)

    def set_world_space(self, rect):
        self.world_rect = rect

    def set_initial_pose(self, pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]

    def update_pose(self, dt):
        if self.speed != 0:
            turning_radius = self.wheel_base / \
                math.tan(math.radians(self.wheel_angle)
                         ) if self.wheel_angle != 0 else float('inf')
            angular_velocity = self.speed / \
                turning_radius if turning_radius != float('inf') else 0
            self.angle += math.degrees(angular_velocity * dt)
            self.angle = normalize_angle(self.angle)
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

    def pure_pursuit(self, lookahead=1, wheelbase=0.5):
        """
        Uses self.cart.job_path for path following.
        """
        job_path = self.job_path
        if not job_path:
            return 0.0  # No path, no steering

        x, y, theta_deg = self.get_pose()

        theta = math.radians(normalize_angle(theta_deg))

        # Find the lookahead point
        # (next point on the path at least lookahead distance away)
        path_xy = [(px, py) for px, py, _ in job_path]

        for px, py in path_xy:
            dist = math.hypot(px - x, py - y)
            if dist >= lookahead:
                target_x, target_y = px, py
                break
        else:
            target_x, target_y = path_xy[-1]

        # Transform target point to vehicle coordinates
        dx = target_x - x
        dy = target_y - y
        local_x = math.cos(-theta)*dx - math.sin(-theta)*dy
        local_y = math.sin(-theta)*dx + math.cos(-theta)*dy

        if abs(local_y) < 1e-6:
            return 0.0

        radius = (local_x**2 + local_y**2) / (2*local_y)
        steering = math.atan2(wheelbase, radius)

        print(
            f"pose | theta={theta_deg:.2f}° \t| Steering: {steering:.2f} "
            f"\t| Target: x={target_x:.2f} \t y={target_y:.2f}"
        )
        return steering

    def run_controller_step(self):

        wheel_angle_rad = self.pure_pursuit()
        self.wheel_angle = max(-45, min(45, math.degrees(wheel_angle_rad)))
        self.update_pose(dt=1.0)
