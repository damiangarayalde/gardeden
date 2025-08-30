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
            cart.y -= self.angle_step
        if keys[pygame.K_d]:
            cart.y += self.angle_step
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
        # NEW: index for path progress
        self.current_path_index = 0

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
        self.current_path_index = 0  # reset progress when new path is set

    def pure_pursuit(self, lookahead=1.5, wheelbase=0.5):
        """
        Uses self.cart.job_path for path following.
        """
        job_path = self.job_path
        if not job_path:
            return 0.0  # No path, no steering

        x, y, theta_deg = self.get_pose()
        theta = math.radians(normalize_angle(theta_deg))
        path_xy = [(px, py) for px, py, _ in job_path]

        # --- Find closest point to current position ---
        dists = [math.hypot(px - x, py - y) for px, py in path_xy]
        closest_index = int(min(range(len(dists)), key=lambda i: dists[i]))

        # --- Scan forward from closest point to find lookahead target ---
        target_x, target_y = path_xy[-1]  # fallback
        for i in range(closest_index, len(path_xy)):
            px, py = path_xy[i]
            if math.hypot(px - x, py - y) >= lookahead:
                target_x, target_y = px, py
                self.current_path_index = i
                break

        # --- Transform target to local vehicle frame ---
        dx = target_x - x
        dy = target_y - y
        local_x = math.cos(-theta)*dx - math.sin(-theta)*dy
        local_y = math.sin(-theta)*dx + math.cos(-theta)*dy

        if abs(local_y) < 1e-3:
            return 0.0

        # Radius of curvature
        radius = (local_x**2 + local_y**2) / (2*local_y)
        steering = math.atan2(wheelbase, abs(radius))

        # Sign: negative if target is to the left of cart
        if local_x < 0:
            steering = -steering

        print(
            f"pose | theta={theta_deg:.1f}° "
            f"| Steering: {math.degrees(steering):.2f} rad "
            f"| Target: x={target_x:.1f}, Wheel={self.wheel_angle:.2f}, "
            f"index={self.current_path_index}"
        )

        return steering

    def run_controller_step(self):

        wheel_angle_rad = self.pure_pursuit()
        self.wheel_angle = max(-45, min(45, math.degrees(wheel_angle_rad)))
        self.update_pose(dt=1.0)
