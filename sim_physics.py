import math
import yaml


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to the range [-180, 180) degrees."""
    wrapped = angle_deg % 360
    if wrapped >= 180:
        wrapped -= 360
    return wrapped


class UserController:
    def __init__(self, angle_step=0.2, speed_step=0.002):
        self.angle_step = angle_step
        self.speed_step = speed_step

    def set_params(self, angle_step, speed_step):
        self.angle_step = angle_step
        self.speed_step = speed_step

    def update(self, pygame, cart):
        keys = pygame.key.get_pressed()
        if keys[pygame.K_a]:  # spin left
            cart.y -= self.angle_step
        if keys[pygame.K_d]:  # spin right
            cart.y += self.angle_step
        if keys[pygame.K_w]:
            cart.v_l += self.speed_step
            cart.v_r += self.speed_step
        if keys[pygame.K_s]:
            cart.v_l -= self.speed_step
            cart.v_r -= self.speed_step


class Thing:

    def __init__(self, initial_config="config.yml"):
        # Load config
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
        self.user_controller = UserController(config["cart"]["speed_step"])
        self.job_path = None
        # self.closed_loop_controller = ClosedLoopController(self)
        self.turn_radius = config["cart"].get("turn_radius", 0.2)
        self.overlap = config["cart"].get("overlap", 0.2)
        # NEW: index for path progress
        self.current_path_index = 0
        # Differential drive speeds
        self.v_l = 0.0  # left wheel
        self.v_r = 0.0  # right wheel

    def set_world_space(self, rect):
        self.world_rect = rect

    def set_initial_pose(self, pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]  # degrees

    def update_pose(self, dt):
        """Update robot pose using differential drive model."""
        self.speed = (self.v_r + self.v_l) / 2.0
        self.wheel_angle = (self.v_r - self.v_l) / self.wheel_base

        angle_rad = math.radians(self.angle)
        self.x += self.speed * math.cos(angle_rad) * dt
        self.y += self.speed * math.sin(angle_rad) * dt
        self.angle = normalize_angle(
            self.angle + math.degrees(self.wheel_angle * dt))

        self.walked_path.append((self.x, self.y))
        return self.x, self.y, self.angle

    def get_pose(self):
        return self.x, self.y, self.angle

    def get_walked_path(self):
        return self.walked_path

    def get_blade_diameter(self):
        return self.blade_diameter

    def set_job_path(self, job_path):
        self.job_path = job_path
        self.current_path_index = 0

    def pure_pursuit(self, lookahead=1.0):
        """Pure pursuit for differential drive: outputs left/right wheel speeds."""
        if not self.job_path:
            return 0.0, 0.0

        x, y, theta_deg = self.get_pose()
        theta = math.radians(normalize_angle(theta_deg))
        path_xy = [(px, py) for px, py, _ in self.job_path]

        # Find closest point
        dists = [math.hypot(px - x, py - y) for px, py in path_xy]
        closest_index = int(min(range(len(dists)), key=lambda i: dists[i]))

        # Find carrot point
        carrot_x, carrot_y = path_xy[-1]
        for i in range(closest_index + 1, len(path_xy)):
            px, py = path_xy[i]
            if math.hypot(px - x, py - y) >= lookahead:
                carrot_x, carrot_y = px, py
                self.current_path_index = i
                break

        # Transform carrot to local frame
        dx = carrot_x - x
        dy = carrot_y - y
        local_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        # Simple wheel command logic
        base_speed = 0.01
        k_turn = 0.8

        v_l = base_speed
        v_r = base_speed

        if local_y > 0.1:   # carrot to left
            v_l = self.speed/2*(1 - k_turn)
            v_r = self.speed/2*(1 + k_turn)
        elif local_y < -0.1:  # carrot to right
            v_l = self.speed/2*(1 + k_turn)
            v_r = self.speed/2*(1 - k_turn)

        print(
            f"pose | θ={theta_deg:.1f}° "
            f"| carrot local=({local_x:.2f}, {local_y:.2f}) "
            f"| v_l={v_l:.3f}, v_r={v_r:.3f}"
        )

        return v_l, v_r

    def run_controller_step(self):
        v_l, v_r = self.pure_pursuit()
        self.v_l, self.v_r = v_l, v_r
        self.update_pose(dt=1.0)
