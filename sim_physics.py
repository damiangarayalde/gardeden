import math
import yaml


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to the range [-180, 180) degrees."""
    wrapped = angle_deg % 360
    if wrapped >= 180:
        wrapped -= 360
    return wrapped


class Thing:

    def __init__(self, initial_config="config.yml"):
        # Load config
        with open(initial_config, "r") as f:
            config = yaml.safe_load(f)

        self.paused = False
        self.rect = config["cart"]["rect"]
        self.world_rect = config["world"]["rect"]
        self.walked_path = []

        self.set_pose(pose=config["cart"]["initial_pose"])
        self.speed = 0.0
        self.target_speed = 0.0
        self.distance_between_wheels = config["cart"]["distance_between_wheels"]
        self.blade_pos = tuple(config["cart"]["blade_pos"])
        self.blade_diameter = config["cart"]["blade_diameter"]
        self.angle_step = config["cart"]["angle_step"]
        self.speed_step = config["cart"]["speed_step"]
        self.job_path = None
        self.turn_radius = config["cart"].get("turn_radius", 0.2)
        self.overlap = config["cart"].get("overlap", 0.2)
        self.current_path_index = 0
        # Differential drive speeds
        self.v_l = 0.0  # left wheel
        self.v_r = 0.0  # right wheel

    def set_world_space(self, rect):
        self.world_rect = rect

    def toggle_pause(self):
        self.paused = not self.paused
        self.update_speed()

    def update_speed(self):
        self.speed = 0.0 if self.paused else self.target_speed

    def set_pose(self, pose):
        self.x = pose[0]
        self.y = pose[1]
        self.angle = pose[2]  # degrees
        self.walked_path.append((self.x, self.y))

    def estimate_next_pose(self, dt):
        """Provides the next estimated pose based on current config."""
        omega = (self.v_r - self.v_l) / self.distance_between_wheels

        angle_rad = math.radians(self.angle)
        x = self.x + self.speed * math.cos(angle_rad) * dt
        y = self.y + self.speed * math.sin(angle_rad) * dt
        angle = normalize_angle(
            self.angle + math.degrees(omega * dt))

        return x, y, angle

    def get_pose(self):
        return self.x, self.y, self.angle

    def get_walked_path(self):
        return self.walked_path

    def get_blade_diameter(self):
        return self.blade_diameter

    def set_job_path(self, job_path):
        self.job_path = job_path
        self.current_path_index = 0

    def pure_pursuit(self, lookahead=0.2):
        """Pure pursuit controller for differential drive."""
        if not self.job_path:
            return 0.0, 0.0

        x, y, theta_deg = self.get_pose()
        theta = math.radians(normalize_angle(theta_deg))
        path_xy = [(px, py) for px, py, _ in self.job_path]

        # --- Find closest path point ---
        dists = [math.hypot(px - x, py - y) for px, py in path_xy]
        closest_index = int(min(range(len(dists)), key=lambda i: dists[i]))

        # --- Find carrot point (first point beyond lookahead) ---
        carrot_x, carrot_y = path_xy[-1]
        for i in range(closest_index + 1, len(path_xy)):
            px, py = path_xy[i]
            if math.hypot(px - x, py - y) >= lookahead:
                carrot_x, carrot_y = px, py
                self.current_path_index = i
                break

        # --- Transform carrot point to local frame ---
        dx = carrot_x - x
        dy = carrot_y - y
        local_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        # --- Compute curvature (kappa) ---
        Ld = math.hypot(local_x, local_y)
        if Ld < 1e-6:
            kappa = 0.0
        else:
            kappa = (2 * local_y) / (Ld * Ld)

        # --- Desired angular velocity ---
        omega = self.speed * kappa

        # --- Convert to wheel speeds (differential drive) ---
        self.v_l = self.speed - 0.5 * self.distance_between_wheels * omega
        self.v_r = self.speed + 0.5 * self.distance_between_wheels * omega

        print(
            f"pose | θ={theta_deg:.1f}° "
            f"| carrot local=({local_x:.2f}, {local_y:.2f}) "
            f"| kappa={kappa:.3f}, ω={omega:.3f} "
            f"| v_l={self.v_l:.3f}, v_r={self.v_r:.3f}"
        )

    def run_inputs_bindings(self, input):

        if input:
            if input == "left":
                self.y -= self.angle_step
            elif input == "right":
                self.y += self.angle_step
            elif input == "move_faster":
                self.target_speed += self.speed_step
            elif input == "move_slower":
                self.target_speed -= self.speed_step
            elif input == "toggle_pause":
                self.toggle_pause()

    def run_controller(self, user_inputs, measured_cart_pose=None):

        # update cart model pose based on observer data
        self.set_pose(measured_cart_pose)

        # update the cart config based on user inputs
        self.run_inputs_bindings(user_inputs)

        self.update_speed()
        # run the path following controller
        self.pure_pursuit()
