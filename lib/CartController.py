import math
import yaml


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to the range [-180, 180) degrees."""
    wrapped = angle_deg % 360
    if wrapped >= 180:
        wrapped -= 360
    return wrapped


class CartSpecs:
    """Static configuration parameters of the cart (do not change over time)."""

    def __init__(self, config: dict):
        self.rect = config["rect"]
        self.distance_between_wheels = config["distance_between_wheels"]
        self.blade_pos = tuple(config["blade_pos"])
        self.blade_diameter = config["blade_diameter"]
        self.lateral_disturbance = config["lateral_disturbance"]
        self.speed_step = config["speed_step"]
        self.angle_step = config["angle_step"]
        self.turn_radius = config.get("turn_radius", 0.2)
        self.overlap = config.get("overlap", 0.2)


class CartState:
    """Time-varying state of the cart (changes as it moves)."""

    def __init__(self, initial_pose):
        self.x, self.y, self.heading_deg = initial_pose   # (m, m, deg)
        self.speed = 0.0              # current linear speed
        self.target_speed = 0.0       # desired linear speed
        self.angular_speed = 0.0      # yaw rate (rad/s)
        self.v_left = 0.0             # left wheel linear speed
        self.v_right = 0.0            # right wheel linear speed
        self.walked_path = [(self.x, self.y)]


class Cart:
    """Main cart model, including specs, state, and control logic."""

    def __init__(self, initial_config="config.yml"):
        # Load config
        with open(initial_config, "r") as f:
            config = yaml.safe_load(f)

        # Separate static and dynamic parts
        self.specs = CartSpecs(config["cart"])
        self.state = CartState(initial_pose=(0, 0, 0))  # x,y, angle in degrees
        self.world_rect = config["world"]["rect"]

        # Task/path-following parameters
        self.job_path = None
        self.current_path_index = 0

        # Flags
        self.paused = False
        self.tracking_enabled = False

    # -------------------------------------------------------------------------
    # Retrievers for cart parameters needed by other modules
    # -------------------------------------------------------------------------

    def get_walked_path(self):
        return self.state.walked_path

    def get_blade_diameter(self):
        return self.specs.blade_diameter

    # -------------------------------------------------------------------------
    # State management
    # -------------------------------------------------------------------------
    def set_pose(self, pose):
        """Update the cart's current pose."""
        self.state.x, self.state.y, self.state.heading_deg = pose
        self.state.heading_deg = normalize_angle(self.state.heading_deg)
        self.state.walked_path.append((self.state.x, self.state.y))

    def get_pose(self):
        """Return the current pose (x, y, heading_deg)."""
        return self.state.x, self.state.y, self.state.heading_deg

    def estimate_next_pose(self, dt: float):
        """Predict next pose given current speed and angular speed."""
        omega = (self.state.v_right - self.state.v_left) / \
            self.specs.distance_between_wheels

        angle_rad = math.radians(self.state.heading_deg)

        x = self.state.x + self.state.speed * math.cos(angle_rad) * dt
        y = self.state.y + self.state.speed * math.sin(angle_rad) * dt

        heading = normalize_angle(
            self.state.heading_deg + math.degrees(omega * dt))

        return x, y, heading

    # -------------------------------------------------------------------------
    # Control / inputs
    # -------------------------------------------------------------------------
    def toggle_pause(self):
        self.paused = not self.paused
        print(f"Paused: {self.paused}")
        if self.paused:
            self.state.speed = 0.0

    def toggle_tracking(self):
        self.tracking_enabled = not self.tracking_enabled
        print(f"Tracking active: {self.tracking_enabled}")
        self.state.speed = 0.0
        self.state.target_speed = 0.0
        self.state.angular_speed = 0.0

    def change_speed(self, delta: float):
        """Increase/decrease target speed by delta."""
        if not self.paused:
            self.state.target_speed += delta
            self.state.speed = self.state.target_speed

    def set_speed_percent(self, percent: float):
        """Set target speed based on joystick percentage of max speed."""
        if not self.paused:
            max_speed = self.specs.speed_step * 2  # example: scale step into max
            self.state.target_speed = percent * max_speed
            self.state.speed = self.state.target_speed

    def set_angular_rate(self, omega: float):
        """Set angular velocity directly (rad/s)."""
        if not self.paused:
            self.state.angular_speed = omega
            self._update_wheel_speeds(self.state.angular_speed)

    # def set_world_space(self, rect):
    #     self.world_rect = rect

    def apply_lateral_disturbance(self, displacement: float):
        """Simulate a lateral displacement disturbance."""
        self.state.y += displacement

    def adjust_direction(self, angle_step: float):
        """Change angular speed by adjusting heading rate."""
        self.state.angular_speed += angle_step
        self._update_wheel_speeds(self.state.angular_speed)

    def _update_wheel_speeds(self, omega: float):
        """Convert linear + angular velocity to differential wheel speeds."""
        d = self.specs.distance_between_wheels
        self.state.v_left = self.state.speed - 0.5 * d * omega
        self.state.v_right = self.state.speed + 0.5 * d * omega

    def apply_inputs(self, inputs: dict):
        """
        Apply key/value inputs coming from joystick or keyboard.
        Example inputs:
          {"x": 120, "y": -20, "pause": 1, "faster": 1}
        """

        # Analog inputs
        if "speed_percent" in inputs or "omega" in inputs:
            self._apply_analog(inputs)

        # Discrete keyboard-like commands
        if inputs.get("turn_left"):
            self.adjust_direction(+self.specs.angle_step)
        if inputs.get("turn_right"):
            self.adjust_direction(-self.specs.angle_step)
        if inputs.get("toggle_pause"):
            self.toggle_pause()
        if inputs.get("toggle_tracking"):
            self.toggle_tracking()
        if inputs.get("faster"):
            self.change_speed(+self.specs.speed_step)
        if inputs.get("slower"):
            self.change_speed(-self.specs.speed_step)
        if inputs.get("disturb_left"):
            self.apply_lateral_disturbance(+self.specs.lateral_disturbance)
        if inputs.get("disturb_right"):
            self.apply_lateral_disturbance(-self.specs.lateral_disturbance)

    def _apply_analog(self, inputs: dict):
        """Handle analog joystick-style inputs."""
        # Normalize to [-1,1]
        sp_norm = inputs.get("speed_percent", 0) / 255.0
        omega_norm = inputs.get("omega", 0) / 255.0

        # Apply to cart
        self.set_speed_percent(sp_norm)
        max_turn_rate = 0.04  # rad/s, tune
        self.set_angular_rate(omega_norm * max_turn_rate)

    # -------------------------------------------------------------------------
    # Path following (pure pursuit)
    # -------------------------------------------------------------------------

    def set_job_path(self, job_path):
        """Set the desired path for the cart to follow."""
        self.job_path = job_path
        self.current_path_index = 0

    def pure_pursuit(self, base_lookahead: float = 0.2):
        """Run pure pursuit controller to follow the job path."""
        if not self.job_path:
            return

        # dynamic lookahead that grows with speed
        lookahead = base_lookahead + self.state.speed * 0.5

        x, y, theta_deg = self.get_pose()
        theta = math.radians(theta_deg)
        path_xy = [(px, py) for px, py, _ in self.job_path]

        # find closest path point
        dists = [math.hypot(px - x, py - y) for px, py in path_xy]
        closest_index = int(min(range(len(dists)), key=lambda i: dists[i]))

        # find carrot point beyond lookahead
        carrot_x, carrot_y = path_xy[-1]
        for i in range(closest_index + 1, len(path_xy)):
            px, py = path_xy[i]
            if math.hypot(px - x, py - y) >= lookahead:
                carrot_x, carrot_y = px, py
                self.current_path_index = i
                break

        # transform carrot point to local frame
        dx, dy = carrot_x - x, carrot_y - y
        local_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        # compute curvature (kappa)
        Ld = math.hypot(local_x, local_y)
        kappa = (2 * local_y) / (Ld * Ld) if Ld > 1e-6 else 0.0

        # desired angular velocity
        omega = self.state.speed * kappa
        self._update_wheel_speeds(omega)

    def run_controller(self):
        """Update the cart state based on sensors and user input."""

        print(f'speed: {100*self.state.speed:.2f} m/s, target speed: {100*self.state.target_speed:.2f} m/s, angular speed: {100*self.state.angular_speed:.2f} rad/s')

        # run the path following controller
        if self.tracking_enabled:
            self.pure_pursuit()
