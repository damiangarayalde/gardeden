import math
import numpy as np
import yaml
from lib.path import get_job_path, dubins_segment, angle_wrap

WALKED_PATH_MIN_STEP = 0.01  # m, minimum movement before a new walked_path point is recorded
PURSUIT_SEARCH_DISTANCE = 1.0  # m of path ahead of the last matched waypoint to search
# m, a waypoint counts as cut once the blade center passes this close; keep it above the
# controller's worst tracking error (~0.1 m) or normal driving leaves false "uncut" gaps
COVERAGE_TOLERANCE = 0.1


def normalize_angle(angle_deg: float) -> float:
    """Normalize an angle to the range [-180, 180) degrees."""
    wrapped = angle_deg % 360
    if wrapped >= 180:
        wrapped -= 360
    return wrapped


class RobotSpecs:
    """Static configuration parameters of the robot (do not change over time)."""

    def __init__(self, config: dict):
        self.rect = config["rect"]
        self.distance_between_wheels = config["distance_between_wheels"]
        self.blade_pos = tuple(config["blade_pos"])
        self.blade_diameter = config["blade_diameter"]
        self.lateral_disturbance = config["lateral_disturbance"]
        self.speed_step = config["speed_step"]              # m/s per keypress
        self.angle_step = config["angle_step"]              # rad/s per keypress
        self.max_speed = config["max_speed"]                # m/s at full joystick
        self.max_angular_speed = config["max_angular_speed"]  # rad/s at full joystick
        self.max_lateral_accel = config["max_lateral_accel"]  # m/s², auto mode slows for curves
        self.turn_radius = config.get("turn_radius", 0.2)
        self.overlap = config.get("overlap", 0.2)


class RobotState:
    """Time-varying state of the robot (changes as it moves)."""

    def __init__(self, initial_pose, config: dict):
        self.x, self.y, self.heading_deg = initial_pose   # (m, m, deg)
        self.speed = 0.0              # current linear speed (m/s)
        self.target_speed = 0.0       # desired linear speed (m/s)
        self.angular_speed = 0.0      # yaw rate (rad/s)
        self.v_left = 0.0             # left wheel linear speed (m/s)
        self.v_right = 0.0            # right wheel linear speed (m/s)
        self.walked_path = [(self.x, self.y)]
        self.paused = config["paused_at_beginning"]
        self.tracking_enabled = config["tracking_enabled"]
        self.horn = config["horn"]


class Robot:
    """Main robot model, including specs, state, and control logic."""

    def __init__(self, initial_config="config.yml"):
        # Load config
        with open(initial_config, "r") as f:
            config = yaml.safe_load(f)

        # Separate static and dynamic parts
        self.specs = RobotSpecs(config["robot"])
        # x,y, angle in degrees
        self.state = RobotState(initial_pose=(
            self.specs.blade_diameter/2, self.specs.blade_diameter/2, 0), config=config["robot"])
        self.world_rect = config["world"]["rect"]

        self.set_job_path(get_job_path(
            self, safety_buffer=self.specs.turn_radius + 0.2))

    # -------------------------------------------------------------------------
    # Retrievers for robot parameters needed by other modules
    # -------------------------------------------------------------------------

    def get_walked_path(self):
        return self.state.walked_path

    def get_blade_diameter(self):
        return self.specs.blade_diameter

    # -------------------------------------------------------------------------
    # State management
    # -------------------------------------------------------------------------
    def set_pose(self, pose):
        """Update the robot's current pose."""
        self.state.x, self.state.y, self.state.heading_deg = pose
        self.state.heading_deg = normalize_angle(self.state.heading_deg)
        # Only record real movement, otherwise the path (and redraw cost) grows every frame
        last_x, last_y = self.state.walked_path[-1]
        if math.hypot(self.state.x - last_x, self.state.y - last_y) >= WALKED_PATH_MIN_STEP:
            self.state.walked_path.append((self.state.x, self.state.y))
        self._mark_covered()

    def get_pose(self):
        """Return the current pose (x, y, heading_deg)."""
        return self.state.x, self.state.y, self.state.heading_deg

    def estimate_next_pose(self, dt: float):
        """Predict next pose after dt seconds, given current speed and wheel speeds."""
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
        self.state.paused = not self.state.paused
        print(f"Paused: {self.state.paused}")
        if self.state.paused:
            self.stop()

    def toggle_tracking(self):
        self.state.tracking_enabled = not self.state.tracking_enabled
        print(f"Tracking active: {self.state.tracking_enabled}")
        self.stop()
        if self.state.tracking_enabled:
            self._resume_job()

    def stop(self):
        """Zero all motion. Heading is integrated from the wheel speeds, so those must stop too."""
        self.state.speed = 0.0
        self.state.target_speed = 0.0
        self.state.angular_speed = 0.0
        self._update_wheel_speeds(0.0)

    def change_speed(self, delta: float):
        """Increase/decrease target speed by delta."""
        if not self.state.paused:
            self._set_target_speed(self.state.target_speed + delta)

    def set_speed_percent(self, percent: float):
        """Set target speed from joystick fraction [-1, 1] of max speed."""
        if not self.state.paused:
            self._set_target_speed(percent * self.specs.max_speed)

    def _set_target_speed(self, speed: float):
        # Pure pursuit only steers correctly going forward, so no reversing in auto mode
        if self.state.tracking_enabled:
            speed = max(0.0, speed)
        self.state.target_speed = speed
        self.state.speed = speed

    def set_angular_rate(self, omega_percent: float):
        """Set angular velocity from joystick fraction [-1, 1] of max yaw rate."""
        if not self.state.paused:
            self.state.angular_speed = omega_percent * self.specs.max_angular_speed
            self._update_wheel_speeds(self.state.angular_speed)

    def apply_lateral_disturbance(self, displacement: float):
        """Simulate a sideways push relative to the robot's heading (positive = to its left)."""
        heading_rad = math.radians(self.state.heading_deg)
        self.state.x -= displacement * math.sin(heading_rad)
        self.state.y += displacement * math.cos(heading_rad)

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
          {"speed_percent": 120, "omega_percent": -20, "toggle_pause": 1, "faster": 1}
        """

        # Analog inputs
        if "speed_percent" in inputs or "omega_percent" in inputs:
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
        if inputs.get("horn_pressed"):
            self.state.horn = True
        if inputs.get("horn_released"):
            self.state.horn = False

    def _apply_analog(self, inputs: dict):
        """Handle analog joystick-style inputs."""
        # Normalize to [-1,1]
        sp_norm = inputs.get("speed_percent", 0) / 255.0
        omega_norm = inputs.get("omega_percent", 0) / 255.0

        # Apply to robot
        self.set_speed_percent(sp_norm)
        self.set_angular_rate(omega_norm)

    # -------------------------------------------------------------------------
    # Path following (pure pursuit)
    # -------------------------------------------------------------------------

    def set_job_path(self, job_path):
        """Set the desired path for the robot to follow."""
        self.job_path = job_path
        self._path_xy = np.array([(px, py) for px, py, *_ in job_path]).reshape(-1, 2)
        self.covered = np.zeros(len(job_path), dtype=bool)  # per waypoint: grass already cut
        self.given_up = np.zeros(len(job_path), dtype=bool)  # uncut, but unreachable: stop retrying
        # Only straight stretches (lanes, perimeter sides) must be cut. Turn arcs just link
        # them: they bulge past COVERAGE_TOLERANCE of what the blade really cuts, so going
        # back for "missed" arc waypoints would be pure wasted travel
        theta = np.array([th for _, _, th, *_ in job_path])
        straight = np.abs(angle_wrap(np.diff(theta))) < 1e-6
        self.required = np.r_[straight, False] | np.r_[False, straight]
        self._last_auto_resume = None
        # Waypoint spacing → how many waypoints make up PURSUIT_SEARCH_DISTANCE of path
        steps = np.hypot(*np.diff(self._path_xy, axis=0).T)
        self._path_step = float(np.median(steps)) if steps.size else 0.01
        self._search_window = max(2, math.ceil(PURSUIT_SEARCH_DISTANCE / self._path_step))
        self.current_path_index = 0
        self.approach_path = None   # transit path to the resume waypoint, (N, 2) array
        self._approach_xy = None    # approach_path + the job path after it (pursuit target)
        self._approach_index = 0

    def _blade_center(self):
        heading_rad = math.radians(self.state.heading_deg)
        bx, by = self.specs.blade_pos
        return (self.state.x + bx * math.cos(heading_rad) - by * math.sin(heading_rad),
                self.state.y + bx * math.sin(heading_rad) + by * math.cos(heading_rad))

    def _mark_covered(self):
        """Mark every waypoint under the blade as cut (manual or auto, it's grass either way)."""
        d = np.hypot(*(self._path_xy - self._blade_center()).T)
        self.covered |= d <= COVERAGE_TOLERANCE

    def first_uncovered_index(self):
        """Index of the earliest waypoint still to cut, or None if the whole job is done."""
        todo = np.flatnonzero(self.required & ~(self.covered | self.given_up))
        return int(todo[0]) if todo.size else None

    def _resume_job(self, auto=False):
        """Continue from the first uncut waypoint, reaching it on a Dubins transit so the robot
        arrives aligned with the lane instead of joining it partway and skipping grass.
        auto: called by the controller itself (not a user toggle) to go back for missed grass."""
        resume = self.first_uncovered_index()
        if auto and resume is not None and resume == self._last_auto_resume:
            # Came back for this patch and still missed it: give up on the whole uncut run
            # rather than loop forever
            run_end = resume + int(np.argmax(np.append(self.covered[resume:], True)))
            self.given_up[resume:run_end] = True
            print(f"Giving up on waypoints {resume}-{run_end - 1}: unreachable")
            resume = self.first_uncovered_index()
        self._last_auto_resume = resume if auto else None
        if resume is None:
            self._finish_job()
            return
        self.current_path_index = resume
        self.approach_path = None
        rx, ry, rth = self.job_path[resume]
        # Close by, pursuit along the job path joins it fine; a tiny Dubins detour just overshoots
        if math.hypot(rx - self.state.x, ry - self.state.y) > 2 * self.specs.turn_radius:
            q0 = (self.state.x, self.state.y, math.radians(self.state.heading_deg))
            seg = dubins_segment(q0, (rx, ry, rth), self.specs.turn_radius, step=self._path_step)
            self.approach_path = np.array([(px, py) for px, py, _ in seg])
            # Pursue the approach *continued* by the job path: if the approach just ended,
            # the carrot would collapse onto its last point right under the robot near the
            # handoff, and 2*y/Ld^2 would spike the steering (heading flip / bounce)
            self._approach_xy = np.vstack(
                [self.approach_path, self._path_xy[resume:resume + self._search_window]])
            self._approach_index = 0
        print(f"Resuming at waypoint {resume}/{len(self.job_path)}")

    @staticmethod
    def _closest_index(path_xy, point, start, end):
        """Index of the path point closest to point, searching only [start, end)."""
        d = np.hypot(*(path_xy[start:end] - point).T)
        return start + int(np.argmin(d))

    def pure_pursuit(self, base_lookahead: float = 0.2, lookahead_time: float = 0.25):
        """Follow the approach path (if resuming) and then the job path."""
        if not self.job_path:
            return

        # dynamic lookahead that grows with speed (m + s * m/s)
        lookahead = base_lookahead + self.state.speed * lookahead_time

        if self.approach_path is not None:
            self._approach_index = self._pursue(self._approach_xy, self._approach_index, lookahead)
            past_end = self._approach_index - len(self.approach_path)
            if past_end >= 0:  # reached the resume waypoint: carry on along the job path
                self.current_path_index += past_end
                self.approach_path = self._approach_xy = None
            return

        self.current_path_index = self._pursue(self._path_xy, self.current_path_index, lookahead)
        i = self.current_path_index
        if self._should_jump(i):
            # Go for the next uncut stretch: missed patches, or past grass already cut
            self._resume_job(auto=True)

    def _should_jump(self, i):
        """True when following the path from waypoint i would cut nothing new: no uncut
        waypoint left ahead, or at least PURSUIT_SEARCH_DISTANCE of already-cut lane to
        re-drive before the next one (turn arcs don't count, they're just links)."""
        todo_ahead = np.flatnonzero(self.required[i:] & ~(self.covered | self.given_up)[i:])
        if todo_ahead.size == 0:
            return True
        already_cut = (self.required & self.covered)[i:i + todo_ahead[0]].sum()
        return already_cut >= self._search_window

    def _finish_job(self):
        skipped = int(self.given_up.sum())
        print("Job complete" + (f" ({skipped} unreachable waypoints left uncut)" if skipped else ""))
        self.state.tracking_enabled = False
        self.stop()

    def _pursue(self, path_xy, index, lookahead):
        """One pure-pursuit step along path_xy from waypoint index; returns the new index."""
        x, y, theta_deg = self.get_pose()
        theta = math.radians(theta_deg)

        # Closest point, searched only ~1 m of path ahead of the last one: a global search
        # would snap to the neighbouring lane after a push, and progress never goes backwards
        closest_index = self._closest_index(path_xy, (x, y), index, index + self._search_window)

        # find carrot point beyond lookahead
        carrot_x, carrot_y = path_xy[-1]
        for i in range(closest_index + 1, len(path_xy)):
            px, py = path_xy[i]
            if math.hypot(px - x, py - y) >= lookahead:
                carrot_x, carrot_y = px, py
                break

        # transform carrot point to local frame
        dx, dy = carrot_x - x, carrot_y - y
        local_x = math.cos(-theta) * dx - math.sin(-theta) * dy
        local_y = math.sin(-theta) * dx + math.cos(-theta) * dy

        # compute curvature (kappa), capped at the tightest turn the mower can physically make
        Ld = math.hypot(local_x, local_y)
        kappa = (2 * local_y) / (Ld * Ld) if Ld > 1e-6 else 0.0
        max_kappa = 1.0 / self.specs.turn_radius
        kappa = max(-max_kappa, min(max_kappa, kappa))

        # Slow down for curves: the user's speed is a ceiling, cornering v²·kappa is capped
        speed = self.state.target_speed
        if abs(kappa) > 1e-9:
            speed = min(speed, math.sqrt(self.specs.max_lateral_accel / abs(kappa)))
        self.state.speed = speed

        # desired angular velocity
        omega = self.state.speed * kappa
        self._update_wheel_speeds(omega)
        return closest_index

    def run_controller(self):
        """Update the robot state based on sensors and user input."""
        # run the path following controller
        if self.state.tracking_enabled:
            self.pure_pursuit()
