import pygame
import math
import numpy as np
import yaml
import time


class Drawer:
    def __init__(self, initial_config, robot, camera_mode="camera"):

        pygame.display.set_caption("robot Simulation")

        # Load config from YAML
        with open(initial_config, "r") as f:
            config = yaml.safe_load(f)

        # preserve config aspect ratio but maximize to fit the current display
        cfg_w = config["graphics"]["width"]
        cfg_h = config["graphics"]["height"]
        info = pygame.display.Info()
        disp_w, disp_h = info.current_w, info.current_h
        scale = min(disp_w / cfg_w, disp_h / cfg_h)
        self.window_size = (max(1, int(cfg_w * scale)),
                            max(1, int(cfg_h * scale)))
        self.scale = 1
        self.mode = camera_mode
        self.offset = (0, 0)
        self.font = pygame.font.SysFont("Arial", 14)  # SysFont lookup is slow, do it once
        self._job_path_cache = {}
        self.screen = pygame.display.set_mode(
            self.window_size, pygame.RESIZABLE)
        self.robot = robot
        self.set_world_space(robot.world_rect)

        self.set_robot_size(rect=robot.specs.rect,
                            blade_pos=robot.specs.blade_pos,
                            blade_diameter=robot.get_blade_diameter())

    def set_world_space(self, rect):
        """Map rect to 80% of the current drawing surface, keep aspect ratio."""
        self.world_rect = rect
        wx, wy = self.screen.get_size()  # follows window resizes and the split-screen half
        rect_w = abs(rect[2] - rect[0])
        rect_h = abs(rect[3] - rect[1])
        scale_w = (wx * 0.8) / rect_w
        scale_h = (wy * 0.8) / rect_h
        self.scale = min(scale_w, scale_h)
        # Center the mapped area
        mapped_w = rect_w * self.scale
        mapped_h = rect_h * self.scale
        self.offset = ((wx - mapped_w) / 2 - rect[0] * self.scale,
                       (wy - mapped_h) / 2 - rect[1] * self.scale)

    def set_robot_size(self, rect, blade_pos, blade_diameter):
        """
        Configure robot size and blade parameters.
        rect: bounding box for robot in local coordinates (x along heading), meters
        blade_pos: (x, y) offset from robot center
        blade_diameter: effective blade diameter in meters
        """
        self.robot_length = rect[2] - rect[0]
        self.robot_width = rect[3] - rect[1]
        self.blade_pos = blade_pos
        self.blade_diameter = blade_diameter

    def draw_debug_rects(self, rect, margin_x=0.0, margin_y=0.0, safety_buffer=0.0):
        """
        Draw the base rect, the expanded rect (rect ± margin),
        and the inset rect (after safety buffer).
        """

        x_min, y_min, x_max, y_max = rect

        # Expand rect by margin
        x_min_ext = x_min - margin_x
        x_max_ext = x_max + margin_x
        y_min_ext = y_min - margin_y
        y_max_ext = y_max + margin_y

        # Inset by safety buffer
        x_min_in = x_min_ext + safety_buffer
        x_max_in = x_max_ext - safety_buffer
        y_min_in = y_min_ext + safety_buffer
        y_max_in = y_max_ext - safety_buffer

        # Convert to screen coords
        def rect_to_screen(x0, y0, x1, y1):
            p0 = self.map_to_screen(x0, y0)
            p1 = self.map_to_screen(x1, y1)
            return pygame.Rect(min(p0[0], p1[0]), min(p0[1], p1[1]),
                               abs(p1[0] - p0[0]), abs(p1[1] - p0[1]))

        base_rect = rect_to_screen(x_min, y_min, x_max, y_max)
        ext_rect = rect_to_screen(x_min_ext, y_min_ext, x_max_ext, y_max_ext)
        in_rect = rect_to_screen(x_min_in, y_min_in, x_max_in, y_max_in)

        # Draw rectangles with different colors
        pygame.draw.rect(self.screen, (30, 70, 30), base_rect,
                         0)      # green: original rect
        # pygame.draw.rect(self.screen, (150, 150, 0), ext_rect,
        #                  2)     # yellow: allowed outer rect
        # pygame.draw.rect(self.screen, (0, 0, 200), in_rect,
        #                  2)        # blue: inset rect

    def map_to_screen(self, x, y):
        """Map local coordinates to screen coordinates using scale and offset."""
        sx = x * self.scale + self.offset[0]
        sy = y * self.scale + self.offset[1]
        return sx, sy

    def world_to_display(self, pt, mode="zenital", observer=None):
        """Map a world point to screen coords, top-down or projected through the observer."""
        if mode == "zenital":
            return self.map_to_screen(pt[0], pt[1])
        u_img, v_img = observer.world_to_image(pt)  # world → image
        screen_w, screen_h = self.screen.get_size()
        img_w, img_h = observer.frame_size
        return (int(u_img * screen_w / img_w),      # image → screen
                int(v_img * screen_h / img_h))

    def world_to_display_many(self, points, mode="zenital", observer=None):
        """Vectorized world_to_display: (N, 2) world points → (N, 2) float screen coords.
        Long paths go through here: one numpy call instead of N Python-level projections."""
        pts = np.asarray(points, dtype=float).reshape(-1, 2)
        if mode == "zenital":
            return pts * self.scale + self.offset
        screen_w, screen_h = self.screen.get_size()
        img_w, img_h = observer.frame_size
        return observer.world_to_image_many(pts) * (screen_w / img_w, screen_h / img_h)

    def world_radius_to_px(self, center, radius, mode="zenital", observer=None):
        """On-screen radius of a world circle; in camera mode, averages perspective along x and y."""
        return int(self.world_radii_to_px([center], radius, mode, observer)[0])

    def world_radii_to_px(self, centers, radius, mode="zenital", observer=None):
        """Vectorized world_radius_to_px for (N, 2) centers → (N,) int pixel radii."""
        centers = np.asarray(centers, dtype=float).reshape(-1, 2)
        if mode == "zenital":
            return np.full(len(centers), max(1, int(radius * self.scale)))
        c = self.world_to_display_many(centers, mode, observer)
        rx = self.world_to_display_many(centers + (radius, 0.0), mode, observer)
        ry = self.world_to_display_many(centers + (0.0, radius), mode, observer)
        r_px = (np.hypot(*(rx - c).T) + np.hypot(*(ry - c).T)) / 2
        return np.maximum(1, r_px.astype(int))

    def draw_robot(self, robot_pose,
                   robot_rear_lights=False,
                   robot_tracking_enabled=False,
                   robot_horn_active=True,
                   mode="zenital",
                   observer=None):
        """
        Draws a robot at (x, y, heading) with given wheel angle.
        Supports both zenital (top-down) and camera (projected) modes.

        Args:
            robot_pose: (x, y, heading) world coordinates
            robot_rear_lights: whether rear lights are active
            robot_tracking_enabled: whether tracking cone is visible
            robot_horn_active: whether horn/blade is active
            mode (str): "zenital" (top-down view) or "camera" (projected on snapshot)
            observer (Observer): required if mode="camera"
        """
        if mode == "camera" and observer is None:
            raise ValueError("observer is required for camera projection mode")

        x, y, heading = robot_pose
        heading_rad = math.radians(heading)
        half_l = self.robot_length / 2
        half_w = self.robot_width / 2

        # --- Helper: world → display (screen) coords ---
        def world_to_display(pt):
            return self.world_to_display(pt, mode, observer)

        # --- robot body ---
        body_corners_world = [
            (x + dx * math.cos(heading_rad) - dy * math.sin(heading_rad),
             y + dx * math.sin(heading_rad) + dy * math.cos(heading_rad))
            for dx, dy in [(-half_l, -half_w), (half_l, -half_w), (half_l, half_w), (-half_l, half_w)]
        ]
        body_corners_display = [world_to_display(
            pt) for pt in body_corners_world]
        pygame.draw.polygon(self.screen, (0, 100, 200), body_corners_display)

        # --- Blade circle ---
        circle_color = (100, 210, 210) if robot_horn_active else (
            120, 220, 250)
        blade_x = x + \
            self.blade_pos[0] * math.cos(heading_rad) - \
            self.blade_pos[1] * math.sin(heading_rad)
        blade_y = y + \
            self.blade_pos[0] * math.sin(heading_rad) + \
            self.blade_pos[1] * math.cos(heading_rad)
        blade_center_display = world_to_display((blade_x, blade_y))
        blade_radius_px = self.world_radius_to_px(
            (blade_x, blade_y), self.blade_diameter / 2, mode, observer)

        pygame.draw.circle(self.screen, circle_color,
                           blade_center_display, blade_radius_px)

        # --- Heading indicator ---
        tip_length = half_l
        tip_x = x + tip_length * math.cos(heading_rad)
        tip_y = y + tip_length * math.sin(heading_rad)
        center_display = world_to_display((x, y))
        tip_display = world_to_display((tip_x, tip_y))
        pygame.draw.line(self.screen, (255, 0, 0),
                         center_display, tip_display, 3)

        # --- Rear lights ---
        light_w = self.robot_length * 0.07
        light_h = self.robot_width * 0.15
        side_offset = half_w * 0.5

        t = time.time()
        blink_on = int(t % 1.0 < 0.5)
        blink_color, standby_color = (255, 180, 0), (60, 30, 10)

        for side in [-1, 1]:
            dx_local = -half_l
            dy_local = side * side_offset

            cx = x + dx_local * math.cos(heading_rad) - \
                dy_local * math.sin(heading_rad)
            cy = y + dx_local * math.sin(heading_rad) + \
                dy_local * math.cos(heading_rad)

            color = blink_color if (
                robot_rear_lights and blink_on) else standby_color

            rect_corners_world = [
                (cx + ldx * math.cos(heading_rad) - ldy * math.sin(heading_rad),
                 cy + ldx * math.sin(heading_rad) + ldy * math.cos(heading_rad))
                for ldx, ldy in [(-light_w/2, -light_h/2),
                                 (light_w/2, -light_h/2),
                                 (light_w/2,  light_h/2),
                                 (-light_w/2,  light_h/2)]
            ]
            rect_corners_display = [world_to_display(
                pt) for pt in rect_corners_world]
            pygame.draw.polygon(self.screen, color, rect_corners_display)

        # --- Tracking cone ---
        if robot_tracking_enabled:
            cone_length = half_l
            cone_angle = math.radians(10)
            cone_points_world = [(x, y)] + [
                (x + cone_length * math.cos(a), y + cone_length * math.sin(a))
                for a in [heading_rad - cone_angle, heading_rad + cone_angle]
            ]
            cone_points_display = [world_to_display(
                pt) for pt in cone_points_world]
            pygame.draw.polygon(self.screen, (255, 0, 0),
                                cone_points_display, 0)

    def draw_job_path(self, mode="zenital", observer=None):
        """
        Draw the job path with labels at start, mid, and end.

        Args:
            job_path (list): list of (x, y, ...) world points
            mode (str): "zenital" (top-down view) or "camera" (projected on snapshot)
            observer (Observer): required if mode="camera"
        """
        job_path = self.robot.job_path

        if not job_path or len(job_path) < 2:
            return

        if mode not in ("zenital", "camera"):
            raise ValueError("mode must be 'zenital' or 'camera'")
        if mode == "camera" and observer is None:
            raise ValueError("observer is required for camera projection mode")

        # --- Map world path to screen coordinates (static path: cached per view and size) ---
        cache_key = (mode, self.screen.get_size(), id(job_path))
        if cache_key not in self._job_path_cache:
            self._job_path_cache[cache_key] = self.world_to_display_many(
                [(x, y) for x, y, *_ in job_path], mode, observer).tolist()
        path_points = self._job_path_cache[cache_key]

        # --- Draw path ---

        color = (100, 100, 255, 200)

        # draw on an SRCALPHA surface then blit to preserve translucency
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.lines(overlay, color, False, path_points, 2)

        # Transit back to the first uncut waypoint after manual driving
        approach = self.robot.approach_path
        if approach is not None and len(approach) > 1:
            pygame.draw.lines(overlay, (255, 200, 0, 220), False,
                              self.world_to_display_many(approach, mode, observer).tolist(), 2)
        self.screen.blit(overlay, (0, 0))

        # --- Pick start, mid, end ---
        start_idx, mid_idx, end_idx = 0, len(job_path) // 2, len(job_path) - 1
        labels = [("Start", start_idx), ("Mid", mid_idx), ("End", end_idx)]

        for label, idx in labels:
            x, y, *_ = job_path[idx]
            sx, sy = path_points[idx]

            msg = f"{label}"                            # Option A:   End
            # msg = f"{label}: [ {x:.1f}, {y:.1f}]"     # Option B:   End: [2.1, 12.2]

            text = self.font.render(msg, True, color)
            self.screen.blit(text, (sx + 5, sy - 15))

    def draw_walked_path(self, walked_path, blade_diameter,
                         mode="zenital", observer=None):
        """
        Draw the walked path and blade area as a green line and shaded area.

        Args:
            walked_path (list): list of (x, y) world points
            blade_diameter (float): diameter of the blade for coverage area
            mode (str): "zenital" (top-down view) or "camera" (projected on snapshot)
            observer (Observer): required if mode="camera"
        """
        if mode == "camera" and observer is None:
            raise ValueError("observer is required for camera projection mode")

        if not walked_path or len(walked_path) < 2:
            return

        # --- Convert path points ---
        path_points = self.world_to_display_many(walked_path, mode, observer).astype(int).tolist()

        # --- Blade coverage radius (per point: perspective shrinks it with distance) ---
        blade_radii = self.world_radii_to_px(walked_path, blade_diameter / 2, mode, observer).tolist()

        # --- Draw on transparent overlay to get 50% translucency ---
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)

        # semi-transparent colors (RGBA)
        coverage_color = (50, 120, 50, 150)  # 50% alpha
        line_color = (0, 150, 0, 150)       # 50% alpha

        # Draw blade coverage circles onto overlay
        for pos, radius in zip(path_points, blade_radii):
            pygame.draw.circle(overlay, coverage_color, pos, radius)

        # Draw walked path line onto overlay
        if len(path_points) > 1:
            pygame.draw.lines(overlay, line_color, False, path_points, 2)

        # Blit overlay onto main screen
        self.screen.blit(overlay, (0, 0))

    def dim_background(self, alpha=150, color=(0, 0, 0)):
        """Draw a translucent overlay over the current screen to dim the background.
        alpha: 0..255 (128 ≈ 50% dim)
        """
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        overlay.fill((color[0], color[1], color[2], alpha))
        self.screen.blit(overlay, (0, 0))

    def _draw_zenital_view(self, robot):
        """Helper method to draw the zenital view"""
        self.set_world_space(robot.world_rect)  # recompute scale for the current surface size
        # Draw debug rectangles (background/field)
        self.draw_debug_rects(
            robot.world_rect,
            margin_x=0.0,
            margin_y=0.0,
            safety_buffer=robot.specs.turn_radius + 0.2
        )

        # Dim background so job/walked paths and robot stand out
        self.dim_background()

        # Draw job path
        self.draw_job_path(mode='zenital')

        # Draw walked path
        self.draw_walked_path(
            robot.get_walked_path(),
            blade_diameter=robot.get_blade_diameter(),
            mode='zenital'
        )

        # Draw robot
        self.draw_robot(
            robot_pose=robot.get_pose(),
            robot_rear_lights=robot.state.paused,
            robot_tracking_enabled=robot.state.tracking_enabled,
            robot_horn_active=robot.state.horn,
            mode='zenital'
        )

    def _draw_camera_view(self, robot, observer):
        """Helper method to draw the camera view"""
        # Get window size for current surface (might be full screen or half)
        screen_w, screen_h = self.screen.get_size()

        # Pull fresh background from observer
        background = observer.get_background_surface(size=(screen_w, screen_h))

        # Draw the background
        self.screen.blit(background, (0, 0))

        # Dim the background so overlays are clearer
        self.dim_background()

        # Draw job path
        self.draw_job_path(mode="camera", observer=observer)

        # Draw walked path
        self.draw_walked_path(
            walked_path=robot.state.walked_path,
            blade_diameter=robot.specs.blade_diameter,
            mode="camera",
            observer=observer
        )

        # Draw robot
        self.draw_robot(
            robot_pose=robot.get_pose(),
            robot_rear_lights=robot.state.paused,
            robot_tracking_enabled=robot.state.tracking_enabled,
            robot_horn_active=robot.state.horn,
            mode="camera",
            observer=observer
        )

    def draw_scene(self, robot,  observer=None):
        """
        Draws the entire simulation scene: job path, walked path, and robot.
        Supports three modes: zenital, camera, or both (split screen).

        Args:
            robot: robot instance with state and specs
            mode (str): "zenital", "camera", or "both"
            observer (Observer): Required for camera and both modes
        """
        mode = self.mode

        if mode in ["camera", "both"] and observer is None:
            raise ValueError("observer is required for camera and both modes")

        # Step 1: Fill the whole screen black
        self.screen.fill((0, 0, 0))

        if mode == "both":
            # Split screen into two halves
            screen_w, screen_h = self.screen.get_size()
            half_w = screen_w // 2

            # Create subsurface for zenital view (left half)
            zenital_surface = self.screen.subsurface(
                pygame.Rect(0, 0, half_w, screen_h))
            # Create subsurface for camera view (right half)
            camera_surface = self.screen.subsurface(
                pygame.Rect(half_w, 0, half_w, screen_h))

            # Draw divider line
            pygame.draw.line(self.screen, (100, 100, 100),
                             (half_w, 0), (half_w, screen_h), 2)

            # Save original screen and replace with zenital subsurface
            original_screen = self.screen
            self.screen = zenital_surface

            # Draw zenital view
            self._draw_zenital_view(robot)

            # Switch to camera subsurface
            self.screen = camera_surface

            # Draw camera view
            self._draw_camera_view(robot, observer)

            # Restore original screen
            self.screen = original_screen

        elif mode == "zenital":
            # Draw only zenital view
            self._draw_zenital_view(robot)

        elif mode == "camera":
            # Draw only camera view
            self._draw_camera_view(robot, observer)

        # Finally update the display
        pygame.display.flip()
