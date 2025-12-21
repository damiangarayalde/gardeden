import pygame
import math
import yaml
import time


class Drawer:
    def __init__(self, initial_config, cart, camera_mode="camera"):

        pygame.display.set_caption("Cart Simulation")

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
        self.cart_size = 1  # Default size
        self.scale = 1
        self.mode = camera_mode
        self.offset = (0, 0)
        self.cart_image = None
        self.screen = pygame.display.set_mode(
            self.window_size, pygame.RESIZABLE)

        self.set_world_space(cart.world_rect)

        self.set_cart_size(rect=cart.specs.rect,
                           blade_pos=cart.specs.blade_pos,
                           blade_diameter=cart.get_blade_diameter())

    def set_world_space(self, rect):
        """Map rect to 80% of window, keep aspect ratio."""
        self.world_rect = rect
        wx, wy = self.window_size
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

    def set_cart_size(self, rect, blade_pos, blade_diameter):
        """
        Configure cart size and blade parameters.
        rect: bounding box for cart in local coordinates
        blade_pos: (x, y) offset from cart center
        blade_diameter: effective blade diameter in cm
        """
        self.cart_size = max(rect[2] - rect[0], rect[3] - rect[1])
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

    def draw_cart(self, cart_pose,
                  cart_rear_lights=False,
                  cart_tracking_enabled=False,
                  cart_horn_active=True,
                  mode="zenital",
                  observer=None):
        """
        Draws a cart at (x, y, heading) with given wheel angle.
        Supports both zenital (top-down) and camera (projected) modes.

        Args:
            cart_pose: (x, y, heading) world coordinates
            cart_rear_lights: whether rear lights are active
            cart_tracking_enabled: whether tracking cone is visible
            cart_horn_active: whether horn/blade is active
            mode (str): "zenital" (top-down view) or "camera" (projected on snapshot)
            observer (Observer): required if mode="camera"
        """
        if mode == "camera" and observer is None:
            raise ValueError("observer is required for camera projection mode")

        x, y, heading = cart_pose
        heading_rad = math.radians(heading)
        cart_size = 0.5  # base size in world units

        half = cart_size / 2

        # --- Helper: world → display (screen) coords ---
        def world_to_display(pt):
            if mode == "zenital":
                return self.map_to_screen(pt[0], pt[1])
            else:  # camera mode
                u_img, v_img = observer.world_to_image(pt)  # world → image
                screen_w, screen_h = self.screen.get_size()
                img_w, img_h = observer.frame_size
                u_scr = int(u_img * screen_w / img_w)       # image → screen
                v_scr = int(v_img * screen_h / img_h)
                return (u_scr, v_scr)

        # --- Cart body ---
        body_corners_world = [
            (x + dx * math.cos(heading_rad) - dy * math.sin(heading_rad),
             y + dx * math.sin(heading_rad) + dy * math.cos(heading_rad))
            for dx, dy in [(-half, -half), (half, -half), (half, half), (-half, half)]
        ]
        body_corners_display = [world_to_display(
            pt) for pt in body_corners_world]
        pygame.draw.polygon(self.screen, (0, 100, 200), body_corners_display)

        # --- Blade circle ---
        circle_color = (100, 210, 210) if cart_horn_active else (120, 220, 250)
        blade_x = x + \
            self.blade_pos[0] * math.cos(heading_rad) - \
            self.blade_pos[1] * math.sin(heading_rad)
        blade_y = y + \
            self.blade_pos[0] * math.sin(heading_rad) + \
            self.blade_pos[1] * math.cos(heading_rad)
        blade_center_display = world_to_display((blade_x, blade_y))

        # compute largest distance from any body corner to the blade center (in pixels)
        def _dist(p, q):
            return math.hypot(p[0] - q[0], p[1] - q[1])

        # body_corners_display is a list of (x,y) screen coordinates
        max_corner_dist = 0.0
        for corner in body_corners_display:
            d = _dist(corner, blade_center_display)
            if d > max_corner_dist:
                max_corner_dist = d

        # blade radius = 80% of the largest corner->center distance
        blade_radius_px = max(1, int(max_corner_dist*0.5))

        pygame.draw.circle(self.screen, circle_color,
                           blade_center_display, blade_radius_px)

        # --- Heading indicator ---
        tip_length = cart_size / 2
        tip_x = x + tip_length * math.cos(heading_rad)
        tip_y = y + tip_length * math.sin(heading_rad)
        center_display = world_to_display((x, y))
        tip_display = world_to_display((tip_x, tip_y))
        pygame.draw.line(self.screen, (255, 0, 0),
                         center_display, tip_display, 3)

        # --- Rear lights ---
        light_w = cart_size * 0.07
        light_h = cart_size * 0.15
        side_offset = cart_size * 0.25

        t = time.time()
        blink_on = int(t % 1.0 < 0.5)
        blink_color, standby_color = (255, 180, 0), (60, 30, 10)

        for side in [-1, 1]:
            dx_local = -half
            dy_local = side * side_offset

            cx = x + dx_local * math.cos(heading_rad) - \
                dy_local * math.sin(heading_rad)
            cy = y + dx_local * math.sin(heading_rad) + \
                dy_local * math.cos(heading_rad)

            color = blink_color if (
                cart_rear_lights and blink_on) else standby_color

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
        if cart_tracking_enabled:
            cone_length = half
            cone_angle = math.radians(10)
            cone_points_world = [(x, y)] + [
                (x + cone_length * math.cos(a), y + cone_length * math.sin(a))
                for a in [heading_rad - cone_angle, heading_rad + cone_angle]
            ]
            cone_points_display = [world_to_display(
                pt) for pt in cone_points_world]
            pygame.draw.polygon(self.screen, (255, 0, 0),
                                cone_points_display, 0)

    def draw_job_path(self, job_path, mode="zenital", observer=None):
        """
        Draw the job path with labels at start, mid, and end.

        Args:
            job_path (list): list of (x, y, ...) world points
            mode (str): "zenital" (top-down view) or "camera" (projected on snapshot)
            observer (Observer): required if mode="camera"
        """

        if not job_path or len(job_path) < 2:
            return

        # --- Map world path to screen/image coordinates ---
        if mode == "zenital":
            path_points = [self.map_to_screen(x, y) for x, y, *_ in job_path]

        elif mode == "camera":
            if observer is None:
                raise ValueError(
                    "observer is required for camera projection mode")

            # Get scaling factors from image coords → screen coords
            screen_w, screen_h = self.screen.get_size()
            img_w, img_h = observer.frame_size

            path_points = []
            for x, y, *_ in job_path:
                u_img, v_img = observer.world_to_image((x, y))  # world → image
                # image → screen
                u_scr = int(u_img * screen_w / img_w)
                v_scr = int(v_img * screen_h / img_h)
                path_points.append((u_scr, v_scr))

        else:
            raise ValueError("mode must be 'zenital' or 'camera'")

        # --- Draw path ---

        color = (100, 100, 255, 128) if mode == "camera" else (
            100, 100, 255, 128)

        # draw on an SRCALPHA surface then blit to preserve translucency
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        pygame.draw.lines(overlay, color, False, path_points, 2)
        self.screen.blit(overlay, (0, 0))

        # --- Create font ---
        font = pygame.font.SysFont("Arial", 14)

        # --- Pick start, mid, end ---
        start_idx, mid_idx, end_idx = 0, len(job_path) // 2, len(job_path) - 1
        labels = [("Start", start_idx), ("Mid", mid_idx), ("End", end_idx)]

        for label, idx in labels:
            x, y, *_ = job_path[idx]
            sx, sy = path_points[idx]

            msg = f"{label}"                            # Option A:   End
            # msg = f"{label}: [ {x:.1f}, {y:.1f}]"     # Option B:   End: [2.1, 12.2]

            text = font.render(msg, True, color)
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

        # --- Helper: world → display coords ---
        def world_to_display(pt):
            if mode == "zenital":
                return self.map_to_screen(pt[0], pt[1])
            else:  # camera mode
                u_img, v_img = observer.world_to_image(pt)  # world → image
                screen_w, screen_h = self.screen.get_size()
                img_w, img_h = observer.frame_size
                u_scr = int(u_img * screen_w / img_w)       # image → screen
                v_scr = int(v_img * screen_h / img_h)
                return (u_scr, v_scr)

        # --- Convert path points ---
        path_points = [world_to_display((x, y)) for x, y in walked_path]

        # --- Blade coverage radius ---
        if mode == "zenital":
            blade_radius = int(blade_diameter * self.scale // 2)
        else:
            # scale blade radius relative to screen size
            img_w, img_h = observer.frame_size
            screen_w, screen_h = self.screen.get_size()
            scale_factor = (screen_w / img_w + screen_h / img_h) / 2
            blade_radius = 25  # fallback value

        # --- Draw on transparent overlay to get 50% translucency ---
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)

        # semi-transparent colors (RGBA)
        coverage_color = (50, 120, 50, 128)  # 50% alpha
        line_color = (0, 150, 0, 128)       # 50% alpha

        # Draw blade coverage circles onto overlay
        for pos in path_points:
            pygame.draw.circle(overlay, coverage_color,
                               (int(pos[0]), int(pos[1])), blade_radius)

        # Draw walked path line onto overlay
        if len(path_points) > 1:
            pygame.draw.lines(overlay, line_color, False, path_points, 2)

        # Blit overlay onto main screen
        self.screen.blit(overlay, (0, 0))

    def dim_background(self, alpha=128, color=(0, 0, 0)):
        """Draw a translucent overlay over the current screen to dim the background.
        alpha: 0..255 (128 ≈ 50% dim)
        """
        overlay = pygame.Surface(self.screen.get_size(), pygame.SRCALPHA)
        overlay.fill((color[0], color[1], color[2], alpha))
        self.screen.blit(overlay, (0, 0))

    def _draw_zenital_view(self, job_path, cart):
        """Helper method to draw the zenital view"""
        # Draw debug rectangles (background/field)
        self.draw_debug_rects(
            cart.world_rect,
            margin_x=0.0,
            margin_y=0.0,
            safety_buffer=cart.specs.turn_radius + 0.2
        )

        # Dim background so job/walked paths and cart stand out
        self.dim_background(alpha=128)

        # Draw job path
        self.draw_job_path(job_path, mode='zenital')

        # Draw walked path
        self.draw_walked_path(
            cart.get_walked_path(),
            blade_diameter=cart.get_blade_diameter(),
            mode='zenital'
        )

        # Draw cart
        self.draw_cart(
            cart_pose=cart.get_pose(),
            cart_rear_lights=cart.state.paused,
            cart_tracking_enabled=cart.state.tracking_enabled,
            cart_horn_active=cart.state.horn,
            mode='zenital'
        )

    def _draw_camera_view(self, job_path, cart, observer):
        """Helper method to draw the camera view"""
        # Get window size for current surface (might be full screen or half)
        screen_w, screen_h = self.screen.get_size()

        # Pull fresh background from observer
        background = observer.get_background_surface(size=(screen_w, screen_h))

        # Draw the background
        self.screen.blit(background, (0, 0))

        # Dim the background so overlays are clearer
        self.dim_background(alpha=128)

        # Draw job path
        self.draw_job_path(job_path, mode="camera", observer=observer)

        # Draw walked path
        self.draw_walked_path(
            walked_path=cart.state.walked_path,
            blade_diameter=cart.specs.blade_diameter,
            mode="camera",
            observer=observer
        )

        # Draw cart
        self.draw_cart(
            cart_pose=cart.get_pose(),
            cart_rear_lights=cart.state.paused,
            cart_tracking_enabled=cart.state.tracking_enabled,
            cart_horn_active=cart.state.horn,
            mode="camera",
            observer=observer
        )

    def draw_scene(self, cart,  observer=None):
        """
        Draws the entire simulation scene: job path, walked path, and cart.
        Supports three modes: zenital, camera, or both (split screen).

        Args:
            cart: Cart instance with state and specs
            mode (str): "zenital", "camera", or "both"
            observer (Observer): Required for camera and both modes
        """
        mode = self.mode

        job_path = cart.job_path

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
            self._draw_zenital_view(job_path, cart)

            # Switch to camera subsurface
            self.screen = camera_surface

            # Draw camera view
            self._draw_camera_view(job_path, cart, observer)

            # Restore original screen
            self.screen = original_screen

        elif mode == "zenital":
            # Draw only zenital view
            self._draw_zenital_view(job_path, cart)

        elif mode == "camera":
            # Draw only camera view
            self._draw_camera_view(job_path, cart, observer)

        # Finally update the display
        pygame.display.flip()
