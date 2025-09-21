import pygame
import math
import yaml
import time


class Drawer:
    def __init__(self, initial_config, cart):

        pygame.display.set_caption("Cart Simulation")

        # Load config from YAML
        with open(initial_config, "r") as f:
            config = yaml.safe_load(f)

        self.window_size = (config["graphics"]["width"],
                            config["graphics"]["height"])
        self.cart_size = 1  # Default size
        self.scale = 1
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

    # def update_image(self, window_size):
    #     """Update world view and cart image on window resize or pose change."""
    #     self.window_size = window_size
    #     if self.world_rect:
    #         self.set_world_space(self.world_rect)
    #     # Optionally, redraw cart at new pose if needed

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

    def draw_cart(self, cart_pose, cart_rear_lights=False, cart_tracking_enabled=False, cart_horn_active=True, mode="zenital", observer=None):
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
        cart_size = 0.5  # self.cart_size * self.scale

        half = cart_size / 2
        print(
            f"cart_size:{self.cart_size}, scale: {self.scale} \t half: {half}  \n")

        # --- Helper function to convert world coordinates to screen/image coordinates ---
        def world_to_display(pt):
            if mode == "zenital":
                return self.map_to_screen(pt[0], pt[1])
            else:  # camera mode
                return observer.world_to_image(pt)

        # --- Draw cart body (square) ---
        body_corners_world = []
        for dx, dy in [(-half, -half), (half, -half), (half, half), (-half, half)]:
            bx = x + dx * math.cos(heading_rad) - dy * math.sin(heading_rad)
            by = y + dx * math.sin(heading_rad) + dy * math.cos(heading_rad)
            body_corners_world.append((bx, by))

        body_corners_display = [world_to_display(
            pt) for pt in body_corners_world]
        pygame.draw.polygon(self.screen, (0, 100, 200), body_corners_display)

        # --- Draw central circle (blade) ---
        if cart_horn_active:
            circle_color = (100, 100, 210)
        else:
            circle_color = (120, 120, 250)

        blade_x = x + self.blade_pos[0] * self.scale * math.cos(
            heading_rad) - self.blade_pos[1] * self.scale * math.sin(heading_rad)
        blade_y = y + self.blade_pos[0] * self.scale * math.sin(
            heading_rad) + self.blade_pos[1] * self.scale * math.cos(heading_rad)
        blade_center_display = world_to_display((blade_x, blade_y))
        pygame.draw.circle(self.screen, circle_color, blade_center_display, int(
            self.blade_diameter * self.scale // 2))

        # --- Draw heading indicator (small line from center) ---
        tip_length = cart_size // 2
        tip_x = x + tip_length * math.cos(heading_rad)
        tip_y = y + tip_length * math.sin(heading_rad)
        center_display = world_to_display((x, y))
        tip_display = world_to_display((tip_x, tip_y))
        pygame.draw.line(self.screen, (255, 0, 0),
                         center_display, tip_display, 3)

        # --- Draw rear lights ---
        light_w = cart_size * 0.07
        light_h = cart_size * 0.15
        side_offset = cart_size * 0.25  # push along Y (up/down from center)

        # Blink control: 1 Hz (0.5s on, 0.5s off)
        t = time.time()
        blink_on = int(t % 1.0 < 0.5)

        # Colors
        blink_color = (255, 180, 0)   # bright amber
        standby_color = (60, 30, 10)  # very dim amber

        for side in [-1, 1]:  # bottom (-1), top (+1) relative to cart
            dx_local = -half  # back face along -X
            dy_local = side * side_offset

            # Calculate light center in world coordinates
            cx = x + dx_local * math.cos(heading_rad) - \
                dy_local * math.sin(heading_rad)
            cy = y + dx_local * math.sin(heading_rad) + \
                dy_local * math.cos(heading_rad)

            # Choose color
            if cart_rear_lights:
                color = blink_color if blink_on else standby_color
            else:
                color = standby_color

            # Compute rectangle corners for each light in world coordinates
            rect_corners_world = []
            for ldx, ldy in [(-light_w/2, -light_h/2),
                             (light_w/2, -light_h/2),
                             (light_w/2,  light_h/2),
                             (-light_w/2,  light_h/2)]:
                lx = cx + ldx * math.cos(heading_rad) - \
                    ldy * math.sin(heading_rad)
                ly = cy + ldx * math.sin(heading_rad) + \
                    ldy * math.cos(heading_rad)
                rect_corners_world.append((lx, ly))

            # Convert to display coordinates and draw
            rect_corners_display = [world_to_display(
                pt) for pt in rect_corners_world]
            pygame.draw.polygon(self.screen, color, rect_corners_display)

        # --- Draw forward tracking cone ---
        if cart_tracking_enabled:
            cone_length = half
            cone_angle = math.radians(10)
            angles = [heading_rad - cone_angle, heading_rad + cone_angle]

            cone_points_world = [(x, y)]
            for a in angles:
                px = x + cone_length * math.cos(a)
                py = y + cone_length * math.sin(a)
                cone_points_world.append((px, py))

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
            path_points = [observer.world_to_image(
                (x, y)) for x, y, *_ in job_path]

        else:
            raise ValueError("mode must be 'zenital' or 'camera'")

        # --- Draw path ---
        color = (0, 0, 255) if mode == "camera" else (100, 100, 200)
        pygame.draw.lines(self.screen, color, False, path_points, 2)

        # --- Create font ---
        font = pygame.font.SysFont("Arial", 14)

        # --- Pick start, mid, end ---
        start_idx, mid_idx, end_idx = 0, len(job_path) // 2, len(job_path) - 1
        labels = [("Start", start_idx), ("Mid", mid_idx), ("End", end_idx)]

        for label, idx in labels:
            x, y, *_ = job_path[idx]
            sx, sy = path_points[idx]

            # Render text (x,y rounded)
            text = font.render(f"{label}: [ {x:.1f}, {y:.1f}]", True, color)

            # Slightly offset to avoid overlap
            self.screen.blit(text, (sx + 5, sy - 15))

    def draw_walked_path(self, walked_path, blade_diameter, mode="zenital", observer=None):
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

        # --- Convert path points to screen/image coordinates ---
        if mode == "zenital":
            path_points = [self.map_to_screen(x, y) for x, y in walked_path]
            # For zenital mode, scale the blade radius with the view scale
            blade_radius = int(blade_diameter * self.scale // 2)
        else:  # camera mode
            path_points = [observer.world_to_image(
                (x, y)) for x, y in walked_path]
            # For camera mode, use fixed scaling (as in the original camera code)
            blade_radius = int(blade_diameter * 10 / 2)

        # --- Draw blade coverage area ---
        for pos in path_points:
            pygame.draw.circle(self.screen, (50, 120, 50),
                               (int(pos[0]), int(pos[1])), blade_radius)

        # --- Draw path line ---
        pygame.draw.lines(self.screen, (0, 150, 0), False, path_points, 2)

    def draw_scene(self, job_path, cart):
        """
        Draws the entire simulation scene: job path, walked path, and cart.
        All cart details are retrieved from the cart instance.
        """

        # Step 1: Fill the whole screen black
        self.screen.fill((0, 0, 0))

        # Step 2: Draw the target area (dark green rectangle)
        dark_green = (30, 70, 30)
        # pygame.draw.rect(
        #     self.screen,
        #     dark_green,
        #     self.world_rect,  # assuming self.target_area is a pygame.Rect
        # )
       # self.screen.fill(dark_green)

        self.draw_debug_rects(cart.world_rect,
                              margin_x=0.0,
                              margin_y=0.0,
                              safety_buffer=cart.specs.turn_radius + 0.2)

        self.draw_job_path(job_path, mode='zenital')

        self.draw_walked_path(
            cart.get_walked_path(),
            blade_diameter=cart.get_blade_diameter()
        )

        self.draw_cart(
            cart_pose=cart.get_pose(),
            cart_rear_lights=cart.state.paused,
            cart_tracking_enabled=cart.state.tracking_enabled,
            cart_horn_active=cart.state.horn
        )

        # Finally update the display
        pygame.display.flip()

    # def draw_scene_overlay(self, screen, observer, assets):
    def draw_scene_overlay(self, observer, job_path, cart):
        """
        Draw the AR scene:
        - Request fresh background from observer
        - Scale to fit screen
        - Draw assets (paths, carts, etc.) using observer transforms
        """

        # 1. Get window size
        screen_w, screen_h = self.screen.get_size()

        # 2. Pull fresh background from observer
        background = observer.get_background_surface(size=(screen_w, screen_h))

        # 3. Draw the background
        self.screen.blit(background, (0, 0))

        self.draw_job_path(job_path, mode="camera", observer=observer)

        self.draw_walked_path(walked_path=cart.state.walked_path,
                              blade_diameter=cart.specs.blade_diameter,
                              mode="camera",
                              observer=observer)

        self.draw_cart(
            cart_pose=cart.get_pose(),
            cart_rear_lights=cart.state.paused,
            cart_tracking_enabled=cart.state.tracking_enabled,
            cart_horn_active=cart.state.horn,
            mode="camera", observer=observer
        )

        # Update window
        pygame.display.flip()
