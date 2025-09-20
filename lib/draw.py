import pygame
import math
import yaml
import time
import cv2
import numpy as np


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
            self.window_size)  # (WIDTH, HEIGHT))

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
        pygame.draw.rect(self.screen, (0, 150, 0), base_rect,
                         2)      # green: original rect
        pygame.draw.rect(self.screen, (150, 150, 0), ext_rect,
                         2)     # yellow: allowed outer rect
        pygame.draw.rect(self.screen, (0, 0, 200), in_rect,
                         2)        # blue: inset rect

    def map_to_screen(self, x, y):
        """Map local coordinates to screen coordinates using scale and offset."""
        sx = x * self.scale + self.offset[0]
        sy = y * self.scale + self.offset[1]
        return sx, sy

    def draw_cart(self, cart_pose, cart_rear_lights=False, cart_tracking_enabled=False, cart_horn_active=True):
        """
        Draws a cart at (x, y, heading) with given wheel angle.
        The cart is a square with a central circle and 4 wheels (rectangles).
        Front wheels rotate to indicate steering.
        """
        x, y, heading = cart_pose

        # Map cart position to screen
        x, y = self.map_to_screen(x, y)
        heading_rad = math.radians(heading)
        cart_size = self.cart_size * self.scale

        # --- Draw cart body (square) ---
        half = cart_size / 2
        body_corners = []
        for dx, dy in [(-half, -half), (half, -half), (half, half), (-half, half)]:
            bx = x + dx * math.cos(heading_rad) - dy * math.sin(heading_rad)
            by = y + dx * math.sin(heading_rad) + dy * math.cos(heading_rad)
            body_corners.append((bx, by))
        pygame.draw.polygon(self.screen, (0, 100, 200), body_corners)

        # --- Draw central circle (blade) ---

        if cart_horn_active:
            circle_color = (100, 100, 210)
        else:
            circle_color = (120, 120, 250)

        blade_x = x + self.blade_pos[0] * self.scale * math.cos(
            heading_rad) - self.blade_pos[1] * self.scale * math.sin(heading_rad)
        blade_y = y + self.blade_pos[0] * self.scale * math.sin(
            heading_rad) + self.blade_pos[1] * self.scale * math.cos(heading_rad)
        pygame.draw.circle(self.screen, circle_color, (int(blade_x), int(
            blade_y)), int(self.blade_diameter * self.scale // 2))

        # --- Draw heading indicator (small line from center) ---
        tip_length = cart_size // 2
        tip_x = x + tip_length * math.cos(heading_rad)
        tip_y = y + tip_length * math.sin(heading_rad)
        pygame.draw.line(self.screen, (255, 0, 0), (x, y), (tip_x, tip_y), 3)

        # --- Draw rear lights ---

        rear_x = -half  # back face along -X
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
            dx_local = rear_x
            dy_local = side * side_offset

            # Rotate into world space
            cx = x + dx_local * math.cos(heading_rad) - \
                dy_local * math.sin(heading_rad)
            cy = y + dx_local * math.sin(heading_rad) + \
                dy_local * math.cos(heading_rad)

            # Choose color
            if cart_rear_lights:
                color = blink_color if blink_on else standby_color
            else:
                color = standby_color

            # Compute rectangle corners for each light (so it rotates with heading)
            rect_corners = []
            for ldx, ldy in [(-light_w/2, -light_h/2),
                             (light_w/2, -light_h/2),
                             (light_w/2,  light_h/2),
                             (-light_w/2,  light_h/2)]:
                lx = cx + ldx * math.cos(heading_rad) - \
                    ldy * math.sin(heading_rad)
                ly = cy + ldx * math.sin(heading_rad) + \
                    ldy * math.cos(heading_rad)
                rect_corners.append((lx, ly))

            pygame.draw.polygon(self.screen, color, rect_corners)

        # --- Draw forward tracking cone ---
        if cart_tracking_enabled:
            cone_length = half
            cone_angle = math.radians(10)
            angles = [heading_rad - cone_angle, heading_rad + cone_angle]
            cone_points = [(x, y)]
            for a in angles:
                px = x + cone_length * math.cos(a)
                py = y + cone_length * math.sin(a)
                cone_points.append((px, py))
            pygame.draw.polygon(self.screen, (255, 0, 0), cone_points, 0)

    def draw_job_path(self, job_path):
        """Draw the job path as a blue line, with labels at start, mid, end."""
        if job_path and len(job_path) > 1:
            job_path_screen = [self.map_to_screen(
                x, y) for x, y, *_ in job_path]

            # Draw the path
            pygame.draw.lines(self.screen, (100, 100, 200),
                              False, job_path_screen, 2)

            # Create font
            font = pygame.font.SysFont("Arial", 14)

            # --- Pick positions: start, mid, end ---
            start_idx = 0
            mid_idx = len(job_path) // 2
            end_idx = len(job_path) - 1

            for idx, label in zip([start_idx, mid_idx, end_idx], ["Start", "Mid", "End"]):
                x, y, *_ = job_path[idx]
                sx, sy = job_path_screen[idx]

                # Render text with coordinates (3 decimals)
                text = font.render(
                    f"{label}: [ {x:.1f}, {y:.1f}]", True, (100, 100, 200))

                # Draw text slightly offset so it doesn't overlap the line
                self.screen.blit(text, (sx + 5, sy - 15))

    def draw_walked_path(self, walked_path, blade_diameter):
        """Draw the walked path and blade area as a green line and shaded area."""
        if len(walked_path) > 1:
            walked_path_screen = [self.map_to_screen(
                x, y) for x, y in walked_path]
            blade_radius = int(blade_diameter * self.scale // 2)
            for pos in walked_path_screen:
                pygame.draw.circle(self.screen, (50, 120, 50),
                                   (int(pos[0]), int(pos[1])), blade_radius)
            pygame.draw.lines(self.screen, (0, 150, 0), False,
                              walked_path_screen, 2)

    def draw_scene(self, job_path, cart):
        """
        Draws the entire simulation scene: job path, walked path, and cart.
        All cart details are retrieved from the cart instance.
        """

        # Step 1: Fill the whole screen black
        self.screen.fill((0, 0, 0))

        # Step 2: Draw the target area (dark green rectangle)
        dark_green = (30, 70, 30)
        pygame.draw.rect(
            self.screen,
            dark_green,
            self.world_rect,  # assuming self.target_area is a pygame.Rect
        )
       # self.screen.fill(dark_green)

        self.draw_debug_rects(cart.world_rect,
                              margin_x=0.0,
                              margin_y=0.0,
                              safety_buffer=cart.specs.turn_radius + 0.2)

        # Step 3: Draw the job path, cart, etc.
        self.draw_job_path(job_path)
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

    def draw_augmented_reality(self, observer, job_path, cart, alpha=0.5):
        """
        Projects the rectangle, path, and cart over the real-world image
        using the inverse homography. Returns an augmented reality frame.
        """

        # Step 1: Get a fresh copy of the observer's frame
        frame = observer.get_augmented_frame()

        # Step 2: Draw job path in image space
        if job_path and len(job_path) > 1:
            path_pixels = [observer.world_to_image(
                (x, y)) for x, y, *_ in job_path]
            cv2.polylines(frame, [np.int32(path_pixels)],
                          False, (255, 0, 0), 2)

        # Step 3: Draw walked path (blade coverage)
        walked_path = cart.get_walked_path()
        if walked_path and len(walked_path) > 1:
            blade_radius = int(cart.get_blade_diameter() *
                               10)  # scale up for visibility
            for x, y in walked_path:
                px, py = observer.world_to_image((x, y))
                cv2.circle(frame, (px, py), blade_radius, (0, 200, 0), -1)

        # Step 4: Draw cart (as polygon projected with homography)
        x, y, heading = cart.get_pose()
        half = self.cart_size / 2
        heading_rad = math.radians(heading)

        # Cart body corners in world coords
        body_world = []
        for dx, dy in [(-half, -half), (half, -half), (half, half), (-half, half)]:
            bx = x + dx * math.cos(heading_rad) - dy * math.sin(heading_rad)
            by = y + dx * math.sin(heading_rad) + dy * math.cos(heading_rad)
            body_world.append((bx, by))

        body_img = [observer.world_to_image(pt) for pt in body_world]
        cv2.polylines(frame, [np.int32(body_img)], True, (0, 100, 200), 2)

        # Draw heading line
        tip_length = half
        tip_x = x + tip_length * math.cos(heading_rad)
        tip_y = y + tip_length * math.sin(heading_rad)
        p1 = observer.world_to_image((x, y))
        p2 = observer.world_to_image((tip_x, tip_y))
        cv2.line(frame, p1, p2, (0, 0, 255), 2)

        # Step 5: Blend augmented layer with real frame
        blended = cv2.addWeighted(observer.frame, 1 - alpha, frame, alpha, 0)

        cv2.imshow("Augmented Reality", blended)

        return blended
