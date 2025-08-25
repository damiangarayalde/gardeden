import pygame
import math


class CartDrawer:
    def __init__(self, window_size):
        self.window_size = window_size
        self.world_rect = None
        self.cart_image = None
        self.cart_size = 40  # Default size
        self.blade_pos = (0, 0)
        self.blade_diameter = 20  # Default diameter in cm
        self.scale = 1.0
        self.offset = (0, 0)

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

    def set_cart_image(self, image):
        """Set the cart image for drawing."""
        if not isinstance(image, pygame.Surface):
            raise ValueError("cart_image must be a pygame.Surface")
        self.cart_image = image

    def update_image(self, window_size, cart_pose):
        """Update world view and cart image on window resize or pose change."""
        self.window_size = window_size
        if self.world_rect:
            self.set_world_space(self.world_rect)
        # Optionally, redraw cart at new pose if needed

    def map_to_screen(self, x, y):
        """Map local coordinates to screen coordinates using scale and offset."""
        sx = x * self.scale + self.offset[0]
        sy = y * self.scale + self.offset[1]
        return sx, sy

    def draw_cart(self, screen, cart_pose, wheel_angle):
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
        pygame.draw.polygon(screen, (0, 100, 200), body_corners)

        # --- Draw central circle (blade) ---
        blade_x = x + self.blade_pos[0] * self.scale * math.cos(
            heading_rad) - self.blade_pos[1] * self.scale * math.sin(heading_rad)
        blade_y = y + self.blade_pos[0] * self.scale * math.sin(
            heading_rad) + self.blade_pos[1] * self.scale * math.cos(heading_rad)
        pygame.draw.circle(screen, (200, 200, 0), (int(blade_x), int(
            blade_y)), int(self.blade_diameter * self.scale // 2))

        # --- Draw wheels ---
        wheel_w, wheel_h = 18 * self.scale, 8 * self.scale  # long axis along heading
        # Place wheels at front/rear edges along heading
        wheel_offset = self.cart_size * self.scale / 2 - wheel_h / 2
        lateral_offset = self.cart_size * self.scale / 2 - wheel_w / 2

        wheel_positions = [
            (-lateral_offset, -wheel_offset),
            (-lateral_offset, wheel_offset),
            (lateral_offset, -wheel_offset),
            (lateral_offset, wheel_offset),
        ]
        for i, (dx, dy) in enumerate(wheel_positions):
            wheel_angle_rad = heading_rad
            if i >= 2:
                wheel_angle_rad += math.radians(wheel_angle)
            wx = x + dx * math.cos(heading_rad) - dy * math.sin(heading_rad)
            wy = y + dx * math.sin(heading_rad) + dy * math.cos(heading_rad)
            wheel_corners = []
            for wdx, wdy in [(-wheel_w/2, -wheel_h/2), (wheel_w/2, -wheel_h/2),
                             (wheel_w/2, wheel_h/2), (-wheel_w/2, wheel_h/2)]:
                cx = wx + wdx * math.cos(wheel_angle_rad) - \
                    wdy * math.sin(wheel_angle_rad)
                cy = wy + wdx * math.sin(wheel_angle_rad) + \
                    wdy * math.cos(wheel_angle_rad)
                wheel_corners.append((cx, cy))
            color = (50, 50, 50) if i < 2 else (120, 120, 120)
            pygame.draw.polygon(screen, color, wheel_corners)

        # --- Draw heading indicator (small line from center) ---
        tip_length = cart_size // 2
        tip_x = x + tip_length * math.cos(heading_rad)
        tip_y = y + tip_length * math.sin(heading_rad)
        pygame.draw.line(screen, (255, 0, 0), (x, y), (tip_x, tip_y), 3)
