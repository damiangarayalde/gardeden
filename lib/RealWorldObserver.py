import numpy as np
import cv2
import yaml
import pygame


class Observer:
    """Homography, calibration of work field, custom transformation from image to real world, AR overlay with pygame."""

    def __init__(self, snapshot_path="snapshot.jpg", predefined_points=None, screen=None):

        # Adjust units (cm, m, etc.)
        self.pts_world = np.array([
            [0, 0],    # bottom-left
            [3.5, 0],  # top-left
            [3.5, 2.1],  # top-right
            [0, 2.1]  # bottom-right
        ], dtype=np.float32)
        # self.pts_world = np.array([
        #     [0, 0],    # bottom-left
        #     [12.7, 0],  # top-left
        #     [12.7, 4.8],  # top-right
        #     [0, 4.8]  # bottom-right
        # ], dtype=np.float32)

        self.clicked_points = []
        self.H = None
        self.H_inv = None
        self.mask = None
        self.calibrated = False

        # Load snapshot
        self.frame = cv2.imread(snapshot_path)
        if self.frame is None:
            raise FileNotFoundError(
                f"Could not load snapshot at {snapshot_path}")

        # Convert BGR → RGB and to pygame surface
        frame_rgb = cv2.cvtColor(self.frame, cv2.COLOR_BGR2RGB)
        h, w, _ = frame_rgb.shape
        self.frame_size = (w, h)
        self.background_surface = pygame.image.frombuffer(
            frame_rgb.tobytes(), (w, h), "RGB")

        # Convert to pixel array for math if needed
        self.frame_size = self.background_surface.get_size()

        if predefined_points is not None:
            if len(predefined_points) == 4:
                self.clicked_points = predefined_points
                self.calibrate_with_points()
            else:
                print("Error: Predefined points must contain exactly 4 points")
        else:
            self.run_calibration(screen)

    def calibrate_with_points(self):
        """Calibrate using predefined 4 image points → world coords."""
        pts_image = np.array(self.clicked_points, dtype=np.float32)
        self.H, self.mask = cv2.findHomography(pts_image, self.pts_world)
        self.H_inv = np.linalg.inv(self.H)  # cached: world_to_image runs per point per frame
        self.calibrated = True
        print("Calibration complete using predefined points!")

    # -------------------------------------------------------------------------
    # Coordinate transforms
    # -------------------------------------------------------------------------

    def image_to_world(self, point):
        """Map image pixel (u,v) to real-world (x,y)."""
        uv1 = np.array([point[0], point[1], 1.0])
        xy1 = self.H @ uv1
        xy1 /= xy1[2]
        return float(xy1[0]), float(xy1[1])

    def world_to_image(self, point):
        """Map real-world (x,y) to image pixel (u,v)."""
        if self.H is None:
            raise ValueError("Homography not calibrated yet")
        xy1 = np.array([point[0], point[1], 1.0])
        uv1 = self.H_inv @ xy1
        uv1 /= uv1[2]
        return int(uv1[0]), int(uv1[1])

    # -------------------------------------------------------------------------
    # Frame accessor
    # -------------------------------------------------------------------------
    def get_background_surface(self, size=None):
        """
        Return a pygame.Surface of the latest camera snapshot.
        If size is provided, return a resized copy.
        """
        surface = self.background_surface.copy()
        if size is not None:
            surface = pygame.transform.smoothscale(surface, size)
        return surface

    # -------------------------------------------------------------------------
    # Calibration with mouse clicks
    # -------------------------------------------------------------------------
    def run_calibration(self, screen):
        """
        Run calibration loop inside pygame window.
        Left click: select 4 calibration points
        ENTER: compute homography
        ESC: exit
        """
        running = True
        font = pygame.font.SysFont("Arial", 16)

        while running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    running = False

                # elif event.type == pygame.VIDEORESIZE:
                #     # Recreate resizable window with new size
                #     screen = pygame.display.set_mode(
                #         (event.w, event.h), pygame.RESIZABLE)

                elif event.type == pygame.MOUSEBUTTONDOWN and not self.calibrated:
                    if len(self.clicked_points) < 4:
                        # Convert from SCREEN coords → IMAGE coords
                        screen_w, screen_h = screen.get_size()
                        img_w, img_h = self.frame_size
                        x_img = int(event.pos[0] * img_w / screen_w)
                        y_img = int(event.pos[1] * img_h / screen_h)
                        self.clicked_points.append((x_img, y_img))
                        print(
                            f"Calibration point {len(self.clicked_points)} (image coords): {(x_img, y_img)}")

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN and len(self.clicked_points) == 4:
                        self.calibrate_with_points()
                    elif event.key == pygame.K_ESCAPE:
                        running = False

            # Get current screen size
            screen_w, screen_h = screen.get_size()

            # Scale background to fit window
            scaled_surface = pygame.transform.smoothscale(
                self.background_surface, (screen_w, screen_h))
            screen.blit(scaled_surface, (0, 0))

            # Draw calibration points (map image coords → screen coords)
            for i, pt_img in enumerate(self.clicked_points):
                x_scr = int(pt_img[0] * screen_w / self.frame_size[0])
                y_scr = int(pt_img[1] * screen_h / self.frame_size[1])
                pygame.draw.circle(screen, (255, 0, 0), (x_scr, y_scr), 5)

                if i < len(self.pts_world):
                    rw = self.pts_world[i]
                    txt = font.render(f"{rw[0]},{rw[1]}", True, (0, 255, 0))
                    screen.blit(txt, (x_scr + 10, y_scr))

            pygame.display.flip()
