import numpy as np
import yaml
import pygame


class Observer:
    """Homography, calibration of work field, custom transformation from image to real world, AR overlay with pygame."""

    def __init__(self, snapshot_path="snapshot.jpg", predefined_points=None):

        # Adjust units (cm, m, etc.)
        self.pts_world = np.array([
            [0, 0],    # bottom-left
            [12.7, 0],  # top-left
            [12.7, 4.8],  # top-right
            [0, 4.8]  # bottom-right
        ], dtype=np.float32)

        self.clicked_points = []
        self.H = None
        self.mask = None
        self.calibrated = False

        # Load a snapshot (replace with camera frame if needed)
        try:
            self.frame_surface = pygame.image.load(snapshot_path).convert()
        except FileNotFoundError:
            raise FileNotFoundError(f"Could not load image at {snapshot_path}")

        # Convert to pixel array for math if needed
        self.frame_size = self.frame_surface.get_size()

        if predefined_points is not None:
            if len(predefined_points) == 4:
                self.clicked_points = predefined_points
                self.calibrate_with_points()
            else:
                print("Error: Predefined points must contain exactly 4 points")

    def calibrate_with_points(self):
        """Calibrate using predefined 4 image points → world coords."""
        pts_image = np.array(self.clicked_points, dtype=np.float32)
        self.H, self.mask = find_homography(pts_image, self.pts_world)
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
        H_inv = np.linalg.inv(self.H)
        xy1 = np.array([point[0], point[1], 1.0])
        uv1 = H_inv @ xy1
        uv1 /= uv1[2]
        return int(uv1[0]), int(uv1[1])

    # -------------------------------------------------------------------------
    # Frame accessor
    # -------------------------------------------------------------------------
    def get_augmented_surface(self):
        """Return a fresh pygame.Surface copy of the frame for drawing AR overlays."""
        return self.frame_surface.copy()

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

                elif event.type == pygame.VIDEORESIZE:
                    # Recreate resizable window with new size
                    screen = pygame.display.set_mode(
                        (event.w, event.h), pygame.RESIZABLE)

                elif event.type == pygame.MOUSEBUTTONDOWN and not self.calibrated:
                    if len(self.clicked_points) < 4:
                        self.clicked_points.append(event.pos)
                        print(
                            f"Calibration point {len(self.clicked_points)}: {event.pos}")

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_RETURN and len(self.clicked_points) == 4:
                        self.calibrate_with_points()
                    elif event.key == pygame.K_ESCAPE:
                        running = False

            # Get current screen size
            screen_w, screen_h = screen.get_size()

            # Scale the snapshot to fit the screen size
            scaled_surface = pygame.transform.smoothscale(
                self.frame_surface, (screen_w, screen_h))

            # Draw scaled snapshot
            screen.blit(scaled_surface, (0, 0))

            # Draw calibration points
            for i, pt in enumerate(self.clicked_points):
                pygame.draw.circle(screen, (255, 0, 0), pt, 5)
                if i < len(self.pts_world):
                    rw = self.pts_world[i]
                    txt = font.render(f"{rw[0]},{rw[1]}", True, (0, 255, 0))
                    screen.blit(txt, (pt[0] + 10, pt[1]))

            pygame.display.flip()


# -------------------------------------------------------------------------
# Replacement for cv2.findHomography (DLT method, numpy only)
# -------------------------------------------------------------------------
def find_homography(src_pts, dst_pts):
    """
    Minimal replacement for cv2.findHomography using Direct Linear Transform (DLT).
    src_pts, dst_pts: Nx2 arrays
    Returns: H (3x3), mask
    """
    if src_pts.shape[0] < 4:
        raise ValueError("At least 4 points required")

    A = []
    for (x, y), (X, Y) in zip(src_pts, dst_pts):
        A.append([-x, -y, -1, 0, 0, 0, x*X, y*X, X])
        A.append([0, 0, 0, -x, -y, -1, x*Y, y*Y, Y])
    A = np.array(A)

    # Solve using SVD
    _, _, Vt = np.linalg.svd(A)
    H = Vt[-1].reshape(3, 3)
    H /= H[2, 2]
    return H, None
