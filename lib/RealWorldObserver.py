import cv2
import numpy as np
import yaml


class Observer:
    """Homography, calibration of work field, custom transformation from image to real wold, future place for mowler detector."""

    def __init__(self):  # , initial_config="config.yml"):
        # # Load config
        # with open(initial_config, "r") as f:
        #     config = yaml.safe_load(f)

        # # x,y, angle in degrees
        # self.state = CartState(initial_pose=(0, 0, 0), config=config["cart"])
        # self.world_rect = config["world"]["rect"]

        # Adjust units (cm, m, etc.)
        self.pts_world = np.array([
            [0, 0],       # top-left
            [4.8, 0],     # top-right
            [4.8, 12.7],   # bottom-right
            [0, 12.7]      # bottom-left
        ], dtype=np.float32)

        self.clicked_points = []
        self.H = None
        self.mask = None
        self.calibrated = False

        # Load a snapshot (replace with camera frame if needed)
        # f = cv2.imread("snapshot.jpg")
        self.frame = cv2.imread("snapshot.jpg")  # f.copy()

        cv2.namedWindow("Calibration")
        cv2.setMouseCallback("Calibration", self.mouse_callback, self.frame)

    # -------------------------------------------------------------------------
    #
    # -------------------------------------------------------------------------

    def image_to_world(self, point):
        """Map image pixel (u,v) to real-world (x,y)."""
        uv1 = np.array([point[0], point[1], 1.0])
        xy1 = self.H @ uv1
        xy1 /= xy1[2]
        return float(xy1[0]), float(xy1[1])

    def mouse_callback(self, event, x, y, flags, param):

        if event == cv2.EVENT_LBUTTONDOWN:
            if not self.calibrated:
                # Collect the 4 calibration points
                if len(self.clicked_points) < 4:
                    self.clicked_points.append([x, y])
                    print(
                        f"Calibration point {len(self.clicked_points)}: ({x}, {y})")
            else:
                # After calibration, map any clicked point
                real_coords = self.image_to_world((x, y))
                print(f"Pixel ({x},{y}) → Real world {real_coords}")
                cv2.circle(param, (x, y), 5, (255, 0, 0), -1)
                cv2.putText(param, f"{real_coords[0]:.1f},{real_coords[1]:.1f}",
                            (x+10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                            (255, 0, 0), 1, cv2.LINE_AA)

    def run_calibration(self):

        while True:
            display = self.frame

            if display is None:
                raise FileNotFoundError(
                    f"Could not load image at {snapshot_path}")

            # Draw calibration points
            for i, pt in enumerate(self.clicked_points):
                cv2.circle(display, tuple(pt), 5, (0, 0, 255), -1)
                if i < len(self.pts_world):
                    rw = self.pts_world[i]
                    cv2.putText(display, f"{rw[0]},{rw[1]}",
                                (pt[0]+10, pt[1]), cv2.FONT_HERSHEY_SIMPLEX,
                                0.5, (0, 255, 0), 1, cv2.LINE_AA)

            cv2.imshow("Calibration", display)
            key = cv2.waitKey(1) & 0xFF

            # Press Enter to compute homography once 4 points are set
            if key == 13 and len(self.clicked_points) == 4 and not self.calibrated:
                pts_image = np.array(self.clicked_points, dtype=np.float32)
                self.H, self.mask = cv2.findHomography(
                    pts_image, self.pts_world)
                print("Homography matrix:\n", self.H)
                self.calibrated = True
                print(
                    "Calibration complete! Now click anywhere to get real-world coords.")

            # ESC to quit
            elif key == 27:
                break

        cv2.destroyAllWindows()
