from pathlib import Path
import cv2
import cv2.aruco as aruco

# Load image
image = cv2.imread(str(Path(__file__).with_name("aruco_test.jpg")))
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Load dictionary & parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
detector = aruco.ArucoDetector(aruco_dict, parameters)

corners, ids, _ = detector.detectMarkers(gray)
print(f"Detected ids: {None if ids is None else ids.flatten().tolist()}")

aruco.drawDetectedMarkers(image, corners, ids)
cv2.imshow("ArUco Detection", image)
cv2.waitKey(0)
cv2.destroyAllWindows()
