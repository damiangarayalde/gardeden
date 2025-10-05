import cv2
import cv2.aruco as aruco
# Load image
image = cv2.imread(
    "/Users/damiangarayalde/Work - Github Repos/gardeden/tests/aruco_test.jpg")
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

# Load dictionary & parameters
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
parameters = aruco.DetectorParameters()
cv2.imshow("ArUco Detection", image)
