import cv2
import numpy as np

# Define marker size in meters
MARKER_SIZE = 0.095  # 9.5 cm

# Load the pre-defined dictionary

aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
parameters = cv2.aruco.DetectorParameters()


# Load camera calibration parameters
# Replace with your own camera calibration parameters
camera_matrix = np.array([[728, 0, 337], [0, 724, 323], [0, 0, 1]])  # Example values
dist_coeffs = np.zeros((5, 1))  # Assume no distortion for now

# Start video capture
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect ArUco markers
    corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

    if ids is not None:
        for i, corner in enumerate(corners):
            # Estimate pose
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(corner, MARKER_SIZE, camera_matrix, dist_coeffs)
            
            # Distance is the Z translation component
            distance = tvec[0][0][2]

            # Draw marker and display distance
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)


            
           # cv2.aruco.drawAxis(frame, camera_matrix, dist_coeffs, rvec, tvec, 0.05)
            cv2.putText(frame, f"Distance: {distance:.2f} m", 
                        (int(corner[0][0][0]), int(corner[0][0][1]) - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)

    # Show the result
    cv2.imshow("ArUco Distance Measurement", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()

