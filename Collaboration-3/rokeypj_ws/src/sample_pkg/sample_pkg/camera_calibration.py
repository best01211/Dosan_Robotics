import cv2
import numpy as np
import glob

# Chessboard settings
chessboard_size = (9, 6)  # Adjust based on your printed chessboard
square_size = 20  # Square size in mm

# 이미지를 그레이스케일로 변환
gray = None 

# Prepare object points
objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

# Arrays to store object points and image points
objpoints = []
imgpoints = []

# Load images
images = glob.glob("calibration_images/*.jpg")

for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret:  # 체스보드가 감지된 경우에만 추가
        objpoints.append(objp)
        imgpoints.append(corners)
    else:
        print("Chessboard not detected in this image.")

    if ret:
        objpoints.append(objp)
        imgpoints.append(corners)

        print(f"Number of images used: {len(objpoints)}")
        print(f"Number of detected points: {len(imgpoints)}")

        if len(objpoints) == 0 or len(imgpoints) == 0:
            print("Error: No valid images with detected points.")

        cv2.drawChessboardCorners(img, chessboard_size, corners, ret)
        cv2.imshow('Chessboard', img)
        cv2.waitKey(500)

cv2.destroyAllWindows()

# Camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs)

# Save calibration results
np.save("camera_matrix.npy", camera_matrix)
np.save("dist_coeffs.npy", dist_coeffs)

