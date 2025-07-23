import cv2
import numpy as np

def detect_markers(image):
    aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_1000)
    parameters = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(aruco_dict, parameters)

    corners, ids, _ = detector.detectMarkers(image)

    if ids is not None:
        cv2.aruco.drawDetectedMarkers(image, corners, ids)

    return image, ids, corners

# 테스트용 코드
if __name__ == "__main__":
    cap = cv2.VideoCapture(0)  # 웹캠 사용
    while True:
        ret, frame = cap.read()
        if not ret:
            break

        frame, ids, corners = detect_markers(frame)
        cv2.imshow("ArUco Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
