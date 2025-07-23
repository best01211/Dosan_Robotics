import cv2
import os

cap = cv2.VideoCapture(0)
count = 0
save_path = "calibration_images"
os.makedirs(save_path, exist_ok=True)

while count < 20:  # Capture 20 images
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow('Calibration Frame', frame)

    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):  # Press 's' to save the image
        filename = f"{save_path}/calib_{count}.jpg"
        cv2.imwrite(filename, frame)
        print(f"Saved {filename}")
        count += 1
    elif key == ord('q'):  # Press 'q' to exit
        break

cap.release()
cv2.destroyAllWindows()

