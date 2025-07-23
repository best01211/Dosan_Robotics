import numpy as np
import cv2
import glob
import pickle

def calibrate_camera():
    # 체스보드의 내부 코너 개수 정의
    chessboard_size = (9, 6)  # 체스보드 내부 코너 포인트의 행과 열 개수
    
    # 체스보드 코너 포인트를 저장하기 위한 배열 준비
    objpoints = []  # 3D 공간에서의 실제 포인트 좌표 (체스보드 코너)
    imgpoints = []  # 2D 이미지 상의 포인트 좌표
    
    # 3D 공간의 체스보드 포인트 준비 (Z=0인 평면)
    objp = np.zeros((chessboard_size[0] * chessboard_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
    
    # 체스보드 정사각형의 실제 크기 (미터 단위)
    square_size = 0.025  # 2.5cm 크기의 정사각형으로 가정 (필요에 따라 조정)
    objp = objp * square_size  # 실제 물리적 크기로 변환
    
    # 캘리브레이션 이미지 파일 목록 가져오기
    images = glob.glob('calibration_images/*.jpg')  # 캘리브레이션 이미지 폴더 경로 조정
    
    # 이미지 크기 저장을 위한 변수
    image_size = None
    
    print(f"{len(images)}개의 캘리브레이션 이미지를 찾았습니다.")
    
    # 각 이미지에서 체스보드 코너 찾기
    for idx, fname in enumerate(images):
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        if image_size is None:
            image_size = gray.shape[::-1]
        
        # 체스보드 코너 찾기
        ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
        
        # 코너를 찾은 경우, 객체 포인트와 이미지 포인트 저장
        if ret:
            objpoints.append(objp)
            
            # 코너 위치의 정확도 향상 (서브픽셀 정확도)
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            imgpoints.append(corners2)
            
            # 결과 시각화 (선택 사항)
            cv2.drawChessboardCorners(img, chessboard_size, corners2, ret)
            cv2.imshow('Chessboard Corners', img)
            cv2.waitKey(500)  # 0.5초 동안 표시
            
            print(f"이미지 {idx+1}/{len(images)}: 체스보드 코너를 성공적으로 찾았습니다.")
        else:
            print(f"이미지 {idx+1}/{len(images)}: 체스보드 코너를 찾지 못했습니다.")
    
    cv2.destroyAllWindows()
    
    # 카메라 캘리브레이션 수행
    print("카메라 캘리브레이션 수행 중...")
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, image_size, None, None
    )
    
    # 캘리브레이션 결과 확인 및 저장
    print("\n===== 캘리브레이션 결과 =====")
    print(f"캘리브레이션 정확도: {ret}")
    print(f"카메라 행렬:\n{camera_matrix}")
    print(f"왜곡 계수:\n{dist_coeffs}")
    
    # 재투영 오차 계산
    mean_error = 0
    for i in range(len(objpoints)):
        imgpoints_reprojected, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
        error = cv2.norm(imgpoints[i], imgpoints_reprojected, cv2.NORM_L2) / len(imgpoints_reprojected)
        mean_error += error
    
    print(f"평균 재투영 오차: {mean_error/len(objpoints)}")
    
    # 결과를 딕셔너리로 구성
    calibration_results = {
        'camera_matrix': camera_matrix,
        'dist_coeffs': dist_coeffs,
        'rvecs': rvecs,
        'tvecs': tvecs,
        'image_size': image_size,
        'reprojection_error': mean_error/len(objpoints)
    }
    
    # pkl 파일로 저장
    with open('camera_calibration.pkl', 'wb') as f:
        pickle.dump(calibration_results, f)
    
    print("캘리브레이션 결과가 'camera_calibration.pkl' 파일에 저장되었습니다.")
    
    return calibration_results

# 함수 실행
if __name__ == "__main__":
    calibration_results = calibrate_camera()

# 저장된 캘리브레이션 결과를 로드하는 함수 (필요시 사용)
def load_calibration():
    with open('camera_calibration.pkl', 'rb') as f:
        return pickle.load(f)