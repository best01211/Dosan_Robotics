import numpy as np
import cv2
import pickle
import time

def view_camera_with_calibration():
    # 캘리브레이션 파일 로드
    try:
        with open('camera_calibration.pkl', 'rb') as f:
            calibration_data = pickle.load(f)
        
        camera_matrix = calibration_data['camera_matrix']
        dist_coeffs = calibration_data['dist_coeffs']
        
        print("캘리브레이션 파일을 성공적으로 로드했습니다.")
        print(f"카메라 행렬:\n{camera_matrix}")
        print(f"왜곡 계수:\n{dist_coeffs}")
    except Exception as e:
        print(f"캘리브레이션 파일을 로드할 수 없습니다: {e}")
        return
    
    # 카메라 열기
    cap = cv2.VideoCapture(0)
    
    if not cap.isOpened():
        print("카메라를 열 수 없습니다.")
        return
    
    # 초기 프레임을 읽어서 이미지 크기 확인
    ret, frame = cap.read()
    if not ret:
        print("프레임을 읽을 수 없습니다.")
        cap.release()
        return
    
    # 화면에 표시할 옵션 설정
    show_mode = 0  # 0: 왜곡 보정만, 1: 원본만, 2: 원본+왜곡 보정
    grid_display = False  # 그리드 표시 여부
    
    print("\n키 안내:")
    print("  'm': 표시 모드 변경 (왜곡 보정 / 원본 / 양쪽 모두)")
    print("  'g': 그리드 표시 토글")
    print("  's': 현재 프레임 저장")
    print("  'q' 또는 'ESC': 종료")
    
    # FPS 계산을 위한 변수
    prev_frame_time = 0
    new_frame_time = 0
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("프레임을 읽을 수 없습니다.")
            break
        
        # 현재 시간 (FPS 계산용)
        new_frame_time = time.time()
        fps = 1/(new_frame_time - prev_frame_time) if prev_frame_time > 0 else 0
        prev_frame_time = new_frame_time
        
        # 왜곡 보정된 이미지 계산
        undistorted = cv2.undistort(frame, camera_matrix, dist_coeffs, None, camera_matrix)
        
        # 그리드 추가 (옵션)
        if grid_display:
            # 원본 이미지에 그리드 추가
            h, w = frame.shape[:2]
            grid_size = 50
            for i in range(0, h, grid_size):
                cv2.line(frame, (0, i), (w, i), (0, 255, 0), 1)
            for i in range(0, w, grid_size):
                cv2.line(frame, (i, 0), (i, h), (0, 255, 0), 1)
                
            # 보정된 이미지에 그리드 추가
            for i in range(0, h, grid_size):
                cv2.line(undistorted, (0, i), (w, i), (0, 255, 0), 1)
            for i in range(0, w, grid_size):
                cv2.line(undistorted, (i, 0), (i, h), (0, 255, 0), 1)
        
        # FPS 정보 추가
        fps_text = f"FPS: {fps:.1f}"
        cv2.putText(frame, fps_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        cv2.putText(undistorted, fps_text, (10, 30), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 모드에 따라 다른 화면 표시
        if show_mode == 0:
            # 왜곡 보정된 이미지만 표시
            cv2.imshow('Undistorted Camera', undistorted)
            cv2.setWindowTitle('Undistorted Camera', '왜곡 보정된 이미지')
        elif show_mode == 1:
            # 원본 이미지만 표시
            cv2.imshow('Undistorted Camera', frame)
            cv2.setWindowTitle('Undistorted Camera', '원본 이미지')
        else:
            # 원본과 왜곡 보정된 이미지를 나란히 표시
            h, w = frame.shape[:2]
            combined = np.hstack((frame, undistorted))
            
            # 이미지가 너무 크면 크기 조정
            if combined.shape[1] > 1280:
                scale = 1280 / combined.shape[1]
                combined = cv2.resize(combined, None, fx=scale, fy=scale)
            
            cv2.imshow('Undistorted Camera', combined)
            cv2.setWindowTitle('Undistorted Camera', '원본 vs 왜곡 보정')
            
            # 구분선 추가
            cv2.line(combined, (w, 0), (w, h), (0, 0, 255), 2)
            
            # 레이블 추가
            cv2.putText(combined, "원본", (10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            cv2.putText(combined, "왜곡 보정", (w + 10, 60), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # 키 입력 처리
        key = cv2.waitKey(1) & 0xFF
        
        if key == ord('q') or key == 27:  # 'q' 또는 ESC
            break
        elif key == ord('m'):  # 'm' 키로 표시 모드 변경
            show_mode = (show_mode + 1) % 3
            mode_names = ["왜곡 보정된 이미지", "원본 이미지", "원본 vs 왜곡 보정"]
            print(f"표시 모드 변경: {mode_names[show_mode]}")
        elif key == ord('g'):  # 'g' 키로 그리드 표시 토글
            grid_display = not grid_display
            print(f"그리드 표시: {'켜짐' if grid_display else '꺼짐'}")
        elif key == ord('s'):  # 's' 키로 현재 프레임 저장
            timestamp = time.strftime("%Y%m%d_%H%M%S")
            if show_mode == 0 or show_mode == 2:
                undist_filename = f"undistorted_{timestamp}.jpg"
                cv2.imwrite(undist_filename, undistorted)
                print(f"왜곡 보정된 이미지 저장: {undist_filename}")
            
            if show_mode == 1 or show_mode == 2:
                orig_filename = f"original_{timestamp}.jpg"
                cv2.imwrite(orig_filename, frame)
                print(f"원본 이미지 저장: {orig_filename}")
    
    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

# 실행
if __name__ == "__main__":
    view_camera_with_calibration()