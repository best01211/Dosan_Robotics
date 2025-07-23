import cv2
import cv2.aruco as aruco
import numpy as np

def generate_aruco_marker(marker_id=0, marker_size=200, dict_type=aruco.DICT_6X6_250, output_path='aruco_marker.png'):
    """
    Aruco 마커를 생성하고 저장하는 함수
    :param marker_id: 생성할 마커 ID (0~249)
    :param marker_size: 마커의 크기 (픽셀 단위)
    :param dict_type: 사용할 Aruco 마커 딕셔너리
    :param output_path: 저장할 파일 경로
    """
    # Aruco 딕셔너리 로드
    aruco_dict = aruco.getPredefinedDictionary(dict_type)
    
    # Aruco 마커 생성
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = aruco.generateImageMarker(aruco_dict, marker_id, marker_size)
    
    # 마커 이미지 저장
    cv2.imwrite(output_path, marker_image)
    print(f"Aruco 마커 (ID: {marker_id})가 {output_path}에 저장되었습니다.")

if __name__ == "__main__":
    generate_aruco_marker(marker_id=23, marker_size=300, output_path='aruco_marker_23.png')
