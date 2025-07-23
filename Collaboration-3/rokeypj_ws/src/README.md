# rokeypj_ws (TurtleBot3 기능 모듈 워크스페이스)

`rokeypj_ws`는 TurtleBot3 자율주행 프로젝트의 하위 워크스페이스로, 센서 드라이버, 인식 기능, 로봇팔 제어, 외부 연동 모듈 등 다양한 기능 패키지를 포함합니다.  
해당 워크스페이스는 `rokeyracing_ws`와 종속 관계로 연동되며, 각 패키지는 독립적으로 실행할 수 있습니다.

---

## 🧩 패키지 구성 및 기능

| 패키지명 | 주요 역할 | 설명 |
|----------|-----------|------|
| `aruco_yolo` | 마커 및 객체 인식 | ArUco 마커 기반 위치 인식, YOLO 객체 검출, 주행 판단 및 로봇팔 제어 트리거 |
| `fsd_pkg` | 고급 자율주행 확장 | 경로 계획, 동적 장애물 회피, 센서 융합 등 고속/복잡 경로 자율주행용 |
| `ld08_driver` | LiDAR 드라이버 | LD08 센서 ROS2 연결 및 `/scan` 퍼블리시. SLAM 및 장애물 회피에 활용 |
| `sample_pkg` | 테스트 및 예제 | ROS2 노드, launch 파일 구조 실험용 코드 포함. 빠른 디버깅 및 기능 테스트 |
| `turtlebot3_manipulation` | 로봇팔 제어 | 하드웨어 bring-up, 컨트롤러 설정, 관절 제어 등 매니퓰레이터 초기화 및 실행 |
| `turtlebot_cosmo_interface` | 외부 연동 | 외부 장비 및 시스템과 데이터 통신. 산업 응용 또는 커스텀 목적 |
| `turtlebot_moveit` | 경로 계획 및 시각화 | MoveIt 기반 trajectory 생성, RViz 연동 시뮬레이션, 로봇팔 조작 planning group 구성 |

---

## 🚀 실행 명령어

```bash
# 1. TurtleBot3 매니퓰레이터 하드웨어 실행
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

# 2. MoveIt 및 RViz 실행
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

# 3. 로봇팔 컨트롤러 실행 (C++)
ros2 run turtlebot_moveit turtlebot_arm_controller

# 4. Pick-and-Place 자동화 실행 (Python)
ros2 run aruco_yolo pick_and_place
