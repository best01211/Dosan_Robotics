# TurtleBot3 Autonomous Driving (ROS2 기반)

본 프로젝트는 **TurtleBot3**를 활용한 ROS2 기반 자율 주행 시스템을 구축하고자 합니다.  
전체 구조는 기능 단위로 분리된 두 개의 워크스페이스로 구성되며, 상호 종속 관계를 통해 통합 실행이 가능합니다.

---

## 🗂️ 프로젝트 구성

### 🔸 `rokeyracing_ws`

- 상위 워크스페이스로 전체 자율주행 시스템의 실행을 담당합니다.
- 하위 기능 워크스페이스(`rokeypj_ws`)의 패키지를 참조하여 통합 실행을 수행합니다.

### 🔹 `rokeypj_ws`

- 하위 워크스페이스로 센서 드라이버, 인식 기능, 로봇팔 제어, 외부 연동 모듈 등 개별 기능을 포함합니다.
- 각 패키지는 독립 실행 가능하며, `rokeyracing_ws`와 연동되어 실제 동작에 참여합니다.

---

## 📦 `rokeypj_ws` 패키지 구성

| 패키지명                   | 주요 역할                         | 주요 기능 요약 |
|---------------------------|-----------------------------------|----------------|
| `aruco_yolo`              | 마커 및 객체 인식                | ArUco 기반 위치 인식 및 YOLO 객체 검출, 로봇팔 제어 트리거 |
| `fsd_pkg`                 | Full Self Driving 확장           | 고급 자율주행 알고리즘, 경로 계획 및 센서 융합 |
| `ld08_driver`             | LD08 LiDAR 드라이버              | `/scan` 토픽 퍼블리시, SLAM/장애물 회피 입력 |
| `sample_pkg`             | 예제 및 기능 테스트              | ROS2 노드 및 launch 테스트용 코드 |
| `turtlebot3_manipulation` | TurtleBot3 로봇팔 제어           | 하드웨어 bring-up, 관절 제어, 초기화 노드 |
| `turtlebot_cosmo_interface` | 외부 장비와 연동 인터페이스    | 외부 시스템과 통신 (용도 커스텀 가능) |
| `turtlebot_moveit`        | 로봇팔 경로 계획 및 시각화       | MoveIt 기반 시뮬레이션 및 trajectory 생성 |

---

## 🧪 주요 실행 명령어

```bash
# 1. TurtleBot3 매니퓰레이터 하드웨어 실행
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

# 2. MoveIt + RViz 로봇팔 제어
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

# 3. 로봇팔 컨트롤러 테스트
ros2 run turtlebot_moveit turtlebot_arm_controller

# 4. ArUco + YOLO 기반 Pick-and-Place 자동화
ros2 run aruco_yolo pick_and_place
