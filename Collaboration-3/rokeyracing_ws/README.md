# rokeyracing_ws (TurtleBot3 자율주행 통합 워크스페이스)

`rokeyracing_ws`는 TurtleBot3의 자율주행 미션과 매니퓰레이터 제어를 통합한 ROS2 기반 워크스페이스입니다.  
카메라 보정, 이미지 인식, 주행 제어, 로봇팔 조작 등을 하나의 시스템으로 구성하여 복잡한 오토레이스 환경을 자동으로 주행하고 물체를 인식 및 조작할 수 있도록 설계되었습니다.

---

## 🧩 주요 구성 패키지

| 패키지명 | 주요 역할 | 세부 기능 |
|----------|-----------|-----------|
| `turtlebot3_autorace_camera` | 카메라 영상 처리 | 영상 캡처, intrinsic/extrinsic 보정, 이미지 투영 |
| `turtlebot3_autorace_detect` | 비전 인식 기능 | 차선, 정지선, 표지판 등 탐지 |
| `turtlebot3_autorace_mission` | 주행 제어 | 차선 주행, 회전, 정지 등 제어 로직 |
| `turtlebot3_autorace_tracking` | 객체 추적 | 비전 기반 대상 추적 알고리즘 |
| `turtlebot3_fsd` | 고급 자율주행 확장 | 경로 계획, 센서 융합 등의 FSD 기능 |

---

## 🚀 실행 명령어 요약

```bash
# 1. TurtleBot3 하드웨어 및 매니퓰레이터 실행
ros2 launch turtlebot3_manipulation_bringup hardware.launch.py

# 2. MoveIt + RViz 로봇팔 제어 실행
ros2 launch turtlebot3_manipulation_moveit_config moveit_core.launch.py

# 3. 카메라 실행 (해상도 320x240, 포트 0 기준)
ros2 launch turtlebot3_autorace_camera camera.launch.py video_port:=0 image_width:=320 image_height:=240

# 4. 카메라 보정
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_camera extrinsic_camera_calibration.launch.py

# 5. 차선 감지
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py

# 6. 주행 제어 (차선 중심 기반)
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
````

---

## 🛰 시스템 구성 요약

### 🔷 ① MoveIt & 매니퓰레이터 블록

* **노드**: `move_group`
* **토픽**: `/move_group/goal`, `/display_planned_path`
* **기능**: 로봇팔의 경로 계획 및 실행

### 🔷 ② Arm Controller 블록

* **노드**: `arm_controller`, `joint_state_broadcaster`
* **토픽**: `/arm_controller/joint_trajectory`, `/joint_states`
* **기능**: 제어 trajectory 실행 및 관절 피드백

### 🔷 ③ TF Broadcaster

* **노드**: `robot_state_publisher`
* **토픽**: `/tf`, `/tf_static`
* **기능**: 좌표계 정보 공유 및 통합

### 🔷 ④ 비전 인식 파이프라인

* **입력**: `/camera/image_raw/compressed`
* **처리 흐름**:

  ```text
  /camera/image_raw/compressed
       ↓ republish
  /camera/image
       ↓ Debayer
  /camera/image_rect
       ↓ Rectify
  /camera/image_rect_color
       ↓ Projection
  /camera/image_projected
  ```
* **출력**: `/camera/image_projected` → `detect_lane` 입력 이미지

---

## 🔄 전체 흐름도

```
[Camera Input]
    ↓
[camera.launch.py]
    ↓
[Projection & Rectification]
    ↓
[/camera/image_projected]
    ↓
[detect_lane.launch.py]
    ↓
[차선 중심 좌표 계산]
    ↓
[control_lane.launch.py]
    ↓
[주행 제어 명령 → TurtleBot3 실행]
```

---

## 📌 유의 사항

* `/camera/image_projected`는 반드시 카메라 보정이 완료된 투영 이미지여야 인식 정확도가 향상됩니다.
* `autorace_mission/control_lane.py`에서 주차, 정지선 감지 등 전체 주행 로직을 통합 처리합니다.
* 주행 및 팔 제어를 동시에 수행할 경우, `/tf` 및 `/joint_states` 연동이 핵심 트리거로 작용합니다.

---

