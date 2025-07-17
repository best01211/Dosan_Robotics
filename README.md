```markdown
# 🧠 Dosan Robotics – TurtleBot3 자율주행 시뮬레이션 프로젝트

본 프로젝트는 ROS 2 (Humble)와 Gazebo 시뮬레이터 기반의 TurtleBot3 자율주행 기능을 구현하고 시뮬레이션하는 워크스페이스입니다. 다양한 센서 기반 감지 기능과 미션 수행 로직을 포함하여, 실제 로봇 적용을 위한 기초 프레임워크를 구성하였습니다.

---

## 🚀 기술 스택

- 운영체제: Ubuntu 22.04
- 로봇 플랫폼: TurtleBot3 (Burger / Burger Cam)
- 미들웨어: ROS 2 Humble
- 시뮬레이터: Gazebo
- 주요 패키지:
  - `turtlebot3_autorace`: 차선/신호/표지판 감지, 미션 수행
  - `turtlebot3_simulations`: Gazebo 환경 정의
  - `turtlebot3`: 실제 로봇 운용 코드
  - `DynamixelSDK`: 모터 제어용 SDK

---

## 🗂 프로젝트 구조

```

turtlebot3\_ws/
├── src/
│   ├── turtlebot3/
│   ├── turtlebot3\_autorace/
│   ├── turtlebot3\_simulations/
│   └── turtlebot3\_msgs/
├── build/
├── install/
├── log/

````

---

## 🔧 실행 순서

### 1. 빌드 및 환경 설정

```bash
cd ~/turtlebot3_ws
colcon build --symlink-install
source install/setup.bash
````

### 2. 모델 및 환경 변수 설정

```bash
export TURTLEBOT3_MODEL=burger_cam
export ROS_DOMAIN_ID=30
```

### 3. 시뮬레이터 실행

```bash
ros2 launch turtlebot3_gazebo turtlebot3_autorace_2020.launch.py
```

---

## 🧠 자율주행 모듈 예시

### 차선 감지 및 추적

```bash
ros2 launch turtlebot3_autorace_detect detect_lane.launch.py calibration_mode:=True
ros2 launch turtlebot3_autorace_mission control_lane.launch.py
```

### 교통 표지판 인식

```bash
ros2 launch turtlebot3_autorace_camera intrinsic_camera_calibration.launch.py
ros2 launch turtlebot3_autorace_detect detect_sign.launch.py mission:=SELECT_MISSION
```

---

## 📎 참고 자료

* [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/overview/)
* [ROS 2 Documentation](https://docs.ros.org/en/humble/index.html)

---

## 👨‍💻 개발자 정보

* 작성자: best01211
* 이메일: [jinwoobest.andy01@gmail.com](mailto:jinwoobest.andy01@gmail.com)
* GitHub: [https://github.com/best01211](https://github.com/best01211)

```
```
