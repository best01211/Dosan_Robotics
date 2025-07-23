# TurtleBot3 Autonomous Driving (ROS2 기반)

본 프로젝트는 **TurtleBot3**를 활용한 ROS2 기반 자율 주행 시스템을 구축하고자 합니다.  
전체 구조는 기능 단위로 분리된 두 개의 워크스페이스로 구성되며, 상호 종속 관계를 통해 통합 실행이 가능합니다.

## 🗂️ 프로젝트 구성

- `rokeyracing_ws`: 상위 워크스페이스로, 전체 자율주행 시스템의 실행을 담당합니다.
- `rokeypj_ws`: 하위 워크스페이스로, 사용자 정의 기능 또는 특정 모듈 패키지를 포함합니다.

> `rokeyracing_ws`에서 최종 실행을 담당하며, `rokeypj_ws`의 기능들을 종속적으로 사용합니다.

## ⚙️ 종속 관계 설정

`rokeyracing_ws`가 `rokeypj_ws`를 참조하도록 설정하려면 다음 절차를 따릅니다:

1. `rokeypj_ws` 먼저 빌드
    ```bash
    cd ~/rokeypj_ws
    colcon build
    source install/setup.bash
    ```

2. `rokeyracing_ws`에서 `rokeypj_ws`를 참조한 후 빌드
    ```bash
    cd ~/rokeyracing_ws
    source ~/rokeypj_ws/install/setup.bash
    colcon build
    ```

## 📌 사용 시 주의 사항

- 각 워크스페이스는 독립적으로 `colcon build`가 가능해야 합니다.
- `source` 명령은 터미널을 새로 열 때마다 다시 실행되어야 하므로 `.bashrc`에 등록하는 것이 좋습니다:

    ```bash
    echo "source ~/rokeypj_ws/install/setup.bash" >> ~/.bashrc
    echo "source ~/rokeyracing_ws/install/setup.bash" >> ~/.bashrc
    source ~/.bashrc
    ```

