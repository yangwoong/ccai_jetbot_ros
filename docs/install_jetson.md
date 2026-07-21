# Jetson Nano Installation Notes

## ROS2 Humble 호환성

Jetson Nano Dev Kit 4GB의 공식 JetPack 4.x 기반 OS는 Ubuntu 18.04입니다. ROS2 Humble은 Ubuntu 22.04 기준이라 공식 조합이 아닙니다.

권장 선택지는 다음 중 하나입니다.

- Ubuntu 22.04 계열 Jetson Nano 이미지에서 ROS2 Humble을 네이티브 설치
- JetPack 4.x 위에서 ROS2 Humble 컨테이너 사용
- 개발/검증은 Ubuntu 22.04 x86_64에서 진행하고, Jetson에는 `/cmd_vel`, 카메라, 센서 브리지만 배치

## 최소 설치 순서

```bash
sudo apt-get update
sudo apt-get install -y curl gnupg lsb-release
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
sudo apt-get update
sudo apt-get install -y ros-humble-ros-base python3-colcon-common-extensions
```

프로젝트 의존성:

```bash
./scripts/install_jetson_humble.sh
colcon build --symlink-install
```

## Waveshare JetBot 연결

이 저장소의 기본 제어 출력은 `/cmd_vel`입니다. Waveshare JetBot 모터 드라이버가 `/cmd_vel`을 직접 구독하지 않는 경우, JetBot 모터 API와 `/cmd_vel` 사이의 브리지 노드를 추가해야 합니다.

카메라는 `/image_raw/compressed` 토픽을 사용합니다. 실제 카메라 드라이버 토픽명이 다르면 `ccai_jetbot_patrol/config/robot.yaml`의 `image_topic`을 바꿉니다.

