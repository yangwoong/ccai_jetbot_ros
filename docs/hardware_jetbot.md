# JetBot Hardware Setup

Waveshare JetBot AI Kit 기준 하드웨어 연결 문서입니다.

## 지원 기능

- `/cmd_vel`을 JetBot 좌/우 모터 출력으로 변환
- 0.91인치 128x32 OLED에 IP와 모터 상태 표시
- 선택 GPIO 상태 LED heartbeat
- USB 또는 CSI 카메라를 320x240 JPEG로 발행
- 웹 채팅에서 작은 카메라 화면 표시

## 관련 ROS 노드

- `jetbot_hardware_node`
- `camera_node`
- `web_chat_node`

실행은 기본 launch에 포함되어 있습니다.

```bash
ros2 launch ccai_jetbot_patrol patrol.launch.py
```

## 모터

`jetbot_hardware_node`는 기본적으로 NVIDIA JetBot Python 패키지의 `jetbot.Robot`을 사용합니다.

컨테이너 안에서 확인:

```bash
python3 -c "from jetbot import Robot; print('jetbot ok')"
```

실패하면 현재 컨테이너에 JetBot Python 패키지가 없거나 I2C/GPIO 접근이 막힌 상태입니다. Waveshare JetBot 이미지 또는 NVIDIA JetBot 설치가 필요합니다.

수동 모터 테스트:

```bash
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.08}, angular: {z: 0.0}}"

ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"
```

방향이 반대면 `ccai_jetbot_patrol/config/robot.yaml`에서 `left_trim`, `right_trim`을 음수로 바꾸거나 좌우 값을 조정합니다.

```yaml
jetbot_hardware_node:
  ros__parameters:
    left_trim: 1.0
    right_trim: 1.0
```

## OLED와 LED

Waveshare JetBot AI Kit의 OLED는 SSD1306 계열 128x32 디스플레이입니다. 기본 설정은 I2C bus 0입니다.

```yaml
jetbot_hardware_node:
  ros__parameters:
    oled_enabled: true
    oled_bus: 0
    status_led_pin: -1
```

상태 LED를 별도 GPIO에 연결했다면 `status_led_pin`을 Jetson BOARD pin 번호로 설정합니다. 사용하지 않으면 `-1`을 유지합니다.

I2C 확인:

```bash
i2cdetect -y 0
```

OLED가 보통 `0x3c`로 보이면 정상입니다.

## 카메라

기본은 USB 카메라 `/dev/video0`입니다.

```yaml
camera_node:
  ros__parameters:
    camera_index: 0
    use_gstreamer: false
    width: 320
    height: 240
    fps: 5.0
    jpeg_quality: 45
```

CSI 카메라를 쓰면:

```yaml
camera_node:
  ros__parameters:
    use_gstreamer: true
```

CSI 카메라 컨테이너 실행에는 `/tmp/argus_socket` 접근이 필요합니다. `scripts/host_docker_run.sh`는 호스트에 `/tmp/argus_socket`이 있으면 자동으로 컨테이너에 마운트합니다.

카메라 토픽 확인:

```bash
ros2 topic hz /image_raw/compressed
```

웹 이미지 확인:

```bash
curl http://127.0.0.1:8080/api/camera.jpg --output /tmp/jetbot.jpg
```

관리자 PC 브라우저:

```text
http://JETSON_IP:8080
```

## 순찰 동작

`patrol_node`는 `patrol start` 시 전진과 제자리 회전을 반복합니다.

```yaml
patrol_node:
  ros__parameters:
    linear_speed: 0.12
    angular_speed: 0.35
    patrol_forward_seconds: 4.0
    patrol_turn_seconds: 1.2
```

값을 너무 크게 잡으면 실내에서 충돌 위험이 있습니다. 처음에는 바퀴를 들어 올린 상태로 테스트하세요.

