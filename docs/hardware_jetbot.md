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

`jetbot_hardware_node`는 기본적으로 `auto` 모드입니다. 먼저 NVIDIA JetBot Python 패키지의 `jetbot.Robot`을 시도하고, 없으면 I2C PCA9685 MotorHAT 직접 제어로 fallback합니다. 이 fallback은 `smbus` Python 모듈이 없어도 `/dev/i2c-*` ioctl로 동작합니다.

컨테이너 안에서 확인:

```bash
python3 -c "from jetbot import Robot; print('jetbot ok')"
```

실패해도 `pca9685 motor backend ready` 로그가 나오면 모터 제어는 계속 동작할 수 있습니다. `pca9685 motor backend unavailable on bus=..., address=...`가 모든 bus/address에서 나오면 I2C 장치가 보이지 않는 상태입니다.

I2C 확인:

```bash
./scripts/host_docker_diag.sh
```

MotorHAT/PCA9685가 보통 `0x60` 또는 `0x40`으로 보이면 정상입니다.

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
    motor_backend: "auto"
    motor_i2c_bus: -1
    motor_i2c_address: 0
    left_motor_channel: 1
    right_motor_channel: 2
    left_trim: 1.0
    right_trim: 1.0
```

## OLED와 LED

Waveshare JetBot AI Kit의 OLED는 SSD1306 계열 128x32 디스플레이입니다. 기본 설정은 I2C bus 자동 탐색입니다.

```yaml
jetbot_hardware_node:
  ros__parameters:
    oled_enabled: true
    oled_bus: -1
    status_led_pin: -1
```

상태 LED를 별도 GPIO에 연결했다면 `status_led_pin`을 Jetson BOARD pin 번호로 설정합니다. 사용하지 않으면 `-1`을 유지합니다.

I2C 확인:

```bash
./scripts/host_docker_diag.sh
```

OLED가 보통 `0x3c`로 보이면 정상입니다.

## 카메라

기본은 USB 카메라 `/dev/video0`입니다.

```yaml
camera_node:
  ros__parameters:
    camera_index: 0
    camera_mode: "usb"
    use_gstreamer: false
    force_v4l2: true
    width: 320
    height: 240
    fps: 5.0
    jpeg_quality: 45
```

CSI 카메라를 쓰면:

```yaml
camera_node:
  ros__parameters:
    camera_mode: "csi"
    use_gstreamer: true
```

CSI 카메라 컨테이너 실행에는 `/tmp/argus_socket` 접근과 NVIDIA 런타임이 필요할 수 있습니다. `scripts/host_docker_run.sh`는 `CCAI_CAMERA_MODE=csi`일 때 호스트에 `/tmp/argus_socket`이 있으면 자동으로 컨테이너에 마운트하고 `--runtime nvidia`를 사용합니다.

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=csi ./scripts/host_docker_run.sh
```

카메라 토픽 확인:

```bash
ros2 topic hz /image_raw/compressed
```

카메라 포맷 확인:

```bash
./scripts/host_docker_diag.sh
```

`camera_backend: "auto"`에서 `camera_mode: "usb"`는 V4L2 후보만 재시도합니다. `camera_mode: "csi"`는 `nvarguscamerasrc` CSI 후보만 사용하므로 USB `/dev/video0`와 섞이지 않습니다.

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
