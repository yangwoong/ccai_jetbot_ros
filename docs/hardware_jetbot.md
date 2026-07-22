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
    camera_device: ""
    camera_mode: "usb"
    use_gstreamer: false
    force_v4l2: true
    width: 320
    height: 240
    fps: 5.0
    jpeg_quality: 45
```

USB 카메라를 따로 연결해서 사용할 때:

```bash
# 안전 모드에서 USB 카메라만 실행
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=usb ./scripts/host_docker_run.sh

# 장치가 /dev/video1 등으로 잡히면 직접 지정
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=usb CCAI_CAMERA_DEVICE=/dev/video1 ./scripts/host_docker_run.sh
```

USB 장치 확인:

```bash
./scripts/host_docker_diag.sh
CCAI_CAMERA_DEVICE=/dev/video0 ./scripts/host_camera_probe.sh
```

CSI 카메라를 쓰면:

```yaml
camera_node:
  ros__parameters:
    camera_mode: "csi"
    use_gstreamer: true
    csi_sensor_mode: 3
    csi_capture_width: 816
    csi_capture_height: 616
    csi_fps: 30
```

CSI 카메라 컨테이너 실행에는 Argus socket 접근과 NVIDIA 런타임이 필요합니다. `scripts/host_docker_run.sh`는 `CCAI_CAMERA_MODE=csi`일 때 기본적으로 `/tmp:/tmp`를 마운트하고 `--ipc host`, `--runtime nvidia`를 사용합니다. `/tmp/argus_socket` 단일 마운트는 `nvargus-daemon` 재시작 후 socket inode가 바뀌면 컨테이너에서 stale socket을 볼 수 있습니다.

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=csi ./scripts/host_docker_run.sh
```

CSI 파이프라인 기본값은 NVIDIA JetBot의 `sensor-mode=3`, `816x616`, `NV12`, `30fps` 설정을 따릅니다. 센서 방향이 뒤집혀 있으면 `CCAI_CSI_FLIP_METHOD=2`처럼 `nvvidconv flip-method` 값을 지정합니다.

JetBot 공개 코드와 동일하게 open probe 단계에서는 첫 프레임이 읽히면 성공으로 봅니다. 순찰 중 무효 프레임 필터는 별도로 유지됩니다.

CSI만 직접 확인:

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=disabled DOCKER_RUNTIME_NVIDIA=1 CCAI_ARGUS_MOUNT_MODE=tmp ./scripts/host_docker_run.sh
./scripts/host_csi_probe.sh
```

CSI 모드는 기본적으로 10회만 open/probe를 재시도합니다. Argus 로그에 `Sensor could not be opened` 또는 `V4L2Device not available`이 반복되면 CSI 센서/케이블/Jetson 드라이버 쪽 문제로 보고 USB 카메라 운용을 먼저 확인하세요.

Docker 내부 CSI가 `Failed to create CaptureSession` 또는 `opened=true/read=false`로 실패하지만 호스트의 JetBot 공개 코드는 영상이 나오는 경우, CSI를 호스트에서 열고 Docker ROS는 MJPEG URL로 받습니다.

```bash
# 호스트에서 CSI를 MJPEG로 송출
./scripts/host_csi_mjpeg_stop.sh
./scripts/host_csi_mjpeg_start.sh

# Docker ROS 카메라 노드는 URL을 구독
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=url ./scripts/host_docker_run.sh
```

확인:

```bash
curl http://127.0.0.1:8090/health
curl http://127.0.0.1:8090/snapshot.jpg --output /tmp/csi.jpg
curl http://127.0.0.1:8080/api/camera.jpg --output /tmp/jetbot.jpg
```

호스트의 JetBot 공개 코드가 동작한다면 MJPEG 서버는 기본적으로 `jetbot.Camera`를 먼저 시도하고, 실패하면 OpenCV GStreamer로 fallback합니다. JetBot 백엔드만 강제하려면:

```bash
CCAI_CSI_HOST_BACKEND=jetbot ./scripts/host_csi_mjpeg_start.sh
```

카메라 토픽 확인:

```bash
ros2 topic hz /image_raw/compressed
```

카메라 포맷 확인:

```bash
./scripts/host_docker_diag.sh
```

`camera_backend: "auto"`에서 `camera_mode: "usb"`는 V4L2 후보만 재시도합니다. `camera_mode: "csi"`는 `nvarguscamerasrc` CSI 후보만 사용하므로 USB `/dev/video0`와 섞이지 않습니다. USB 장치 번호가 바뀌면 `CCAI_CAMERA_DEVICE=/dev/videoN`으로 고정하세요.

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
