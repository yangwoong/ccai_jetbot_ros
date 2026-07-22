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

Waveshare JetBot AI Kit은 CSI 카메라를 씁니다. 기본값은 CSI이고, `host_docker_run.sh`가 안전 모드가 아닐 때 `CCAI_CAMERA_MODE=csi`, `--runtime nvidia`, `--ipc host`, `-v /tmp:/tmp`를 자동으로 구성합니다. 별도 설정 없이 아래처럼 실행하면 됩니다.

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 ./scripts/host_docker_run.sh
```

USB 카메라를 쓰는 경우에만 `CCAI_CAMERA_MODE=usb`를 명시합니다.

```yaml
camera_node:
  ros__parameters:
    camera_index: 0
    camera_device: ""
    camera_mode: "csi"
    use_gstreamer: true
    force_v4l2: true
    width: 320
    height: 240
    fps: 5.0
    jpeg_quality: 45
```

### CSI가 안 나올 때 가장 먼저 확인할 것: `nvargus-daemon`

Docker 안에서 CSI가 매번 `Failed to create CaptureSession`으로 실패하는데 `gst-launch-1.0 nvarguscamerasrc ...`를 호스트에서 직접 실행해도 똑같이 실패한다면, 이건 Docker/ROS 코드 문제가 아니라 호스트의 `nvargus-daemon` 설정 문제일 수 있습니다. `/etc/systemd/system/nvargus-daemon.service`에 `Environment="enableCamInfiniteTimeout=1"`이 들어있으면 일부 L4T(R32.7.1 확인됨) + imx219 조합에서 Argus의 ViCsi 오픈 단계가 깨져서 세션 생성이 항상 실패합니다. 이 경우 해당 줄을 지우고 재적용하면 해결됩니다.

```bash
sudo cp /etc/systemd/system/nvargus-daemon.service /etc/systemd/system/nvargus-daemon.service.bak
sudo sed -i '/Environment="enableCamInfiniteTimeout=1"/d' /etc/systemd/system/nvargus-daemon.service
sudo systemctl daemon-reload
sudo systemctl restart nvargus-daemon
```

확인:

```bash
gst-launch-1.0 nvarguscamerasrc num-buffers=1 sensor-mode=3 ! 'video/x-raw(memory:NVMM),width=816,height=616,format=NV12,framerate=30/1' ! nvvidconv ! jpegenc ! filesink location=/tmp/argus_test.jpg -e
ls -l /tmp/argus_test.jpg
```

이 파일 크기가 0보다 크면 CSI 자체는 정상입니다. 이건 이 Jetson의 systemd 설정 문제라 저장소 코드에는 반영되지 않으니, 재설치/재플래시 시 다시 나타날 수 있다는 점을 기억하세요.

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

CSI 카메라 컨테이너 실행에는 Argus socket 접근과 NVIDIA 런타임이 필요합니다. `scripts/host_docker_run.sh`는 `CCAI_CAMERA_MODE=csi`일 때 항상 `/tmp:/tmp`를 마운트하고 `--ipc host`, `--runtime nvidia`를 사용합니다(이게 기본값이자 유일한 마운트 방식입니다). `/tmp/argus_socket`만 단일 마운트하면 `nvargus-daemon` 재시작 후 socket inode가 바뀌었을 때 컨테이너에서 stale socket을 볼 수 있어서, 전체 `/tmp`를 마운트하는 방식으로 통일했습니다.

CSI 파이프라인 기본값은 NVIDIA JetBot의 `sensor-mode=3`, `816x616`, `NV12`, `30fps` 설정을 따릅니다. 센서 방향이 뒤집혀 있으면 `CCAI_CSI_FLIP_METHOD=2`처럼 `nvvidconv flip-method` 값을 지정합니다.

JetBot 공개 코드와 동일하게 open probe 단계에서는 첫 프레임이 읽히면 성공으로 봅니다. 순찰 중 무효 프레임 필터는 별도로 유지됩니다.

CSI만 직접 확인:

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=disabled DOCKER_RUNTIME_NVIDIA=1 ./scripts/host_docker_run.sh
./scripts/host_csi_probe.sh
```

CSI 모드는 기본적으로 10회만 open/probe를 재시도합니다. Argus 로그에 `Sensor could not be opened` 또는 `V4L2Device not available`이 반복되면 CSI 센서/케이블/Jetson 드라이버 쪽 문제입니다. 특히 `Failed to create CaptureSession`이 Docker 안팎을 가리지 않고 매번 나면, 위의 "CSI가 안 나올 때 가장 먼저 확인할 것" 절의 `nvargus-daemon` `enableCamInfiniteTimeout` 문제부터 확인하세요 — 대부분 이게 원인입니다.

### (드문 경우) 호스트 MJPEG 브리지로 우회

`nvargus-daemon`을 고쳐도 Docker 안에서만 CSI가 계속 실패하고 호스트에서는 되는 경우에 한해서만 아래 우회 경로를 씁니다. 이 경로는 호스트에서 별도 프로세스(`host_csi_mjpeg_server.py`)로 CSI를 열어 MJPEG/스냅샷으로 서비스하고, Docker ROS는 그 URL을 구독합니다. 정상적인 경우엔 필요 없는 추가 구성요소이니 CSI 직접 모드가 되면 이 경로는 쓰지 마세요.

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

`curl` 저장 옵션은 ASCII 하이픈 2개인 `--output`을 써야 합니다. `—output`처럼 긴 대시가 들어가면 URL 파싱 오류가 납니다.

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
