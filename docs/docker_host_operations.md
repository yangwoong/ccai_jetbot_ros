# Docker Host Operations

호스트에서 컨테이너에 직접 진입하지 않고 코드 업데이트, 빌드, 실행, 로그 확인을 하는 방법입니다.

기준 경로:

```text
호스트: /home/roboat/work/ros2_ws/ccai_jetbot_ros
도커:   /home/workspace/ccai_jetbot_ros
```

## 1. 최초 실행

호스트에서 실행합니다.

```bash
cd /home/roboat/work/ros2_ws/ccai_jetbot_ros
./scripts/host_docker_run.sh
```

기본 실행은 재부팅/장치 충돌을 피하기 위한 안전 모드입니다. 웹, LLM 제어, 텔레그램, OTA만 실행하고 모터, 카메라, 비전, VLM 이미지 분석은 꺼둡니다.

```bash
CCAI_SAFE_START=1 ./scripts/host_docker_run.sh
```

`host_docker_run.sh`는 기본적으로 컨테이너 시작 전에 워크스페이스를 다시 빌드합니다. 이미 빌드된 설치본을 그대로 쓰려면 다음처럼 실행합니다.

```bash
FORCE_BUILD_ON_RUN=0 ./scripts/host_docker_run.sh
```

안전 모드에서 재부팅이 멈추면 장치를 하나씩 켭니다.

```bash
# 1단계: 카메라 노드만 올리고 실제 장치는 열지 않음
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 ./scripts/host_docker_run.sh

# USB 카메라 장치 열기
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=usb ./scripts/host_docker_run.sh

# USB 장치 번호가 /dev/video1이면 직접 지정
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=usb CCAI_CAMERA_DEVICE=/dev/video1 ./scripts/host_docker_run.sh

# CSI 카메라 장치 열기
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=csi ./scripts/host_docker_run.sh

# 2단계: 카메라 + 비전
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_ENABLE_VISION=1 ./scripts/host_docker_run.sh

# 3단계: 모터/OLED 하드웨어 추가
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_ENABLE_VISION=1 CCAI_ENABLE_HARDWARE=1 ./scripts/host_docker_run.sh

# 전체 운영 모드
CCAI_SAFE_START=0 ./scripts/host_docker_run.sh
```

그래도 카메라나 I2C 장치 접근이 부족하면 마지막 단계에서만 privileged를 켭니다.

```bash
CCAI_SAFE_START=0 DOCKER_PRIVILEGED=1 ./scripts/host_docker_run.sh
```

Jetson CSI 카메라는 Argus를 사용하므로, USB `/dev/video0`와 분리해서 실행합니다. CSI 모드는 기본적으로 `/tmp:/tmp`를 마운트하고 `--ipc host`, `--runtime nvidia`를 사용합니다. `nvargus-daemon` 재시작 후 `/tmp/argus_socket` 단일 마운트가 stale 상태가 되는 문제를 피하기 위한 설정입니다.

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=csi ./scripts/host_docker_run.sh
```

CSI 센서는 NVIDIA JetBot 기본 파이프라인인 `sensor-mode=3`, `816x616`, `NV12`, `30fps`를 먼저 시도합니다. ROS 카메라 노드와 별개로 CSI만 확인하려면:

```bash
CCAI_SAFE_START=1 CCAI_ENABLE_CAMERA=1 CCAI_CAMERA_MODE=disabled DOCKER_RUNTIME_NVIDIA=1 CCAI_ARGUS_MOUNT_MODE=tmp ./scripts/host_docker_run.sh
./scripts/host_csi_probe.sh
```

USB 카메라를 연결한 뒤 ROS 카메라 노드와 별개로 OpenCV 장치 접근만 확인할 수 있습니다.

```bash
CCAI_CAMERA_DEVICE=/dev/video0 ./scripts/host_camera_probe.sh
```

이 명령에서 Jetson이 재부팅되면 ROS 코드 문제가 아니라 Jetson CSI/Argus/전원/커널 쪽 문제일 가능성이 큽니다. 재부팅 후 아래 로그를 수집합니다.

```bash
./scripts/host_docker_diag.sh
journalctl -k -b -1 --no-pager -n 200
journalctl -u nvargus-daemon -b -1 --no-pager -n 200
```

기본값:

```bash
CONTAINER_NAME=ccai-jetbot
HOST_WS=/home/roboat/work/ros2_ws
REPO_DIR=ccai_jetbot_ros
IMAGE=dustynv/ros:humble-desktop-l4t-r32.7.1
```

다른 이미지나 컨테이너명을 쓰려면:

```bash
CONTAINER_NAME=jetbot-patrol IMAGE=my-ros-humble:latest ./scripts/host_docker_run.sh
```

웹 채팅:

```text
http://JETSON_IP:8080
```

컨테이너에 `fastapi`/`uvicorn`이 없어도 웹채팅은 Python 표준 라이브러리 fallback 서버로 동작합니다.

JetBot 하드웨어와 카메라 설정은 `docs/hardware_jetbot.md`를 먼저 확인하세요.

## 2. 로그 확인

호스트에서:

```bash
./scripts/host_docker_logs.sh
```

또는:

```bash
docker logs -f ccai-jetbot
```

컨테이너 생성 이후의 과거 로그가 모두 섞이면 원인 판단이 어렵습니다. 최신 하드웨어 상태는 다음 진단 스크립트로 확인합니다.

```bash
./scripts/host_docker_diag.sh
```

재부팅 반복을 조사할 때는 먼저 안전 모드 컨테이너를 만든 뒤 진단합니다.

```bash
CCAI_SAFE_START=1 ./scripts/host_docker_run.sh
./scripts/host_docker_diag.sh
```

최근 로그만 직접 볼 때:

```bash
docker logs --since 3m ccai-jetbot
```

## 3. 코드 업데이트 + 빌드 + 재시작

호스트에서:

```bash
cd /home/roboat/work/ros2_ws/ccai_jetbot_ros
./scripts/host_docker_update.sh
```

기본 업데이트는 apt 설치를 건너뛰고 Python/ROS 워크스페이스 빌드만 수행합니다. 컨테이너에 OS 패키지가 처음부터 없을 때만 다음처럼 실행합니다.

```bash
INSTALL_OS_DEPS=1 ./scripts/host_docker_update.sh
```

JetPack 4.x 또는 Ubuntu 18.04 기반 컨테이너에서 ROS apt 저장소 키가 만료되어 `EXPKEYSIG F42ED6FBAB17C654`가 발생하면, 우선 `INSTALL_OS_DEPS` 없이 업데이트하세요. 이미 `colcon`, `opencv`, `PIL`이 설치되어 있으면 apt가 필요 없습니다.

이 스크립트가 수행하는 일:

1. 호스트 저장소에서 `git fetch origin`
2. 호스트 저장소에서 `git pull --ff-only`
3. 실행 중인 컨테이너 안에서 `./scripts/container_build.sh`
4. 컨테이너 재시작

컨테이너에 들어가지 않아도 됩니다.

## 4. 수동 API 점검

웹 상태:

```bash
curl http://127.0.0.1:8080/api/status
```

명령 전송:

```bash
curl -X POST http://127.0.0.1:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"status"}'
```

LLM 연결 상태 확인 명령:

```bash
curl -X POST http://127.0.0.1:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"llm status"}'
```

자연어 제어 예:

```bash
curl -X POST http://127.0.0.1:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"입구를 점검하고 이상 있으면 보고해"}'
```

직접 명령 예:

```bash
curl -X POST http://127.0.0.1:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"patrol start"}'

curl -X POST http://127.0.0.1:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"status"}'
```

## 5. 환경변수

호스트 저장소의 `.env`를 컨테이너 실행 시 자동으로 읽습니다.

```bash
CCAI_VLLM_API_BASE_URL=http://H200_IP:8000/v1
CCAI_VLLM_API_KEY=CHANGE_ME_LONG_RANDOM_KEY
CCAI_VLLM_MODEL=Qwen/Qwen3-VL-32B-Instruct
CCAI_TELEGRAM_BOT_TOKEN=...
CCAI_TELEGRAM_ALLOWED_CHAT_ID=...
ROS_LOCALHOST_ONLY=1
```

`.env` 변경 후에는 컨테이너를 재시작합니다.

```bash
docker restart ccai-jetbot
```

`ddsi_udp_conn_write to udp/... failed` 로그가 반복되면 외부 ROS2/DDS participant로 전송을 시도하다 실패하는 상태입니다. 이 로봇 프로젝트는 웹/텔레그램으로 외부와 통신하므로 기본값 `ROS_LOCALHOST_ONLY=1`을 권장합니다.
