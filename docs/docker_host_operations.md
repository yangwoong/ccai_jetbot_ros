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

기본값:

```bash
CONTAINER_NAME=ccai-jetbot
HOST_WS=/home/roboat/work/ros2_ws
REPO_DIR=ccai_jetbot_ros
IMAGE=osrf/ros:humble-ros-base
```

다른 이미지나 컨테이너명을 쓰려면:

```bash
CONTAINER_NAME=jetbot-patrol IMAGE=my-ros-humble:latest ./scripts/host_docker_run.sh
```

웹 채팅:

```text
http://JETSON_IP:8080
```

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
