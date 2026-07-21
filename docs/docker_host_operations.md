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
```

`.env` 변경 후에는 컨테이너를 재시작합니다.

```bash
docker restart ccai-jetbot
```
