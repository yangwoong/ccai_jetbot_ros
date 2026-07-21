# Connectivity Guide

호스트 `/home/roboat/work/ros2_ws`를 도커 내부 `/home/workspace`로 마운트하는 구성을 기준으로 설명합니다.

## 1. H200 vLLM API 키 설정

H200 서버에서 vLLM을 OpenAI 호환 API로 실행할 때 `VLLM_API_KEY`를 지정합니다.

```bash
cd /home/roboat/work/ros2_ws/ccai_jetbot_ros

export VLLM_API_KEY="CHANGE_ME_LONG_RANDOM_KEY"
export PORT=8000
export TENSOR_PARALLEL_SIZE=1

./scripts/start_h200_vllm.sh Qwen/Qwen3-VL-32B-Instruct
```

직접 실행할 때는 다음과 같습니다.

```bash
python3 -m vllm.entrypoints.openai.api_server \
  --host 0.0.0.0 \
  --port 8000 \
  --model Qwen/Qwen3-VL-32B-Instruct \
  --trust-remote-code \
  --limit-mm-per-prompt image=1 \
  --api-key "CHANGE_ME_LONG_RANDOM_KEY"
```

H200 방화벽에서 Jetson/로봇이 접근할 수 있도록 8000 포트를 엽니다.

```bash
sudo ufw allow from JETSON_IP to any port 8000 proto tcp
```

개발 중 전체 내부망에 열어야 하면 다음처럼 할 수 있지만, 운영에서는 권장하지 않습니다.

```bash
sudo ufw allow 8000/tcp
```

H200에서 로컬 확인:

```bash
curl -H "Authorization: Bearer CHANGE_ME_LONG_RANDOM_KEY" \
  http://127.0.0.1:8000/v1/models
```

Jetson 또는 Jetson 도커 안에서 확인:

```bash
curl -H "Authorization: Bearer CHANGE_ME_LONG_RANDOM_KEY" \
  http://H200_IP:8000/v1/models
```

## 2. Jetson 도커 `.env` 설정

호스트에서 `.env`를 만듭니다.

```bash
cd /home/roboat/work/ros2_ws/ccai_jetbot_ros
cp .env.example .env
```

내용 예시:

```bash
CCAI_VLLM_API_BASE_URL=http://H200_IP:8000/v1
CCAI_VLLM_API_KEY=CHANGE_ME_LONG_RANDOM_KEY
CCAI_VLLM_MODEL=Qwen/Qwen3-VL-32B-Instruct
CCAI_TELEGRAM_BOT_TOKEN=123456789:TELEGRAM_BOT_TOKEN
CCAI_TELEGRAM_ALLOWED_CHAT_ID=123456789
CCAI_OTA_MANIFEST_URL=
```

`scripts/run_patrol.sh`는 실행 시 저장소 루트의 `.env`를 자동으로 읽습니다.

## 3. 외부에서 도커 웹 채팅 접속

웹 채팅 노드는 기본적으로 컨테이너 안에서 `0.0.0.0:8080`으로 실행됩니다.

### 권장: host network

Jetson/로봇에서 가장 단순한 방식은 `--network host`입니다. 이 경우 컨테이너의 8080 포트가 곧 Jetson 호스트의 8080 포트입니다.

```bash
docker run -it --rm \
  --name ccai-jetbot \
  --network host \
  --privileged \
  -v /home/roboat/work/ros2_ws:/home/workspace \
  -w /home/workspace/ccai_jetbot_ros \
  osrf/ros:humble-ros-base \
  bash
```

컨테이너 안에서 실행:

```bash
cd /home/workspace/ccai_jetbot_ros
source /opt/ros/humble/setup.bash 2>/dev/null || source /opt/ros/humble/install/setup.bash
colcon build --symlink-install
source install/setup.bash
./scripts/run_patrol.sh
```

관리자 PC 브라우저에서 접속:

```text
http://JETSON_IP:8080
```

API 상태 확인:

```bash
curl http://JETSON_IP:8080/api/status
```

웹 API로 명령 전송:

```bash
curl -X POST http://JETSON_IP:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"patrol start"}'
```

### 대안: 포트 매핑

`--network host`를 쓰지 않는 경우에는 8080 포트를 명시적으로 매핑합니다.

```bash
docker run -it --rm \
  --name ccai-jetbot \
  -p 8080:8080 \
  --privileged \
  -v /home/roboat/work/ros2_ws:/home/workspace \
  -w /home/workspace/ccai_jetbot_ros \
  osrf/ros:humble-ros-base \
  bash
```

접속 주소는 동일합니다.

```text
http://JETSON_IP:8080
```

단, ROS2 DDS 통신까지 여러 컨테이너/호스트에 걸쳐 사용할 계획이면 host network가 더 단순합니다.

## 4. 텔레그램 연결 방법

### BotFather에서 봇 생성

1. 텔레그램에서 `@BotFather`를 엽니다.
2. `/newbot`을 입력합니다.
3. 봇 이름과 username을 정합니다.
4. 발급된 토큰을 복사합니다.

토큰 형식:

```text
123456789:AA...TOKEN...
```

### chat_id 확인

관리자 계정에서 만든 봇에게 아무 메시지나 보냅니다.

그 다음 Jetson, H200, 또는 인터넷 가능한 PC에서:

```bash
curl "https://api.telegram.org/botBOT_TOKEN/getUpdates"
```

응답에서 다음 값을 찾습니다.

```json
"chat":{"id":123456789}
```

이 숫자가 `CCAI_TELEGRAM_ALLOWED_CHAT_ID`입니다.

### `.env`에 등록

```bash
CCAI_TELEGRAM_BOT_TOKEN=BOT_TOKEN
CCAI_TELEGRAM_ALLOWED_CHAT_ID=123456789
```

컨테이너를 재시작하거나 노드를 다시 실행합니다.

```bash
cd /home/workspace/ccai_jetbot_ros
./scripts/run_patrol.sh
```

텔레그램에서 다음 명령을 보냅니다.

```text
status
patrol start
inspect entrance
patrol stop
go home
```

로봇 이벤트는 `/ccai/events`로 발행되고, `telegram_bridge_node`가 허용된 chat_id로 다시 전송합니다.

## 5. 네트워크 점검 순서

H200 API:

```bash
curl -H "Authorization: Bearer $CCAI_VLLM_API_KEY" \
  "$CCAI_VLLM_API_BASE_URL/models"
```

웹 채팅:

```bash
curl http://JETSON_IP:8080/api/status
```

텔레그램:

```bash
curl "https://api.telegram.org/bot$CCAI_TELEGRAM_BOT_TOKEN/getMe"
```

ROS 토픽:

```bash
ros2 topic echo /ccai/events
ros2 topic echo /ccai/status
```

## 6. 보안 주의

- 웹 채팅에는 아직 로그인 기능이 없습니다. 운영망에서는 VPN, 방화벽, 리버스 프록시 인증 중 하나로 보호해야 합니다.
- vLLM API는 `--api-key`를 반드시 설정합니다.
- 텔레그램은 `allowed_chat_id`를 설정해 지정 관리자만 명령할 수 있게 합니다.
- 8080 웹 포트와 8000 vLLM 포트를 인터넷 전체에 직접 노출하지 마세요.

