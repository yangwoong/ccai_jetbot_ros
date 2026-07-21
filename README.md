# CCAI JetBot ROS Patrol

ROS2 Humble 기반 자율순찰 로봇 프로젝트입니다. Jetson Nano Dev Kit 4GB + Waveshare JetBot AI Kit에서 순찰/임무 수행을 담당하고, H200 서버의 vLLM OpenAI 호환 API로 Qwen3-VL-32B 시각 추론을 호출합니다. 관리자는 웹 채팅, 텔레그램, ROS 로그/토픽으로 로봇과 소통합니다.

## 중요한 환경 전제

- ROS2 Humble은 Ubuntu 22.04가 기준입니다.
- Jetson Nano 공식 JetPack 4.x는 Ubuntu 18.04라서 Humble을 네이티브로 쓰기 어렵습니다.
- 권장 구성은 `Jetson Nano: Ubuntu 22.04 계열 이미지 또는 Humble 컨테이너`, `H200: vLLM 서버`, `관리자 PC: 웹 브라우저/텔레그램`입니다.
- Jetson Nano 4GB에서는 대형 VLM을 로컬 실행하지 않고, 이미지/프롬프트만 H200 vLLM API로 전송합니다.

## 구성

- `ccai_jetbot_patrol`: ROS2 Python 패키지
- `patrol_node`: 순찰 상태 머신, `/cmd_vel` 제어, 임무 수신
- `vlm_client_node`: 카메라 이미지를 vLLM OpenAI 호환 API로 분석
- `web_chat_node`: FastAPI 기반 관리자 웹 채팅/API
- `telegram_bridge_node`: 텔레그램 Bot API 브리지
- `ota_agent_node`: OTA manifest 확인 및 업데이트 계획/적용
- `launch/patrol.launch.py`: 통합 실행
- `config/*.yaml`: 로봇/순찰/연동 설정
- `scripts/*.sh`: Jetson, H200 설치와 실행 보조 스크립트
- `systemd/*.service`: 운영 서비스 예시

## 빠른 시작

```bash
cd /path/to/ccai_jetbot_ros
./scripts/install_jetson_humble.sh
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
ros2 launch ccai_jetbot_patrol patrol.launch.py
```

웹 채팅 기본 주소:

```text
http://JETSON_IP:8080
```

## H200 vLLM 예시

```bash
./scripts/start_h200_vllm.sh Qwen/Qwen3-VL-32B-Instruct
```

Jetson의 `config/robot.yaml`에서 다음 값을 H200 주소로 설정합니다.

```yaml
vlm_client:
  ros__parameters:
    api_base_url: "http://H200_IP:8000/v1"
    model: "Qwen/Qwen3-VL-32B-Instruct"
```

또는 systemd/.env 환경변수로 `CCAI_VLLM_API_BASE_URL`, `CCAI_VLLM_API_KEY`, `CCAI_VLLM_MODEL`, `CCAI_TELEGRAM_BOT_TOKEN`, `CCAI_TELEGRAM_ALLOWED_CHAT_ID`, `CCAI_OTA_MANIFEST_URL`을 지정할 수 있습니다.

도커 외부 웹 채팅, 텔레그램, H200 API 키 설정은 [Connectivity Guide](docs/connectivity.md)를 참고하세요.

## 관리자 명령 예시

웹 또는 텔레그램에서 JSON 또는 짧은 텍스트 명령을 보낼 수 있습니다.

```json
{"type":"patrol_start"}
{"type":"patrol_stop"}
{"type":"go_home"}
{"type":"inspect","target":"charging station"}
{"type":"say","text":"현재 상태 보고"}
```

짧은 텍스트도 지원합니다.

```text
patrol start
patrol stop
go home
inspect entrance
status
```

## OTA 업데이트 흐름

1. 서버에 OTA manifest JSON을 게시합니다.
2. Jetson의 `ota_agent_node`가 주기적으로 manifest를 확인합니다.
3. `auto_apply: false`이면 계획만 로그로 보고합니다.
4. `auto_apply: true`이면 manifest에 정의된 명령을 순서대로 실행합니다.

manifest 예시:

```json
{
  "version": "2026.07.21-1",
  "commands": [
    "git fetch origin",
    "git pull --ff-only",
    "colcon build --symlink-install",
    "sudo systemctl restart ccai-jetbot.service"
  ]
}
```

운영 환경에서는 OTA manifest 서명 검증, HTTPS, 명령 allowlist, 롤백 파티션 또는 컨테이너 이미지 롤백을 추가해야 합니다.

## 검증

```bash
python3 -m compileall ccai_jetbot_patrol
colcon test
```
