# Operations

## Jetson 배포

1. Ubuntu 22.04 또는 ROS2 Humble 컨테이너 환경을 준비합니다.
2. 저장소를 `/home/jetson/ccai_jetbot_ros`에 클론합니다.
3. `scripts/install_jetson_humble.sh`를 실행합니다.
4. `.env.example`을 참고해 `.env`를 만들거나 `ccai_jetbot_patrol/config/robot.yaml`의 H200 주소와 텔레그램 토큰을 설정합니다.
5. `colcon build --symlink-install` 후 `ros2 launch ccai_jetbot_patrol patrol.launch.py`를 실행합니다.

## systemd 등록

```bash
sudo cp systemd/ccai-jetbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ccai-jetbot.service
journalctl -u ccai-jetbot.service -f
```

## H200 vLLM

H200 서버에서는 CUDA, PyTorch, vLLM 설치 후 다음을 실행합니다.

```bash
PORT=8000 TENSOR_PARALLEL_SIZE=1 ./scripts/start_h200_vllm.sh Qwen/Qwen3-VL-32B-Instruct
```

상태 확인:

```bash
curl -H "Authorization: Bearer VLLM_API_KEY" http://H200_IP:8000/v1/models
```

H200 API 키, 도커 외부 웹 채팅, 텔레그램 연결 절차는 `docs/connectivity.md`에 상세히 정리되어 있습니다.

## 로그 확인

```bash
ros2 topic echo /ccai/status
ros2 topic echo /ccai/events
journalctl -u ccai-jetbot.service -f
```

## 수동 명령 주입

```bash
ros2 topic pub --once /ccai/mission_command std_msgs/msg/String "{data: 'patrol start'}"
ros2 topic pub --once /ccai/mission_command std_msgs/msg/String "{data: 'inspect entrance'}"
ros2 topic pub --once /ccai/mission_command std_msgs/msg/String "{data: 'patrol stop'}"
```
