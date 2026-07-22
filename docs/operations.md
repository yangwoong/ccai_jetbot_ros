# Operations

## Jetson 배포

이 프로젝트의 표준 배포 방식은 Docker입니다. 저장소를 `/home/roboat/work/ros2_ws/ccai_jetbot_ros`에 클론한 뒤 `.env.example`을 참고해 `.env`를 만들고, `docs/docker_host_operations.md`의 절차(최초 실행, 부팅 시 자동 실행, 코드 업데이트)를 따르세요. 하드웨어/카메라 설정은 `docs/hardware_jetbot.md`를 참고합니다.

Docker 없이 Jetson에 ROS2 Humble을 직접 설치해서 실행하려면 `scripts/install_jetson_humble.sh` → `colcon build --symlink-install` → `ros2 launch ccai_jetbot_patrol patrol.launch.py` 순서로 진행할 수 있지만, 권장 경로가 아니며 이 문서의 나머지 안내는 Docker 배포를 기준으로 합니다.

## systemd 등록 (부팅 시 자동 실행)

`systemd/ccai-jetbot.service`는 부팅 후 Docker가 뜨면 `host_docker_run.sh`를 자동 실행합니다. 자세한 설정은 `docs/docker_host_operations.md`의 "부팅 시 자동 실행" 절을 참고하세요.

```bash
sudo cp systemd/ccai-jetbot.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now ccai-jetbot.service
journalctl -u ccai-jetbot.service -f
```

## H200 vLLM

H200 서버에서는 CUDA, PyTorch, vLLM 설치 후 다음을 실행합니다.

```bash
PORT=8000 TENSOR_PARALLEL_SIZE=1 ./scripts/start_h200_vllm.sh Qwen/Qwen3-VL-70B-Instruct
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
