# Architecture

```mermaid
flowchart LR
  Admin[Admin] --> Web[Web chat]
  Admin --> Telegram[Telegram bot]
  Web --> ROS[ROS2 mission_command]
  Telegram --> ROS
  ROS --> Patrol[patrol_node]
  Camera[JetBot camera] --> VLM[vlm_client_node]
  VLM --> H200[H200 vLLM Qwen3-VL-32B]
  H200 --> VLM
  VLM --> Patrol
  Patrol --> Base[/cmd_vel JetBot base]
  Patrol --> Events[events/status/logs]
  OTA[ota_agent_node] --> Repo[Git/manifest server]
```

## Runtime Topics

- `/ccai/mission_command` (`std_msgs/String`): 관리자 명령
- `/ccai/status` (`std_msgs/String`): JSON 상태 보고
- `/ccai/events` (`std_msgs/String`): 이벤트/알림
- `/ccai/vlm_observation` (`std_msgs/String`): VLM 이미지 분석 결과
- `/cmd_vel` (`geometry_msgs/Twist`): JetBot 주행 명령
- `/image_raw/compressed` (`sensor_msgs/CompressedImage`): 카메라 입력

## Roadmap

- Nav2 연동: 현재 `patrol_node`는 기본 속도 제어 상태 머신입니다. 실제 맵 기반 순찰은 Nav2 `NavigateToPose` 액션 클라이언트로 확장합니다.
- JetBot 모터 드라이버: Waveshare JetBot ROS2 드라이버 또는 `/cmd_vel` 호환 브리지 연결이 필요합니다.
- 안전: 범퍼, 초음파, depth camera, e-stop 토픽을 추가해 `patrol_node`가 즉시 정지하도록 확장합니다.
- OTA: 운영 전 manifest 서명 검증과 명령 allowlist를 추가해야 합니다.

