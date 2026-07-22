# Architecture

```mermaid
flowchart LR
  Admin[Admin] --> Web[Web chat]
  Admin --> Telegram[Telegram bot]
  Web --> ROS[ROS2 mission_command]
  Telegram --> ROS
  ROS --> Patrol[patrol_node]
  Camera[JetBot camera] --> Vision[vision_nav_node YOLO+HOG]
  Camera --> VLM[vlm_client_node]
  Vision --> Patrol
  Vision -- obstacle trigger --> VLM
  VLM --> H200[H200 vLLM Qwen3-VL-70B]
  H200 --> VLM
  VLM --> Patrol
  Patrol --> Base[/cmd_vel JetBot base]
  Patrol -- risk alert --> Events[events/status/logs]
  Events --> Telegram
  Events --> Web
  OTA[ota_agent_node] --> Repo[Git/manifest server]
```

## Runtime Topics

- `/ccai/mission_command` (`std_msgs/String`): 관리자 명령
- `/ccai/status` (`std_msgs/String`): JSON 상태 보고
- `/ccai/events` (`std_msgs/String`): 이벤트/알림 (텔레그램/웹채팅으로 전달됨)
- `/ccai/vision_cmd_vel` (`geometry_msgs/Twist`): `vision_nav_node`의 자율 주행/따라가기 명령
- `/ccai/vlm_trigger` (`std_msgs/String`): 장애물 등 이벤트 발생 시 VLM 즉시 분석 요청
- `/ccai/vlm_observation` (`std_msgs/String`): VLM 이미지 분석 결과, `{"risk": bool, "summary": "...", "raw": "..."}`
- `/cmd_vel` (`geometry_msgs/Twist`): JetBot 주행 명령
- `/image_raw/compressed` (`sensor_msgs/CompressedImage`): 카메라 입력

자율 주행(YOLO 기반 장애물/경로 판단)과 따라가기, VLM 위험 알림의 세부 동작은 [Vision and Alerts](vision_and_alerts.md)를 참고하세요.

## Roadmap

핵심 목표(장애물 회피 → 지역 탐색/LLM 라벨링 → 내비게이션 지도 → 임무 할당 → 이상 감지)와 단계별 구현 상태, 다음 작업 순서는 [Navigation Roadmap](navigation_roadmap.md)에 정리되어 있습니다. 그 외 항목:

- 안전: 범퍼, 초음파, depth camera, e-stop 토픽을 추가해 `patrol_node`가 즉시 정지하도록 확장합니다.
- OTA: 운영 전 manifest 서명 검증과 명령 allowlist를 추가해야 합니다.

