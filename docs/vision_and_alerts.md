# Vision Navigation and Patrol Alerts

`vision_nav_node`(자율 주행/따라가기)와 `vlm_client_node`(영상 분석/알림)의 동작 방식과 설정을 정리합니다.

## 1. 순찰 중 바퀴가 안 움직이던 문제 (수정됨)

`patrol_node`는 `use_vision_cmd_vel: true`일 때 `vision_nav_node`가 최근에 보낸 주행 명령(`/ccai/vision_cmd_vel`)을 우선 사용합니다. 기존 코드는 이 명령이 "최근에 없으면" 무조건 정지시켰는데, `vision_nav_node`가 비활성화되어 있거나 아직 한 번도 명령을 보낸 적이 없는 경우에도 계속 정지 상태로 남아 **바퀴가 전혀 움직이지 않는 문제**가 있었습니다.

이제는 `vision_nav_node`가 실제로 한 번이라도 명령을 보낸 뒤에 끊긴 경우에만(카메라/비전 유실에 대한 안전 정지) 정지시키고, 애초에 비전 명령을 받은 적이 없으면 `patrol_node`의 기본 전진/회전 패턴으로 주행합니다. [patrol_node.py](../ccai_jetbot_patrol/ccai_jetbot_patrol/patrol_node.py)의 `drive_loop`를 참고하세요.

## 2. YOLO 기반 자율 주행 / 따라가기

### 모델 준비

YOLO 모델(ONNX)이 없으면 `vision_nav_node`는 자동으로 예전 방식(엣지 밀도 기반 장애물 회피 + HOG 사람 검출)으로만 동작합니다. YOLO를 쓰려면 최초 1회 모델을 받습니다.

```bash
./scripts/download_yolo_model.sh
```

기본값은 Ultralytics의 `yolov8n.onnx`(COCO 80종 클래스, 약 12MB)를 `data/models/yolov8n.onnx`에 받습니다. 다른 모델/경로를 쓰려면:

```bash
CCAI_YOLO_MODEL_URL=https://... CCAI_YOLO_MODEL_PATH=data/models/custom.onnx ./scripts/download_yolo_model.sh
```

그리고 `robot.yaml`의 `vision_nav_node.yolo_model_path`를 맞춰줍니다. 모델을 받은 뒤에는 컨테이너를 재시작해야 로드됩니다.

```bash
docker restart ccai-jetbot
```

로드 여부는 `/ccai/events`에서 확인합니다.

```text
yolo model loaded: data/models/yolov8n.onnx
```

또는 모델이 없을 때:

```text
yolo model not found at data/models/yolov8n.onnx; using HOG person detector only (run scripts/download_yolo_model.sh to enable YOLO)
```

### 자율 순찰 (공간/장애물 감지)

`compute_patrol_command`는 기존 엣지 밀도 기반 주행(카메라 하단부의 Canny 엣지 밀도로 좌/중앙/우 클리어니스를 비교해 조향)을 기본 골격으로 유지하면서, YOLO가 있으면 프레임 하단-중앙의 "주행 경로" 영역(가로 가운데 1/3, 세로 하단 `obstacle_path_bottom_fraction` 비율)에 일정 크기(`obstacle_box_min_area`) 이상의 객체가 검출되면 그걸 장애물로 우선 처리해 회전시킵니다. 두 방식이 서로 보완하므로 YOLO 모델이 없어도 기존처럼 동작합니다.

관련 파라미터 (`robot.yaml` → `vision_nav_node`):

```yaml
yolo_model_path: "data/models/yolov8n.onnx"
yolo_input_size: 320
yolo_confidence: 0.45
yolo_nms_threshold: 0.45
yolo_detect_every_n_frames: 3
obstacle_box_min_area: 0.05
obstacle_path_bottom_fraction: 0.5
obstacle_trigger_min_interval_seconds: 4.0
```

### 지정 사람/물체 따라가기

`follow_person` 미션의 `target`이 비어있거나 "사람"/"나"/"me" 등이면 사람을 따라갑니다. 그 외 값은 COCO 80종 클래스 이름(영문) 또는 자주 쓰는 한국어 단어(가방, 배낭, 의자, 컵, 휴대폰, 책, 우산, 시계, 노트북, 자동차, 자전거, 강아지, 고양이, 박스 등, [vision_nav_node.py](../ccai_jetbot_patrol/ccai_jetbot_patrol/vision_nav_node.py)의 `OBJECT_ALIASES` 참고)과 매칭해서 해당 클래스를 따라갑니다. 매칭되는 게 없으면 사람으로 fallback합니다.

예:

```bash
curl -X POST http://127.0.0.1:8080/api/chat \
  -H "Content-Type: application/json" \
  -d '{"message":"저 가방을 따라가"}'
```

YOLO 모델이 없으면 target이 사람인 경우에만 기존 HOG 검출기로 계속 동작하고, 그 외 물체 지정은 무시되고 "찾는 중" 상태로 회전만 합니다.

## 3. 카메라 분석 알림 (VLM → 텔레그램/웹채팅)

`vlm_client_node`는 카메라 프레임을 H200 vLLM(Qwen3-VL)에 보내 분석합니다.

- 주기: 기본 5초에 한 번 (`min_interval_seconds: 5.0`).
- 이벤트 트리거: `vision_nav_node`가 주행 경로에서 장애물(엣지 밀도 초과 또는 YOLO 감지)을 만나면 `/ccai/vlm_trigger`로 즉시 분석을 요청합니다(같은 이벤트가 반복 트리거하지 않도록 `obstacle_trigger_min_interval_seconds`=4초 간격 제한).
- 응답 형식: 모델에게 `RISK:` 또는 `NORMAL:`로 시작하고 이어서 **한국어 50자 이내** 요약 한 줄만 답하도록 프롬프트로 강제합니다. `vlm_client_node`가 이 접두사를 파싱해서 `/ccai/vlm_observation`에 `{"risk": true/false, "summary": "...", "raw": "..."}` JSON으로 발행합니다.
- 알림 결정: `patrol_node`가 이 JSON을 구독해서 `risk: true`이고 로봇이 실제 임무 중(`patrolling`/`following_person`/`inspecting`)이면 `attention required: ...`를 `/ccai/events`에 발행합니다.
- 전달: `/ccai/events`는 `telegram_bridge_node`가 등록된 chat_id로, `web_chat_node`가 웹 채팅 로그로 각각 그대로 전달하므로 **텔레그램과 웹채팅에 동시에** 알림이 뜹니다. 별도 배선을 추가한 게 아니라 기존 이벤트 파이프라인을 그대로 씁니다.

관련 파라미터 (`robot.yaml` → `vlm_client_node`):

```yaml
trigger_topic: "/ccai/vlm_trigger"
summary_max_chars: 50
min_interval_seconds: 5.0
```

동작에는 `.env`의 `CCAI_VLLM_API_BASE_URL`/`CCAI_VLLM_API_KEY`/`CCAI_VLLM_MODEL`이 설정돼 있어야 하고(`docs/connectivity.md` 참고), `CCAI_ENABLE_VLM=1`로 컨테이너가 떠 있어야 합니다.

확인:

```bash
ros2 topic echo /ccai/vlm_observation
ros2 topic echo /ccai/events
```
