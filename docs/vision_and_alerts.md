# Vision Navigation and Patrol Alerts

`vision_nav_node`(자율 주행/따라가기)와 `vlm_client_node`(영상 분석/알림)의 동작 방식과 설정을 정리합니다.

## 1. 순찰 중 바퀴가 안 움직이던 문제 (수정됨)

`patrol_node`는 `use_vision_cmd_vel: true`일 때 `vision_nav_node`가 최근에 보낸 주행 명령(`/ccai/vision_cmd_vel`)을 우선 사용합니다. 기존 코드는 이 명령이 "최근에 없으면" 무조건 정지시켰는데, `vision_nav_node`가 비활성화되어 있거나 아직 한 번도 명령을 보낸 적이 없는 경우에도 계속 정지 상태로 남아 **바퀴가 전혀 움직이지 않는 문제**가 있었습니다.

이제는 `vision_nav_node`가 실제로 한 번이라도 명령을 보낸 뒤에 끊긴 경우에만(카메라/비전 유실에 대한 안전 정지) 정지시키고, 애초에 비전 명령을 받은 적이 없으면 `patrol_node`의 기본 전진/회전 패턴으로 주행합니다. [patrol_node.py](../ccai_jetbot_patrol/ccai_jetbot_patrol/patrol_node.py)의 `drive_loop`를 참고하세요.

## 2. 카메라 영상이 몇 초 지연되던 문제 (수정됨)

카메라 앞을 손으로 가려도 웹채팅 프리뷰에 몇 초 뒤에나 반영되는 문제가 있었습니다. 원인은 `camera_node.py`의 CSI GStreamer 파이프라인이었습니다.

`csi_jetbot`/`csi_jetcam` 백엔드(실제로 열리는 백엔드, 로그의 `backend=csi_jetbot`)는 파이프라인 끝에 옵션 없는 맨 `appsink`를 썼습니다. GStreamer의 `appsink`는 기본적으로 들어오는 프레임을 큐에 쌓는데, CSI 센서는 `csi_fps`(기본 30fps)로 계속 프레임을 만드는 반면 `camera_node`는 `fps`(기존 기본 5fps) 주기로만 큐에서 하나씩 꺼내 갔습니다. 즉 꺼내가는 속도보다 쌓이는 속도가 훨씬 빨라서 큐에 오래된 프레임이 계속 쌓였고, `.read()`는 그 오래된 프레임부터 순서대로 반환했습니다 — 시간이 지날수록 지연이 계속 늘어나는 구조적 버그였고, 그게 "손을 가려도 몇 초 뒤에 나타나는" 증상이었습니다.

`appsink drop=true max-buffers=1 sync=false` 옵션을 모든 CSI 백엔드에 통일해서, 큐에 최신 프레임 1개만 남기고 오래된 프레임은 버리도록 고쳤습니다. 이제 지연은 프레임 주기 수준(수십~백여 ms)으로 유지됩니다. 호스트 MJPEG 브리지(`scripts/host_csi_mjpeg_server.py`)의 파이프라인도 같은 문제가 있어서 함께 고쳤습니다.

추가로 반응 속도를 높이기 위해 `camera_node`의 기본 `fps`를 5 → 10으로, 웹채팅 프리뷰의 폴링 주기를 500ms → 150ms로 올렸습니다. 순찰/장애물 회피는 새 프레임이 도착할 때마다 다시 계산되므로, 프레임이 자주 올수록 그만큼 빨리 반응합니다.

```yaml
camera_node:
  ros__parameters:
    fps: 10.0   # 기존 5.0
```

Jetson Nano급 하드웨어에서 320x240 JPEG 인코딩은 10fps에서도 가벼우므로 CPU 부하 걱정은 크지 않지만, 다른 부하(YOLO CPU 폴백 등)가 겹치면 `fps`를 다시 낮추는 것도 고려하세요.

## 3. YOLO 기반 자율 주행 / 따라가기

### 모델 준비

YOLO 모델(ONNX)이 없으면 `vision_nav_node`는 자동으로 예전 방식(엣지 밀도 기반 장애물 회피 + HOG 사람 검출)으로만 동작합니다. Ultralytics는 `.onnx`를 직접 배포하지 않고 `.pt` 가중치만 배포하므로, `scripts/download_yolo_model.sh`는 `ultralytics` 파이썬 패키지로 `yolov8n.pt`를 받아 그 자리에서 ONNX로 변환합니다. 이 패키지는 Python 3.8 이상이 필요해서 Jetson 호스트의 Python 3.6에서는 바로 실행되지 않습니다.

```bash
# Mac/PC/H200처럼 Python 3.8+가 있는 곳에서 실행
./scripts/download_yolo_model.sh

# 결과 파일을 Jetson 저장소로 복사
scp data/models/yolov8n.onnx roboat@JETSON_IP:/home/roboat/work/ros2_ws/ccai_jetbot_ros/data/models/yolov8n.onnx
```

Jetson에서 그냥 실행하면 Python 버전을 감지해서 위 안내를 그대로 출력하고 종료합니다. 이미 어딘가에 호스팅해 둔 `.onnx`가 있다면 다음처럼 바로 받을 수도 있습니다.

```bash
CCAI_YOLO_MODEL_URL=https://your-mirror/yolov8n.onnx ./scripts/download_yolo_model.sh
```

기본 파일명은 `data/models/yolov8n.onnx`(COCO 80종 클래스, 약 12MB)입니다. 다른 모델을 쓰려면 `CCAI_YOLO_MODEL_NAME`(예: `yolov8s`), `CCAI_YOLO_IMG_SIZE`를 지정하고 `robot.yaml`의 `vision_nav_node.yolo_model_path`를 맞춰줍니다. 모델을 받은 뒤에는 컨테이너를 재시작해야 로드됩니다.

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

`data/models/yolov8n.onnx`는 git에 커밋되어 있으므로(`.gitignore`에서 `data/models/*.onnx`만 예외 처리) `git pull`/`host_docker_update.sh`로 저장소를 받으면 자동으로 같이 동기화됩니다. 별도로 매번 다시 받을 필요가 없습니다.

### GPU 가속 / TensorRT 검증

`vision_nav_node`는 OpenCV가 CUDA로 빌드되어 있으면(`cv2.cuda.getCudaEnabledDeviceCount() > 0`) YOLO 추론에 `DNN_BACKEND_CUDA` + `DNN_TARGET_CUDA_FP16`을 자동으로 사용하고, 아니면 CPU로 폴백합니다. 로드 로그에서 확인합니다.

```text
yolo model loaded: data/models/yolov8n.onnx (cuda)
```

이건 OpenCV DNN 모듈의 CUDA 가속이고, NVIDIA TensorRT 런타임 자체를 쓰는 건 아닙니다. 이 ONNX 모델이 Jetson의 TensorRT로 실제 변환/구동까지 되는지 별도로 검증하려면 (컨테이너 안에서, `trtexec`는 L4T/TensorRT 설치에 포함되어 있습니다):

```bash
./scripts/verify_yolo_tensorrt.sh
```

이 스크립트는 `trtexec --onnx=data/models/yolov8n.onnx --saveEngine=data/models/yolov8n.engine --fp16`로 엔진을 빌드하고 벤치마크 추론까지 실행합니다. 출력에 `FAILED`가 없고 엔진 파일이 생성되면 이 모델이 이 Jetson에서 TensorRT로 정상 구동된다는 뜻입니다. 생성된 `.engine` 파일은 현재 `vision_nav_node`가 직접 로드하는 대상은 아니며(런타임은 OpenCV DNN을 씀), TensorRT 호환성 확인 및 향후 별도 TensorRT 추론 경로를 붙일 때를 위한 산출물입니다.

#### CUDA 추론이 이 모델/OpenCV 조합에서 깨진 경우 (자동 복구)

일부 L4T OpenCV 빌드(실측: OpenCV 4.5.0)는 이 YOLOv8 ONNX export의 특정 연산(`scale_shift`)을 CUDA DNN 백엔드에서 처리하지 못하고 매 프레임 예외를 던집니다. 모델 로드 자체는 성공하고 `(cuda)`로 표시되기 때문에 실행해보기 전까지는 알 수 없습니다. `vision_nav_node`는 이 실패를 감지하면:

1. 첫 실패 시 한 번만 CPU 백엔드로 전환해서 같은 프레임을 재시도합니다 (이벤트: `yolo inference failed on current backend (...); retrying on CPU`).
2. CPU에서도 실패하면 YOLO를 완전히 끄고 엣지 밀도 기반 장애물 회피 + HOG 사람 검출로만 동작합니다 (이벤트: `... disabling YOLO, using HOG/edge-density only`).

즉 매 프레임 에러 로그가 무한히 쌓이지 않고, 최소 한 번의 이벤트로 무슨 일이 있었는지 알 수 있습니다. `docker logs`에서 확인:

```bash
docker logs --since 10m ccai-jetbot | grep -i "yolo inference"
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

#### 항상 느린 속도에서 점차 빨라지는 이동 (속도 램프업)

순찰 시작 직후 곧바로 최고 속도로 튀어나가면 바로 앞 장애물에 부딪힐 수 있어서, 모든 전진 이동은 항상 느린 속도에서 시작해 `speed_ramp_seconds` 동안 목표 속도까지 서서히 올라갑니다(`speed_ramp_min_factor`가 시작 속도 비율, 기본 35%). 이 "전진 구간 시작 시각"은 장애물 회피 회전이 일어날 때마다 리셋되므로, **회전 후 다시 전진할 때도 항상 느린 속도부터 다시 시작**합니다 — 방금 피한 장애물이 아직 근처에 있을 수 있기 때문입니다. `patrol_node`의 기본 전진/회전 패턴(비전 없이 순찰할 때)과 자연어 "앞으로 가"/"뒤로 가" 명령에도 동일한 램프업이 적용됩니다(`patrol_node`의 `speed_ramp_seconds`/`speed_ramp_min_factor`).

```yaml
# vision_nav_node, patrol_node 둘 다 동일한 이름의 파라미터
speed_ramp_seconds: 1.5
speed_ramp_min_factor: 0.35
```

#### 카메라 문제 시 즉시 정지 + 알림

카메라 프레임이 무효(초록 화면, 저대비 등)이거나 일정 시간(`min_valid_frame_seconds`) 프레임이 안 들어오면 `vision_nav_node`는 즉시 정지 명령을 보냅니다(기존 동작). 여기에 더해, 순찰/따라가기 중이면 `camera_alert_min_interval_seconds`(기본 10초) 간격으로 `/ccai/events`에도 알려서 **텔레그램/웹채팅으로 바로 통보**되도록 했습니다 — 이전에는 정지는 됐지만 알림이 없어서 로봇이 왜 멈췄는지 admin이 알 방법이 없었습니다.

```text
camera view is invalid, stopping motion
camera frames stopped arriving, stopping motion
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

## 4. 카메라 분석 알림 (VLM → 텔레그램/웹채팅)

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

## 5. 자연어 이동/속도/분석 명령

웹채팅/텔레그램에서 순찰 시작 없이도 자연어로 개별 동작을 시킬 수 있습니다. `mission.py`의 직접 명령 매칭(빠른 경로)과 LLM 라우팅(그 외 표현) 둘 다로 처리됩니다.

| 동작 | 예시 문장 | 내부 명령 타입 |
|---|---|---|
| 전진 | "앞으로 가", "전진해", "직진" | `move_forward` |
| 후진 | "뒤로 가", "후진해" | `move_backward` |
| 좌회전 | "좌회전해", "왼쪽으로 돌아" | `turn_left` |
| 우회전 | "우회전해", "오른쪽으로 돌아" | `turn_right` |
| 속도 높이기 | "속도 높여", "빠르게" | `set_speed` (up) |
| 속도 낮추기 | "속도 줄여", "천천히" | `set_speed` (down) |
| 즉시 영상 분석 | "영상 분석해", "지금 뭐가 보여" | `analyze` |

- 전진/후진은 `manual_move_seconds`(기본 1.5초), 좌/우회전은 `manual_turn_seconds`(기본 0.8초) 동안만 움직이고 자동으로 멈춥니다(관리자가 텍스트로 조작하는 것이라 계속 움직이면 위험하기 때문에 한 번에 짧게 넛지하는 방식입니다). 필요하면 명령을 여러 번 보내면 됩니다.
- 속도 조절은 `patrol_node`의 `linear_speed`/`angular_speed`에 곱해지는 `speed_scale` 배율을 `speed_step`(기본 0.2)만큼 올리고 내립니다 (`min_speed_scale`~`max_speed_scale`, 기본 0.3~2.0 사이로 clamp).
- "영상 분석해"는 `/ccai/vlm_trigger`로 즉시 분석을 요청하고, 결과가 오면 위험 여부와 상관없이 `analysis result: ...`로 `/ccai/events`에 발행되어 웹채팅/텔레그램에 그대로 보입니다.

관련 파라미터 (`robot.yaml` → `patrol_node`):

```yaml
manual_move_seconds: 1.5
manual_turn_seconds: 0.8
speed_step: 0.2
min_speed_scale: 0.3
max_speed_scale: 2.0
```

## 6. 텔레그램 알림 동작 확인

`telegram_bridge_node`는 `/ccai/events`로 들어오는 모든 이벤트(카메라/모터/순찰/VLM 등)를 등록된 `allowed_chat_id`로 그대로 전달합니다. 이게 실제로 동작하는지 순찰 이벤트가 나기 전에 바로 확인할 수 있도록, **컨테이너(텔레그램 브리지 노드)가 시작될 때마다** 아래 메시지를 자동으로 한 번 보냅니다.

```text
robot system online (container started, 2026-07-22 23:40:00)
```

이 메시지가 안 오면 다음을 확인하세요.

```bash
docker exec ccai-jetbot printenv | grep CCAI_ENABLE_TELEGRAM   # 1이어야 함
docker exec ccai-jetbot printenv | grep CCAI_TELEGRAM          # 토큰/chat_id가 비어있지 않아야 함
docker logs --since 5m ccai-jetbot | grep -i telegram
```

`CCAI_ENABLE_TELEGRAM`은 이전에는 `CCAI_SAFE_START=1`일 때 기본값이 꺼짐이었는데(문서에는 "안전 모드도 텔레그램은 켜짐"이라고 되어 있어서 실제 동작과 문서가 어긋나 있었습니다), 텔레그램은 하드웨어/카메라와 무관하므로 이제는 안전 모드 여부와 상관없이 기본값이 켜짐(`1`)입니다. 끄고 싶으면 `CCAI_ENABLE_TELEGRAM=0`을 명시하세요. `notify_startup: false`로 이 시작 알림만 끌 수도 있습니다.

시작 알림이든 순찰 이벤트든 텔레그램 전송이 실패하면(토큰/`chat_id` 미설정, 잘못된 토큰, 봇에게 먼저 말을 건 적이 없어서 등) 이전에는 아무 흔적 없이 조용히 사라졌습니다. 이제 `telegram_bridge_node`가 실패 사유를 로그로 남깁니다.

```bash
docker logs --since 5m ccai-jetbot | grep -i telegram
```

`telegram send skipped: bot_token not set` / `telegram startup notice skipped: ...` / `telegram send failed: HTTP 4xx ...` 중 하나가 보이면 그게 원인입니다. 특히 HTTP 401/403이면 봇 토큰이 틀렸거나, 관리자가 텔레그램에서 그 봇에게 먼저 아무 메시지나(`/start` 등) 보낸 적이 없는 경우가 많습니다(`docs/connectivity.md`의 chat_id 확인 절차 참고).

## 7. CSI 카메라 호스트 설정 자동 복구

`nvargus-daemon`의 `enableCamInfiniteTimeout=1` 문제(자세한 내용은 `docs/hardware_jetbot.md`)는 호스트 systemd 설정이라 git으로 관리되지 않고, 재플래시/패키지 업데이트/알 수 없는 이유로 다시 나타날 수 있습니다. 이제 이 수정을 스크립트로 자동화했습니다.

```bash
./scripts/host_fix_nvargus_daemon.sh
```

이 스크립트는:

- `/etc/systemd/system/nvargus-daemon.service`에 문제의 줄이 있는지 확인하고, 있으면 백업 후 제거 + `daemon-reload` + `nvargus-daemon` 재시작을 합니다.
- 이미 깨끗하면 아무것도 하지 않고 조용히 끝납니다(반복 실행해도 안전).
- 다음 두 곳에서 **자동으로** 호출되므로 보통 따로 실행할 필요는 없습니다.
  - `host_docker_run.sh`가 `CCAI_CAMERA_MODE=csi`로 컨테이너를 띄울 때마다 (수동 재시작 포함)
  - `systemd/ccai-jetbot.service`가 부팅 시 컨테이너를 띄우기 직전 (`ExecStartPre=+...`로 `User=roboat`인 서비스에서도 root 권한으로 실행되도록 했습니다)

이 스크립트는 호스트에서 직접 실행되므로 (`host_docker_run.sh`가 호출할 때도) 출력은 `docker logs`가 아니라 그 스크립트를 실행한 터미널, 또는 부팅 시 자동 실행이라면 systemd 저널에 남습니다.

```bash
# host_docker_run.sh를 수동으로 실행했을 때: 그 터미널 출력에 [nvargus-fix] 줄이 보입니다
# 부팅 자동 실행(systemd)일 때:
journalctl -u ccai-jetbot.service --since "10 minutes ago" | grep -i nvargus-fix
```
