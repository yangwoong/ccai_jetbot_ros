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

### TensorRT 엔진 (기본 경로) / OpenCV DNN·HOG (폴백)

Jetson Nano(JetPack 4.6.x / L4T r32.7.1)에서 OpenCV DNN 모듈(`cv2.dnn`)로 YOLOv8 ONNX를 CUDA 백엔드로 돌리면 이 플랫폼의 OpenCV 4.5.0 빌드가 특정 연산(`scale_shift`)에서 매 프레임 예외를 던지고, CPU 백엔드로 돌려도 다른 연산(`shape_utils.hpp`의 `total()`)에서 또 실패하는 문제가 실측으로 확인됐습니다. 원인은 OpenCV의 ONNX 임포터 자체가 최신 YOLOv8 export의 연산 패턴을 완전히 지원하지 않는 것이었습니다.

그래서 `vision_nav_node`는 이제 **NVIDIA TensorRT 런타임으로 직접 추론하는 경로를 우선 사용**하고, 안 되면 기존 OpenCV DNN → HOG/엣지 밀도 순서로 내려갑니다 (3단계 폴백, 언제나 항상 켜져 있는 안전망 원칙 유지).

```
1순위: TensorRT 엔진 (ccai_jetbot_patrol/tensorrt_yolo.py) — data/models/yolov8n.engine
2순위: OpenCV DNN ONNX (CUDA 우선, 실패 시 CPU) — data/models/yolov8n.onnx
3순위: HOG 사람 검출 + 엣지 밀도 장애물 회피 (YOLO 전혀 없이도 동작하던 기존 방식)
```

trtexec(NVIDIA 자체 ONNX→TensorRT 변환기)는 OpenCV의 ONNX 임포터보다 훨씬 폭넓게 연산을 지원하므로, 이 경로를 쓰면 opset/버전 호환성 문제를 근본적으로 우회합니다.

#### TensorRT 엔진 빌드 (Jetson에서 1회 실행)

`.engine` 파일은 `.onnx`와 달리 **하드웨어/TensorRT 버전에 종속적이라 이식이 안 되고, git에도 커밋하지 않습니다.** 반드시 실제 Jetson에서 빌드해야 합니다.

```bash
cd /home/roboat/work/ros2_ws/ccai_jetbot_ros
./scripts/build_yolo_tensorrt_engine.sh
docker restart ccai-jetbot
```

`data/models/yolov8n.onnx`(이미 git에 커밋되어 있어 `git pull`로 받아짐)를 `trtexec --onnx=... --saveEngine=data/models/yolov8n.engine --fp16`로 변환합니다. `trtexec`는 L4T/JetPack TensorRT 설치에 포함되어 있고, 호스트에 없으면 컨테이너 안에서 자동으로 실행합니다. 성공하면 벤치마크 추론 결과가 출력되고 `data/models/yolov8n.engine`이 생성됩니다.

로드 여부는 이벤트로 확인합니다.

```text
yolo model loaded via TensorRT: data/models/yolov8n.engine
```

엔진 파일이 없으면 자동으로 다음 단계(OpenCV DNN ONNX)로 내려갑니다:

```text
yolo model loaded: data/models/yolov8n.onnx (cuda)
```

#### TensorRT 실행 실패 시 (자동 복구)

`ccai_jetbot_patrol/tensorrt_yolo.py`는 TensorRT 7.x(Jetson Nano/JetPack 4.6.x가 쓰는 바인딩 인덱스 기반 API)를 대상으로 NVIDIA 공식 샘플(`samples/python/common.py`)의 표준 버퍼 할당 패턴을 따릅니다. `tensorrt`/`pycuda` 파이썬 모듈은 dusty-nv L4T ROS 이미지에 이미 포함되어 있을 가능성이 높지만(별도 설치가 필요할 수도 있음), 없거나 엔진 로드/추론이 실패하면:

1. 엔진 로드 실패 시 즉시 OpenCV DNN ONNX 경로로 폴백합니다 (이벤트: `tensorrt engine load failed (...); falling back to OpenCV DNN ONNX`).
2. 추론 도중 실패하면 TensorRT를 그 세션에서 완전히 끄고 OpenCV DNN ONNX로 전환합니다(이벤트: `tensorrt inference failed (...); falling back to OpenCV DNN ONNX/HOG`). ONNX 모델도 아직 안 열려있으면 그 자리에서 엽니다.
3. OpenCV DNN 쪽도 CUDA→CPU→완전 비활성화의 기존 자동 복구가 그대로 적용됩니다 (아래 참고).

즉 TensorRT가 이 정확한 엔진/JetPack 조합에서 문제가 있어도 로봇은 항상 어떤 형태로든 계속 동작합니다(최소 HOG/엣지 밀도까지는 보장). `docker logs`에서 확인:

```bash
docker logs --since 10m ccai-jetbot | grep -Ei "tensorrt|yolo inference"
```

#### OpenCV DNN 폴백 자체가 실패하는 경우

TensorRT 엔진이 없어서(또는 실패해서) OpenCV DNN ONNX 경로로 왔는데 그것도 실패하는 경우의 기존 자동 복구입니다.

1. 첫 실패 시 한 번만 CPU 백엔드로 전환해서 같은 프레임을 재시도합니다 (이벤트: `yolo inference failed on current backend (...); retrying on CPU`).
2. CPU에서도 실패하면 YOLO를 완전히 끄고 엣지 밀도 기반 장애물 회피 + HOG 사람 검출로만 동작합니다 (이벤트: `... disabling YOLO, using HOG/edge-density only`).

`scripts/download_yolo_model.sh`는 opset 11 + `simplify=True`로 내보내도록 되어 있습니다(`CCAI_YOLO_ONNX_OPSET`, `CCAI_YOLO_ONNX_SIMPLIFY`로 조정 가능) — OpenCV DNN + YOLOv8 export 조합에서 흔히 알려진 완화책이지만, TensorRT 엔진 경로가 정상 동작하면 이 폴백 단계까지 갈 일은 거의 없습니다.

관련 파라미터 (`robot.yaml` → `vision_nav_node`):

```yaml
yolo_model_path: "data/models/yolov8n.onnx"
yolo_engine_path: "data/models/yolov8n.engine"
```

### 자율 순찰 (공간/장애물 감지)

실제 주행 중 전방 장애물을 인식 못 하고 충돌하는 사례가 있었습니다. 원인은 그 시점에 YOLO가 완전히 비활성화된 상태(TensorRT/OpenCV DNN 둘 다 실패해서 자동 폴백)였고, 남은 엣지 밀도(Canny) 방식만으로는 **표면이 매끈하고 색이 균일한 장애물**(벽, 상자, 다리 등 텍스처가 거의 없는 물체)을 잡아내지 못하기 때문이었습니다 — Canny 엣지는 "질감/윤곽선"을 보는 방식이라, 장애물이 근접해서 화면 대부분을 차지하며 흐릿하고 균일해 보일수록 오히려 엣지가 줄어들어 "빈 바닥"으로 오인하기 쉽습니다.

그래서 장애물 감지를 **4가지 독립적인 신호의 OR 조합**으로 강화했습니다 — 하나라도 장애물이라고 판단하면 회피합니다. YOLO가 꺼져 있어도(엔진/모델이 없거나 실패해도) 나머지 3가지는 계속 동작합니다.

1. **YOLO 바운딩박스** (`detect_path_obstacle`): 주행 경로 영역(가로 가운데 1/3, 세로 하단 `obstacle_path_bottom_fraction` 비율)에 `obstacle_box_min_area` 이상의 객체가 검출되면 장애물로 처리합니다.
2. **엣지 밀도** (기존): 카메라 하단부의 Canny 엣지 밀도가 `obstacle_stop_edge_density`를 넘으면 장애물로 처리합니다. 질감이 있는 장애물(가구 모서리, 케이블, 패턴 등)에 잘 반응합니다.
3. **바닥색 대비** (`detect_floor_color_obstacle`, 신규): 바퀴 바로 앞의 "기준 바닥" 색(가장 가까운 하단 스트립)과 조금 더 앞쪽 "주행 경로" 영역의 색을 비교해서, 색 차이(BGR 유클리드 거리)가 `floor_color_diff_threshold`를 넘으면 장애물로 처리합니다. 질감이 없어도 색이 바닥과 다르면 잡아냅니다.
4. **바닥 급변 감지** (`detect_sudden_bottom_change`, 신규): 바퀴 바로 앞 스트립의 색이 이전 프레임 대비 갑자기 크게 바뀌면(`bottom_change_threshold`) 장애물로 처리합니다. 장애물이 이미 "기준 바닥" 영역까지 침범해서 3번 비교가 무력화되는 근접 상황을 잡기 위한 마지막 안전망입니다.

```yaml
yolo_model_path: "data/models/yolov8n.onnx"
yolo_engine_path: "data/models/yolov8n.engine"
yolo_input_size: 320
yolo_confidence: 0.45
yolo_nms_threshold: 0.45
yolo_detect_every_n_frames: 3
obstacle_box_min_area: 0.05
obstacle_path_bottom_fraction: 0.5
obstacle_trigger_min_interval_seconds: 4.0
obstacle_stop_edge_density: 0.16
floor_color_diff_threshold: 40.0
bottom_change_threshold: 35.0
```

**튜닝 참고**: 이 임계값들은 실제 바닥/조명 조건에서 검증된 게 아니라 합리적인 기본값입니다. 순찰 상태 이벤트(`path left=..., center=..., right=..., steer=..., ramp=...`)와 장애물 이벤트(`obstacle center=..., color=True/False, sudden=True/False`)를 보면서 다음처럼 조정하세요.

- **장애물을 놓치고 부딪힘(감지 안 됨)**: `floor_color_diff_threshold`/`bottom_change_threshold`/`obstacle_stop_edge_density`를 낮춰서 더 민감하게 만드세요.
- **장애물이 없는데 자꾸 회피함(오탐)**: 반대로 각 임계값을 올리세요. 특히 바닥이 반질반질하거나(반사) 그림자/조명 얼룩이 있으면 `floor_color_diff_threshold`가 너무 낮으면 오탐이 잦을 수 있습니다.
- 카메라가 아래를 보는 각도, 장착 높이에 따라 "기준 바닥"/"주행 경로" 영역(코드의 `0.60`/`0.85`/`0.90` 비율)이 실제 바닥/장애물과 안 맞을 수 있습니다. 심하게 안 맞으면 `vision_nav_node.py`의 `detect_floor_color_obstacle`/`detect_sudden_bottom_change`의 비율 상수를 조정하세요.

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

### VLM이 응답을 안 하는 경우 (진단 가능하게 수정됨)

기존에는 H200 연결 실패/타임아웃/모델 오류 등이 나면 ROS 로거에만 조용히 남고 텔레그램/웹채팅에는 아무 흔적도 없었습니다(관리자 입장에서는 그냥 "분석이 안 되는" 것으로만 보임). 이제 요청이 실패하면 `error_event_min_interval_seconds`(기본 30초) 간격으로 `/ccai/events`에도 발행되어 웹채팅/텔레그램에 그대로 보입니다.

```text
vlm request failed: HTTPConnectionPool(host='H200_IP', port=8000): ...
```

확인 순서:

```bash
# 1) vlm_client_node 자체가 켜져 있는지
docker exec ccai-jetbot printenv | grep CCAI_ENABLE_VLM   # 1이어야 함

# 2) H200 연결 상태 (llm_control_node와 같은 엔드포인트를 씀)
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"llm status"}'

# 3) 최근 vlm 관련 로그/이벤트
docker logs --since 5m ccai-jetbot | grep -i vlm
```

확인:

```bash
ros2 topic echo /ccai/vlm_observation
ros2 topic echo /ccai/events
```

## 5. 자연어 이동/속도/분석 명령

웹채팅/텔레그램에서 순찰 시작 없이도 자연어로 개별 동작을 시킬 수 있습니다. `mission.py`의 직접 명령 매칭(빠른 경로)과 LLM 라우팅(그 외 표현) 둘 다로 처리됩니다.

| 동작 | 예시 문장 | 내부 명령 타입 |
|---|---|---|
| 전진 (정지할 때까지 계속) | "앞으로 가", "전진해", "직진" | `move_forward` |
| 후진 (정지할 때까지 계속) | "뒤로 가", "후진해" | `move_backward` |
| 저속 전진/후진 | "천천히 앞으로 가", "천천히 뒤로 가" | `move_forward`/`move_backward` (target="slow") |
| 좌/우회전 (짧게) | "좌회전해", "우회전해" | `turn_left`/`turn_right` |
| 정지 | "정지", "멈춰" | `patrol_stop` |
| 속도 높이기/낮추기 | "속도 높여"/"속도 줄여" | `set_speed` (up/down) |
| 즉시 영상 분석 | "영상 분석해", "지금 뭐가 보여" | `analyze` |

**전진/후진은 명시적으로 정지시키기 전까지 계속 이동합니다** (예전에는 1.5초만 움직이고 멈췄는데, "천천히 앞으로 가"가 조금 가다 멈추는 게 의도와 다르다는 피드백으로 수정했습니다 — 의도는 "느리게 계속 가다가 장애물이 있으면 서거나 피하고, 정지 명령이 올 때까지 계속 가는 것"이었습니다). `vision_nav_node`가 켜져 있으면 전진 중 장애물 회피가 그대로 적용됩니다(후진은 카메라가 뒤를 못 보므로 회피 없이 계속 이동 — 정지 명령으로만 멈춥니다). 좌/우회전은 여전히 `manual_turn_seconds`(기본 0.8초) 동안만 도는 짧은 넛지입니다(방향만 살짝 트는 용도).

- 속도 조절은 `patrol_node`의 `linear_speed`/`angular_speed`에 곱해지는 `speed_scale` 배율을 `speed_step`(기본 0.2)만큼 올리고 내립니다 (`min_speed_scale`~`max_speed_scale`, 기본 0.3~2.0 사이로 clamp). "천천히 앞으로 가"는 이거와 별개로 그 이동 한 번에만 `manual_drive_slow_factor`(기본 0.5배)를 추가로 곱합니다.
- "영상 분석해"는 `/ccai/vlm_trigger`로 즉시 분석을 요청하고, 결과가 오면 위험 여부와 상관없이 `analysis result: ...`로 `/ccai/events`에 발행되어 웹채팅/텔레그램에 그대로 보입니다.

관련 파라미터 (`robot.yaml` → `patrol_node`):

```yaml
manual_move_seconds: 1.5   # 위치를 가르치는 중(녹화 중)에만 전/후진에 쓰이는 넛지 길이
manual_turn_seconds: 0.8
manual_drive_slow_factor: 0.5
speed_step: 0.2
min_speed_scale: 0.3
max_speed_scale: 2.0
speed_ramp_seconds: 1.5
speed_ramp_min_factor: 0.35
```

### 모터 방향이 반대였던 문제 (수정됨)

"전진"을 명령하면 실제로는 후진하는 문제가 있었습니다. `jetbot_hardware_node`의 `left_trim`/`right_trim`이 `1.0`으로 되어 있었는데, 이 로봇의 실제 모터 배선/극성 기준으로는 `-1.0`이어야 정방향이 맞았습니다. `robot.yaml`에서 둘 다 `-1.0`으로 바꿨습니다. 좌/우가 바뀌어 보이면(전후는 맞는데 회전 방향만 반대) 둘 중 하나만 부호를 바꾸면 됩니다.

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

## 8. 위치 가르치기와 지역 기반 임무 ("정문앞에 택배가 있는지 보고와")

오도메트리/IMU가 없어서 좌표 기반 지도는 못 만들지만, "이 이름으로 가려면 이렇게 움직이면 된다"는 teach-and-repeat 방식으로 이름 붙은 위치를 다룰 수 있습니다. 자세한 배경은 [Navigation Roadmap](navigation_roadmap.md)을 참고하세요.

### 위치 가르치기

```bash
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"기억 시작"}'
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"앞으로 가"}'
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"정지"}'
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"좌회전해"}'
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"정문으로 저장해"}'
```

`기억 시작` 이후 나온 이동 명령들(전진/후진/좌회전/우회전)이 시간과 함께 기록되고, `<이름>으로 저장해`로 그 시퀀스를 이름과 함께 저장합니다(`data/locations.json`). 이 파일은 컨테이너 재시작에도 유지됩니다(호스트 저장소에 있는 `data/`가 컨테이너에 바인드 마운트되므로).

### 지역 기반 임무 수행

```bash
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"정문앞에 택배가 있는지 보고와"}'
```

이 문장은 직접 명령 패턴에 안 걸리므로 LLM(Qwen3-VL-70B)이 처리합니다. LLM이 `{"type":"inspect","target":"정문","text":"택배가 있는지 확인해줘"}`로 해석하면:

1. "정문"이 저장돼 있으면 그 위치까지 저장된 이동 시퀀스를 재생합니다(이벤트: `heading to 정문 (N steps)`).
2. 도착하면 `vlm_client_node`에게 그 질문("택배가 있는지 확인해줘")을 그대로 물어보고, 결과를 `analysis result: 정문: ...`으로 `/ccai/events`에 발행합니다 — 텔레그램/웹채팅에 그대로 뜹니다.
3. "정문"을 모르면(아직 안 가르쳤으면) 현재 위치에서 바로 확인하고 "위치를 모른다"는 안내를 함께 보냅니다.

### 확인

```bash
docker exec ccai-jetbot cat /home/workspace/ccai_jetbot_ros/data/locations.json
```

관련 파라미터 (`robot.yaml` → `patrol_node`): `locations_file` (기본 `data/locations.json`).
