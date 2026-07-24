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

#### 장애물 감지 디버그 화면 (실제 충돌 후 추가됨)

실제 주행 중 장애물에 충돌했는데 관련 이벤트 로그가 전혀 안 남은 사례가 있었습니다 — 그 순간 카메라 프레임 자체가 끊겨서(`camera frames stopped arriving`) 장애물 판단 로직까지 도달하지 못했을 수도 있고, 애초에 감지가 안 됐을 수도 있어서 로그만으로는 구분이 안 됐습니다. 이제 매 프레임 감지 결과를 카메라 영상 위에 직접 그려서 웹채팅에서 실시간으로 볼 수 있습니다.

웹채팅(`http://JETSON_IP:8080`)에 카메라 프리뷰 옆에 "장애물 감지 디버그" 화면이 추가로 뜹니다.

- **노란 사각형**: 엣지 밀도 장애물 판단 영역
- **하늘색 사각형**: "주행 경로" 색 비교 영역
- **파란 사각형**: "바퀴 바로 앞" 기준 바닥 영역
- **초록 사각형**: YOLO 검출 박스(클래스명+신뢰도)
- 좌상단: `OBSTACLE`(빨강) / `CLEAR`(녹색)
- 좌하단: `edge=... color=... sudden=...` 실측값 (해당 프레임의 원본 수치)

또한 상태 표시줄 아래에 현재 `vision_status`의 전체 detail 문자열이 그대로 표시됩니다(이전에는 `state` 단어만 보이고 실제 수치는 API 응답에는 있어도 화면에 안 보였습니다). 물체를 카메라 앞에 놓고 값이 어떻게 변하는지 직접 보면서 임계값을 조정할 수 있습니다.

`/ccai/vision_debug_image` 토픽으로 발행되고 `/api/vision_debug.jpg`로 제공됩니다. 끄려면:

```yaml
vision_nav_node:
  ros__parameters:
    debug_image_enabled: false
```

#### 회피 방향이 매 프레임 뒤집혀서 제자리에서 좌우로 떨다가 결국 충돌하던 문제 (수정됨)

디버그 화면을 붙인 뒤 실제 영상으로 확인된 문제입니다. 장애물을 피하려고 회전하는 도중 카메라가 빠르게 움직이며 모션 블러가 심해지면 `left_density`/`right_density`가 둘 다 거의 0으로 무너집니다(엣지 자체가 안 보이므로). 그런데 회전 방향을 **매 프레임마다 새로** `left_density < right_density`로 다시 결정하고 있었기 때문에, 이 노이즈 수준의 값 비교가 프레임마다 뒤집히면서 좌회전↔우회전을 빠르게 반복했습니다(사용자가 관찰한 "제자리에서 좌우로 빠르게 움직이는" 증상). 그러다 우연히 한 프레임이 "CLEAR"로 읽히면 그 순간 바로 전진해버려서, 손바닥을 대고 있어도 통과해서 부딪히는 일이 생겼습니다.

수정한 동작:

1. **회전 방향은 장애물을 처음 감지했을 때 한 번만 정하고, `obstacle_avoidance_hold_seconds`(기본 1초) 동안은 이후 프레임이 뭐라 하든 그 방향을 그대로 유지합니다.** 매 프레임 다시 판단하지 않습니다.
2. 방향을 정할 때 `left_density`와 `right_density`의 차이가 `steer_direction_noise_floor`(기본 0.01)보다 작으면 — 즉 노이즈 수준이면 — 그 비교를 신뢰하지 않고 이전에 정했던 방향을 그대로 쓰거나(처음이면 기본값으로 오른쪽을) 씁니다.
3. **장애물이 안 보인다고 바로 전진하지 않습니다.** hold 시간이 다 지났고, 그 후로도 `obstacle_clear_confirm_frames`(기본 5프레임) 연속으로 장애물이 없다고 나와야 그제서야 전진을 재개합니다. 이벤트: `clearing obstacle: confirming clear (N/5)`.
4. 장애물이 없는 정상 주행 중의 좌우 조향값도 지수평활(EMA, `steer_smoothing_alpha` 기본 0.4)을 적용해서 프레임 노이즈로 인한 미세한 좌우 떨림을 줄였습니다.
5. 카메라 프레임이 무효 판정(블러 등으로 `camera view is invalid`)을 받고 복구된 직후에도, 장애물 회피 직후와 동일하게 저속부터 다시 램프업합니다(무효 판정 직후 바로 이전 속도로 튀어나가지 않도록).

```yaml
vision_nav_node:
  ros__parameters:
    obstacle_avoidance_hold_seconds: 1.0
    obstacle_clear_confirm_frames: 5
    steer_direction_noise_floor: 0.01
    steer_smoothing_alpha: 0.4
```

**튜닝 참고**: 여전히 충돌 직전에 전진을 재개한다면 `obstacle_avoidance_hold_seconds`와 `obstacle_clear_confirm_frames`를 올려서 더 오래 확인하게 하세요. 회피 회전이 너무 오래 걸린다 싶으면 반대로 낮추세요. 디버그 화면의 `OBSTACLE`/`CLEAR` 전환이 이제 훨씬 안정적으로(자주 안 뒤집히고) 보여야 합니다 — 여전히 프레임마다 뒤집힌다면 hold/confirm 값을 늘려서 대응하세요.

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

### 위치 가르치기 - 방법 2: 이동 없이 지금 위치만 빠르게 저장

`기억 시작` 없이 그냥 `<이름>으로 저장해`(또는 "현재 작은방이야. 저장해"처럼 자연어로)만 말해도 저장됩니다. 이 경우 재생 가능한 이동 경로는 없지만(그래서 이 이름으로 `정문앞에 택배가 있는지 보고와`처럼 "가서 확인" 임무는 못 시킵니다 - 갈 방법을 모르니까), 현재 화면의 시각적 특징(ORB)은 저장되므로 나중에 그 장면을 다시 봤을 때 인식/대조용으로 쓸 수 있습니다.

```bash
curl -X POST http://127.0.0.1:8080/api/chat -H "Content-Type: application/json" -d '{"message":"작은방으로 저장해"}'
```

응답 이벤트가 두 경우를 구분해서 알려줍니다:
- `location saved: 정문 (5 steps, with return path)` — 이동 경로 + 시각 특징 모두 저장(방법 1).
- `location saved: 작은방 (현재 위치의 시각 특징만 기억됨, 이동 경로 없음 - ...)` — 시각 특징만 저장(방법 2).

이전 버전에서는 `기억 시작` 없이 저장을 시도하면 "no recorded moves to save"로 조용히 실패해서 아무것도 기억되지 않았습니다 - 이제는 항상 뭔가는(최소한 시각 특징은) 저장됩니다.

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

## 9. 회피 중 제자리 회전(무한 스핀) 수정

방향 플래핑(8-2절 이전 항목, 커밋 `2139244`)을 고친 뒤에도 실제 영상에서는 여전히 "제자리에서 계속 회전"하는 문제가 남아 있었습니다. 디버그 오버레이로 확인한 원인은 방향 플래핑과는 다른, 한 단계 더 근본적인 문제였습니다.

- **원인**: 회전 자체가 매 프레임을 블러(motion blur)시키고, 그 블러가 `detect_floor_color_obstacle`/`detect_sudden_bottom_change`(정지된 카메라를 가정하고 만든 검사들)를 다시 "장애물 있음"으로 잘못 트리거합니다. 그 결과 "회전 → 블러 → 장애물 오탐 → 계속 회전"이라는 자기 강화 루프에 갇혀서, `obstacle_clear_streak`가 `obstacle_clear_confirm_frames`까지 쌓일 기회 자체가 없었습니다(회피 중에는 항상 회전 중이라 항상 블러 상태였기 때문).
- **수정 1 — 스터터 턴(stutter turn)**: 회피 중에는 계속 회전하는 대신 `obstacle_turn_pulse_seconds`(기본 0.3초) 회전 → `obstacle_pause_seconds`(기본 0.2초) 완전 정지를 반복합니다. 정지 구간에서는 블러가 사라지므로 그 프레임들만큼은 장애물 유무를 신뢰성 있게 읽을 수 있고, 실제로 클리어됐다면 그 정지 프레임들에서 `obstacle_clear_streak`가 쌓여 정상적으로 빠져나올 수 있습니다.
- **수정 2 — 최대 회피 시간 상한(안전망)**: 위 방식으로도 정말 빠져나오지 못하는 극단적 상황(예: 정말로 사방이 막힌 좁은 공간)에 대비해, `obstacle_avoidance_max_seconds`(기본 6초)를 넘겨 계속 회피 중이면 완전 정지하고 `/ccai/events`에 타임아웃 이벤트를 남깁니다. "무한 회전"의 최악의 경우를 6초 정지로 강제 상한선을 둔 것입니다 — 관리자가 상태를 보고 개입할 수 있습니다.
- 관련 파라미터(`config/robot.yaml`의 `vision_nav_node`): `obstacle_avoidance_max_seconds`, `obstacle_turn_pulse_seconds`, `obstacle_pause_seconds`.
- 코드: `vision_nav_node.py`의 `compute_patrol_command()`.

## 10. 키보드 수동 조작 (전/후/좌/우/정지)

관리자가 웹채팅에서 키보드(방향키 또는 W/A/S/D, Space=정지)로 로봇을 직접 조작할 수 있습니다.

- **동작 방식**: 키를 누르고 있는 동안 이동하고, 떼는 순간 정지합니다(key down = drive, key up = stop). 텍스트 입력창에 포커스가 있을 때는 동작하지 않아서 채팅 타이핑과 충돌하지 않습니다. 화면에는 키보드 대체용 터치/마우스 버튼 패드도 함께 제공됩니다(모바일 등 물리 키보드가 없는 환경 대비).
- **구현**: 브라우저에서 키 이벤트가 발생하면 기존 `/api/chat` 엔드포인트로 이미 있는 텍스트 명령("앞으로 가", "뒤로 가", "좌회전해", "우회전해", "정지")을 그대로 전송합니다 — 새 명령 타입이나 LLM 라우팅 없이 `mission.py`의 직접 패턴 매칭 경로를 그대로 재사용하므로 지연이 거의 없습니다.
- **연속 이동으로 통일**: 이 기능을 위해 `turn_left`/`turn_right`도 `move_forward`/`move_backward`와 동일하게 "정지 명령까지 계속 회전"하는 방식으로 바뀌었습니다(이전에는 회전만 `manual_turn_seconds`(0.8초) 동안의 짧은 넛지였음). 위치 가르치기(`기억 시작` 녹화 중)는 여전히 고정 시간 넛지를 기록해야 재생 가능하므로 그 경우에는 기존 방식(고정 시간)을 그대로 유지합니다.
- 코드: `web_chat_node.py`의 `HTML_PAGE`(키보드/버튼 JS), `patrol_node.py`의 `start_manual_move()`/`drive_loop()`의 `MANUAL_DRIVE` 분기.

## 11. 위치별 시각 특징 인식 (teach-and-repeat 보강)

기존 teach-and-repeat(8절)는 순수하게 "녹화된 이동 시퀀스를 그대로 재생"하는 방식이라, 오도메트리가 없어 누적된 드리프트로 실제로는 다른 곳에 도착해도 확인할 방법이 없었습니다. 이제 위치를 저장할 때 그 자리의 시각적 특징(ORB 키포인트 디스크립터)도 함께 저장해서, 나중에 그 위치로 이동했을 때 실제로 그 장면이 맞는지 시각적으로 대조합니다.

- **저장 시**: `<이름>으로 저장해`로 위치를 저장하면, `patrol_node`가 `vision_nav_node`에 `/ccai/location_feature_request`(`{"action":"capture","label":"정문"}`)를 보냅니다. `vision_nav_node`는 현재 프레임에서 OpenCV ORB(`cv2.ORB_create(nfeatures=300)`)로 키포인트 디스크립터를 추출해 base64로 인코딩한 뒤 `/ccai/location_feature_result`로 돌려주고, `patrol_node`가 이를 `data/locations.json`의 해당 위치에 `features`/`keypoints`로 저장합니다.
- **도착 시**: 저장된 이동 시퀀스 재생이 끝나 도착하면(`REPLAYING` 상태 종료), 저장된 특징이 있으면 `{"action":"match", ...}` 요청을 보내 현재 프레임의 디스크립터와 BFMatcher(Hamming 거리, Lowe's ratio test 0.75)로 비교합니다. 결과는 "visual check at 정문: 일치 (good matches=38, ratio=0.42, keypoints=91)" 같은 형태로 이벤트에 남아 텔레그램/웹채팅에서 확인할 수 있습니다. 일치율(`match_ratio`)이 낮으면 "불일치 가능성 (다른 곳일 수 있음)"으로 표시되어, 드리프트로 엉뚱한 곳에 도착했을 가능성을 관리자가 바로 알 수 있습니다.
- **하위 호환**: 특징 없이(이전 버전에서) 저장된 위치는 `features`가 빈 문자열이라 매칭을 건너뛰고 기존처럼 동작합니다. 평평하고 특징이 거의 없는 장면(흰 벽 등)은 키포인트가 거의 안 나올 수 있는데, 이 경우 "no distinct visual features found" 이벤트를 남기고 이동 시퀀스만으로 저장을 유지합니다(완전 실패시키지 않음).
- 관련 코드: `locations.py`(`features`/`keypoints` 필드, `set_features`/`get_features`), `vision_nav_node.py`(`on_location_feature_request`, `extract_orb_features`, `encode_descriptors`/`decode_descriptors`, `match_orb_features`), `patrol_node.py`(`save_recorded_location`, `on_location_feature_result`, `REPLAYING` 도착 분기).
- **한계**: SLAM/실제 위치추정이 아니라 "이 장면이 이전에 본 장면과 시각적으로 비슷한가"만 확인하는 수준입니다. 조명이 크게 바뀌거나 가구가 옮겨지면 일치율이 낮게 나올 수 있습니다. 그래도 "전혀 확인 안 함"보다는 훨씬 나은 신뢰도를 제공합니다.

관련 파라미터 (`robot.yaml` → `patrol_node`): `locations_file` (기본 `data/locations.json`).

## 12. 좌/우회전 방향 반전 수정

실제 로봇에서 좌/우회전이 반대로 동작하는 게 확인됐습니다(키보드로 왼쪽 화살표를 눌렀는데 오른쪽으로 도는 등). `jetbot_hardware_node`의 `left_trim`/`right_trim`을 `-1.0`으로 바꿔서 전진/후진 반전 문제를 고쳤던 이전 수정(§2 참고)이 좌우 바퀴에 똑같이 곱해지는 값이라, 회전 방향의 물리적 의미까지 같이 뒤집혔던 것입니다. `patrol_node.py`에서 `turn_left`/`turn_right` 명령이 만드는 `angular.z` 부호를 서로 바꿔서 다시 맞췄습니다(`drive_loop()`의 `MANUAL`/`MANUAL_DRIVE`/`REPLAYING` 세 분기 모두).

**주의**: 이 수정 이전에 `기억 시작`으로 녹화해서 저장한 위치(`data/locations.json`)가 있다면, 그 안에 저장된 `turn_left`/`turn_right` 스텝은 이제 반대 의미로 재생됩니다(가르칠 때는 옛 방향 규칙으로 실제 로봇이 올바르게 움직이도록 명령을 냈을 것이므로, 재생 시점의 규칙이 바뀌면 그 경로가 틀어집니다). 이 수정 이후에 만든 위치부터는 문제없고, 기존에 저장된 위치는 다시 가르치는 것을 권장합니다.

## 13. VLM 온디맨드 분석 실패를 즉시 보이게 함

"영상 분석해"/"정문앞에 택배가 있는지 보고와" 같은 **관리자가 직접 요청한** VLM 분석이 실패하면(H200 vLLM 엔드포인트 연결 불가, 타임아웃, 인증 오류 등) 예전에는 주기적 위험 감시용 오류 억제 로직(`error_event_min_interval_seconds`, 30초)에 걸려서 조용히 묻힐 수 있었습니다 — 관리자 입장에서는 "분석해달라고 했는데 아무 반응이 없다"로 보였습니다.

- `vlm_client_node.on_trigger()`가 이 요청이 온디맨드 트리거였는지(`was_triggered`) 기억해서 `analyze_image()`까지 전달합니다.
- 온디맨드 요청이 실패하면 억제 없이 즉시 `vlm analysis failed: ...` 이벤트를 `/ccai/events`로 발행합니다(텔레그램/웹채팅에 바로 뜸). 반면 5초 주기 자동 위험 감시 실패는 기존처럼 30초 억제를 유지합니다(엔드포인트 장애 시 스팸 방지).
- 이 자체가 근본 원인(엔드포인트 연결 문제 등)을 고치진 않지만, **왜 분석이 안 됐는지**를 관리자가 바로 알 수 있게 해서 실제 원인 진단이 가능해집니다. 여전히 아무 이벤트도 안 뜬다면 명령 파싱 이전 단계(예: 카메라 프레임이 전혀 안 들어오는 등) 문제일 가능성이 높습니다.

## 14. 프리뷰 화면과 영상분석(디버그) 화면이 서로 다르게 보이던 문제

웹채팅에는 카메라 원본(`#camera`)과 장애물 감지 디버그 오버레이(`#visionDebug`) 두 화면이 나란히 있는데, 실제로는 순찰/수동전진(`patrol`/`manual_drive`+forward) 상태일 때만 디버그 화면이 갱신되고 있었습니다. `vision_nav_node.on_image()`가 `compute_patrol_command()`(장애물 판정 + 디버그 프레임 발행을 모두 담당)를 오직 그 상태에서만 호출했기 때문에, 로봇이 대기/수동회전/추적 등 다른 상태일 때는 디버그 화면이 마지막 순찰 시점 그대로 멈춰 있어 실시간 카메라 화면과 달라 보였습니다. 사용자가 보고한 "실제 프리뷰영상과 영상분석 프레임이 다르다"는 바로 이 증상입니다.

- **수정**: 장애물 판정 로직을 `analyze_obstacle(frame)`으로 분리해서 **모드와 무관하게 매 유효 프레임마다** 실행하고, 디버그 프레임도 매번 발행합니다(`describe_obstacle()`로 상태 설명 텍스트 생성). 실제로 주행 결정을 내려야 하는 상태(`patrolling`/`manual_drive`+forward)에서는 이미 계산된 신호를 그대로 `compute_patrol_command(frame, signals)`에 넘겨서 회피 상태 머신(§9)을 적용하고, 자체적으로 더 자세한 디버그 텍스트로 다시 발행합니다. 계산은 프레임당 한 번만 하고, 디버그 오버레이는 항상 최신 프레임을 반영합니다.
- **디버그 확인 보강**: 디버그 프레임 좌상단에 `frame #N @ <시각>` 타임스탬프를 추가해서, 필요하면 실제 프리뷰와 디버그 화면의 시간 차이를 눈으로 직접 비교/검증할 수 있게 했습니다.
- 이 수정으로 회피 중 제자리 회전(§9)의 원인 진단이 그동안 실제와 다른(지연된) 디버그 화면을 보고 있었을 가능성도 배제됩니다 — 이제 순찰 중이 아니어도 항상 실시간 장애물 판정을 볼 수 있습니다.
- 참고로 YOLO 추론과 프레임 급변/바닥색 대비 검사가 이제 순찰 중이 아닐 때도 매 프레임 돌아가므로 유휴 상태의 CPU 사용량이 약간 늘어날 수 있습니다(YOLO는 `yolo_detect_every_n_frames`로 계속 스로틀됨).

## 15. 바닥색 대비 오탐으로 전진을 못 하던 문제

실제 영상(2026-07-23 22:12 녹화)의 디버그 오버레이 텔레메트리를 프레임 단위로 확인한 결과, 진짜 장애물이 없는 평범한 장면에서도 `color=True`(바닥색 대비 초과)가 거의 항상 떠 있었습니다. 관찰된 `color` 값 예시: 50.4, 57.1, 63.5, 64.2, 66.8, 73.0, 73.8, 78.0, 106.4, 108.2 — 전부 기존 임계값 `floor_color_diff_threshold`(40.0)를 훌쩍 넘습니다. 반면 같은 프레임들에서 `edge`(0.023~0.072, 정지 임계값 0.16보다 한참 낮음)와 `sudden`(대부분 1 미만, 임계값 35보다 한참 낮음)은 계속 "장애물 없음"으로 정상 판정하고 있었습니다.

- **원인**: `detect_floor_color_obstacle()`은 "바퀴 바로 아래" 기준 밴드와 "그보다 살짝 앞" 밴드의 평균 색상 차이를 봅니다. 이 방/바닥(무늬 있는 장판, 분홍 조명, 어안렌즈로 인한 원근 왜곡)에서는 실제 장애물이 없어도 이 두 밴드가 원래 색이 다르게 보여서, 임계값 40.0이 이 환경의 정상적인 배경 잡음보다도 훨씬 낮았습니다. 결과적으로 4개 신호를 OR로 묶는 조합에서 `color` 신호가 사실상 항상 켜져 있어 나머지 3개 신호가 전부 "이상 없음"이라고 해도 계속 장애물로 판정되고, 회피 상태 머신이 끊임없이 재진입해서 전진을 못 하고 있었습니다.
- **수정**: `floor_color_diff_threshold`를 40.0 → 130.0으로 올렸습니다(이번 영상에서 관측된 "장애물 아님" 최대값 108.2보다 여유 있게 높은 값). 엣지 밀도/YOLO/프레임 급변 3개 신호는 그대로 유지되므로 진짜 장애물 감지 능력 자체가 없어지는 건 아니고, 이 특정 환경에서 신뢰도가 낮았던 신호 하나의 민감도만 낮췄습니다.
- **한계/후속 조치**: 이 임계값은 바닥재·조명·카메라 렌즈에 따라 집마다 다를 수 있습니다. 만약 이후에도 `color` 값이 계속 새 임계값 근처거나 넘는 게 보이면(웹채팅 디버그 오버레이의 `color=` 수치로 확인 가능) `robot.yaml`의 `vision_nav_node.floor_color_diff_threshold`를 그 환경의 정상 배경값보다 높게 추가 조정하세요.

## 16. 정지 명령이 키를 누르지 않아도 계속 나가던 문제 (키보드 조작 버그)

키보드 텔레옵(§10) 추가 이후 실제 사용에서 "정지" 명령이 관리자가 아무 키도 누르지 않았는데도 반복적으로 전송되는 문제가 있었습니다. 채팅창에 "현재방은 작은방이야 기억해"처럼 한글 문장을 입력하는 도중에 이 현상이 나타났습니다.

- **원인**: 키보드 텔레옵은 물리 키의 `event.code`(예: `KeyW`, `KeyA`, `KeyS`, `KeyD`)로 이동 명령을 매핑합니다. 그런데 두벌식 한글 자판에서는 흔한 자음/모음(ㅈ, ㅁ, ㄴ, ㅇ 등)이 바로 이 W/A/S/D 물리 키에 배정돼 있어서, 한글 문장을 입력하는 것만으로도 이 키들의 keydown/keyup 이벤트가 자연스럽게 발생합니다. `keydown` 핸들러는 입력창에 포커스가 있으면 무시하도록 이미 처리돼 있었지만, **`keyup` 핸들러에는 이 검사가 빠져 있었습니다.** 그래서 입력창에 타이핑 중 keydown은 무시되어 `pressedKeys`가 계속 비어있는 상태였는데, keyup에서는 `pressedKeys.size === 0` 조건이 (원래 눌렸던 키가 없어도) 항상 참이 되어 W/A/S/D를 뗄 때마다 `정지`가 매번 전송됐습니다. 한글 단어 하나 타이핑할 때마다 그 안에 포함된 자음/모음 수만큼 "정지"가 반복 전송된 것입니다.
- **왜 심각한가**: 이 버그는 단순히 로그를 지저분하게 만드는 것에 그치지 않습니다 — 로봇이 순찰/수동 전진 중일 때 관리자가 채팅창에 무언가(질문, 위치 이름 등)를 타이핑하면, 그 타이핑 자체가 "정지" 명령을 계속 로봇에 보내서 **전진이 끊기는 원인이 될 수 있습니다.** "역시 전진하지 못해" 증상의 또 다른 원인일 가능성이 있습니다.
- **수정**: `keyup` 핸들러에도 `keydown`과 동일하게 입력창 포커스 여부(`isTypingTarget`) 검사를 추가했습니다. 추가로: (1) 탭/창이 포커스를 잃을 때(`blur`) 눌려 있던 것으로 기록된 키를 정리하고 정지 명령을 보내서, 알트탭 등으로 keyup이 아예 발생하지 않는 경우에도 로봇이 계속 움직인 채로 남지 않게 했습니다. (2) 화면 버튼(터치패드)에서 `touchstart`/`touchend` 이후 브라우저가 호환성을 위해 만들어내는 합성 `mousedown`/`mouseup` 이벤트가 명령을 중복 전송하지 않도록 별도 플래그로 막았습니다. (3) `sendCommand()`에 150ms 내 동일 명령 중복 전송을 걸러내는 안전장치를 추가했습니다.
- 코드: `web_chat_node.py`의 `HTML_PAGE` 내 키보드/버튼 JS(`keyup` 리스너, `blur` 리스너, `touchActive` 플래그, `sendCommand()`).

## 17. 영상분석 요청에 응답이 없을 때 확인할 것

"LLM online" 표시는 `llm_control_node`의 텍스트 명령 라우팅 연결 상태만 나타냅니다. `vlm_client_node`(카메라 이미지를 실제로 분석하는 쪽)는 별도 노드이고, 같은 vLLM 엔드포인트라도 **이미지(멀티모달) 요청**은 텍스트 요청보다 훨씬 느리고 별도로 실패할 수 있습니다. §13에서 온디맨드 분석 실패는 이제 즉시 이벤트로 보이도록 고쳤으니, 최신 코드가 배포된 상태에서 "영상 분석해"를 보냈는데 **"requesting camera analysis" 이벤트조차 안 뜬다면** 명령이 `analyze`로 파싱되지 않았거나 `CCAI_ENABLE_VLM`이 꺼져 있는 것이고, **"vlm analysis failed: ..." 이벤트가 뜬다면** 그 메시지가 원인(타임아웃, 연결 거부, 인증 오류 등)입니다. 70B 모델의 이미지 추론은 텍스트보다 느릴 수 있어 `vlm_client_node.request_timeout_seconds`를 20 → 35초로 늘렸습니다. 확인 순서:

```bash
docker exec ccai-jetbot bash -c "echo \$CCAI_ENABLE_VLM"          # 비어있거나 1이어야 함(기본 활성화)
docker logs ccai-jetbot 2>&1 | grep -i vlm_client                 # vlm_client_node 시작/에러 로그
docker exec ccai-jetbot bash -c "echo \$CCAI_VLLM_API_BASE_URL"   # H200 vLLM 엔드포인트 URL 확인
```

`host_docker_update.sh`로 최신 코드를 반영한 뒤 다시 테스트해 주세요 — 이 문서의 §13(즉시 실패 표시) 수정 전 버전에서는 실패가 30초 억제 로직에 묻혀 안 보였을 수 있습니다.

## 18. 위치 지정 후 순찰(inspect) 요청 시 제자리 무한 회전 - 확정 원인

"정문 순찰해줘"처럼 **위치가 있는 임무**를 요청했을 때 로봇이 제자리에서 끝없이 회전하는 문제의 확정 원인을 찾았습니다.

- **경로**: 그 위치가 §11의 "방법 2"(이동 없이 시각 특징만 저장)로 저장된 경우, `start_replay()`는 재생할 이동 시퀀스가 없다는 걸 확인하고 `start_inspect("", question)`로 폴백해서 `PatrolState.INSPECTING` 상태로 들어갑니다.
- **버그**: `drive_loop()`의 `INSPECTING` 분기는 `twist.angular.z = angular_speed`만 실행하고 있었는데, **이 상태를 벗어나는 조건이 코드 어디에도 없었습니다.** VLM 분석 결과가 도착해도(`on_vlm_observation`) 상태 전환은 하지 않고 이벤트만 발행했으므로, 한 번 INSPECTING에 들어가면 그 뒤로는 관리자가 수동으로 `정지`를 보내기 전까지 영원히 회전만 계속했습니다. "위치 지정 후 순찰 요청 시 제자리 무한 회전"은 바로 이 케이스였습니다.
- **수정**:
  1. `drive_loop()`의 `INSPECTING` 분기는 이제 회전하지 않고 제자리에 멈춥니다(어차피 VLM은 트리거 시점에 도착한 프레임 한 장만 분석하므로 회전이 더 잘 보이게 해주지 않습니다).
  2. `on_vlm_observation()`에서 분석 결과가 도착하면(`pending_analysis` 처리 완료 시) 상태가 `INSPECTING`이면 `STOPPED`로 전환하고 완전히 정지시킵니다 — 이제 이 상태에 진짜 출구가 생겼습니다.
  3. `start_replay()`가 이동 경로 없는 위치로 폴백할 때 위치 이름을 잃어버리지 않고 "location 'X' has no travel path (visual-only save) - inspecting from current position instead" 이벤트를 남기도록 고쳐서, 왜 그 자리에서 멈췄는지(왜 실제로 이동하지 않았는지) 관리자가 바로 알 수 있게 했습니다.
- **여전히 남는 제약**: 시각 특징만 저장된 위치는 여전히 "거기까지 가는 경로"가 없으므로 실제로 그 위치로 이동은 못 합니다(§11에서 이미 문서화된 한계). 그 위치까지 실제로 순찰하게 하려면 §8의 "방법 1"(`기억 시작` → 이동 → 저장)로 다시 가르쳐야 합니다.
- 코드: `patrol_node.py`의 `drive_loop()` INSPECTING 분기, `on_vlm_observation()`, `start_replay()`.

## 19. D435i 깊이 카메라 도입과 CSI 카메라 역할 재정의 (객체 인식 전용)

CSI 카메라를 천장을 보도록 마운트를 바꾸고, 전방에는 실측 깊이를 재는 Intel RealSense D435i를 새로 답니다. 자세한 배경과 다음 단계(RTAB-Map/Nav2)는 [docs/navigation_roadmap.md](navigation_roadmap.md)의 "D435i 도입" 절을 참고하세요. 여기서는 구현 요약만 남깁니다.

- **새 노드 `depth_nav_node`**: D435i 깊이 이미지를 좌/중/우 3분할해 중앙값 실측 거리(m)로 장애물을 판정합니다. §9에서 CSI용으로 검증한 커밋-유지/클리어확인/스터터턴/최대회피시간 상태 머신을 그대로 재사용하되, 입력이 실측 거리라서 텍스처·색·조명에 흔들리는 §15류의 오탐이 구조적으로 없습니다. 순찰 중 장애물이 없으면 더 열린(먼) 방향으로 조향합니다. `patrol_node`가 이미 구독하는 `/ccai/vision_cmd_vel`/`/ccai/vision_status`에 그대로 발행하므로 `patrol_node`는 무수정입니다.
- **CSI 카메라 = 객체 인식 전용**: `vision_nav_node`에 `drive_enabled` 파라미터(기본 `true`)를 추가했습니다. D435i를 켜면 `false`로 바꿔서, CSI는 YOLO 객체 인식/사람 따라가기/디버그 오버레이는 계속하되 주행 명령은 더 이상 내지 않습니다(천장을 보는 카메라로는 바닥 장애물 검사 자체가 의미 없으므로).
- **D435i 연결 이후 기본값**: `depth_nav_node.enabled: true`, `vision_nav_node.drive_enabled: false`로 전환했습니다 — D435i가 순찰/수동전진 주행을 맡고, CSI+YOLO는 객체 인식/사람 따라가기/디버그 용도로 계속 동작합니다. D435i 없이 예전처럼 CSI만으로 순찰하려면 두 값을 되돌리세요.
- **컨테이너 재생성 필요**: `CCAI_ENABLE_DEPTH_NAV` 환경변수와 D435i용 USB 디바이스 마운트(`scripts/host_docker_run.sh` 참고)는 `docker run` 시점에 고정됩니다. 이미 떠 있는 컨테이너를 `docker restart`만 하면 적용되지 않으니, `scripts/host_docker_run.sh`를 다시 실행해서 컨테이너를 새로 만들어야 합니다.
- **설치**: `scripts/install_realsense_d435i.sh`(컨테이너 안에서 실행) — librealsense2를 소스로 빌드(Jetson arm64용 apt 패키지가 없어서, 커널 패치 불필요한 `-DFORCE_RSUSB_BACKEND=true` 유저스페이스 백엔드 사용)하고 `realsense-ros`도 소스 빌드합니다. 스크립트 안 태그/브랜치 이름은 이 환경에서 실시간 검증되지 않았다는 점을 스크립트 주석에도 남겼습니다 — 클론이 실패하면 실제 저장소에서 현재 태그를 확인하세요.
- 활성화 절차: D435i 연결 → `realsense2_camera` 런치로 `/camera/camera/depth/image_rect_raw` 발행 확인 → `robot.yaml`에서 `depth_nav_node.enabled: true`, `vision_nav_node.drive_enabled: false` → `CCAI_ENABLE_DEPTH_NAV=1`로 스택 실행.
- 코드: `ccai_jetbot_patrol/depth_nav_node.py`(신규), `vision_nav_node.py`(`drive_enabled`), `launch/patrol.launch.py`(`CCAI_ENABLE_DEPTH_NAV`), `config/robot.yaml`(`depth_nav_node` 블록), `scripts/install_realsense_d435i.sh`(신규).

## 20. 웹 프리뷰에 D435i 주행 뷰 추가 (주행가능 바닥 오버레이 + 위치 라벨)

D435i의 RGB 영상을 실제 카메라 프리뷰로 쓰고, 그 위에 주행가능 바닥(깊이 기반)과 현재 모드/위치를 오버레이로 표시해달라는 요청으로 추가했습니다.

- **D435i 컬러 스트림 활성화**: `patrol.launch.py`의 realsense 런치 인자에서 `enable_color`를 `false`→`true`로 바꿨습니다(`rgb_camera.color_profile: 640x480x30`). 기존에는 대역폭을 아끼려고 depth만 켰었는데, 이제 이 컬러 영상 자체가 오버레이의 바탕이 됩니다.
- **`depth_nav_node`가 오버레이 프레임 생성**: depth 이미지에서 계산하는 좌/중/우 3분할 거리 신호(§19에서 이미 주행 판단에 쓰던 것)를 그대로 재사용해서, 컬러 프레임 위에 3개 구간을 색으로 칠합니다 — **초록=열림(장애물 정지거리의 2배 이상), 노랑=주의, 빨강=장애물(정지거리 이내)**. 상단에 OBSTACLE/CLEAR 상태, 하단에 `mode=patrolling target=정문`처럼 현재 모드와 위치(목적지 이름 등, `/ccai/status`의 `target` 필드)를 라벨로 표시하고, 그 아래에 최근 주행 판단 문구(예: `depth path left=1.20m center=2.40m right=0.90m steer=-0.15 ramp=1.00`)도 같이 넣습니다. `/ccai/depth_debug_image`로 발행됩니다.
- 깊이 프레임과 컬러 프레임은 별도 토픽/타이밍으로 도착하므로 엄격히 동기화하지 않고, 컬러 프레임이 새로 도착할 때마다 **가장 최근에 계산된** 깊이 신호를 사용해 오버레이를 그립니다(약간의 시간차는 있을 수 있지만 방향 표시 목적에는 충분).
- **웹 UI**: 프리뷰 영역 맨 앞에 "D435i 주행 뷰" 패널을 추가했습니다(`/api/depth_debug.jpg`, 150ms 주기 갱신 - 기존 CSI 카메라/디버그 패널과 동일한 방식). 기존 CSI 카메라 패널은 "CSI 카메라 (객체 인식용)"로, CSI 디버그 패널은 "CSI 장애물 감지 디버그"로 라벨을 바꿔서 각 패널이 뭘 보여주는지 명확히 했습니다.
- 관련 파라미터(`robot.yaml` → `depth_nav_node`): `color_image_topic`(기본 `/camera/camera/color/image_raw`), `debug_image_enabled`.
- 코드: `depth_nav_node.py`(`on_color_image`, `publish_debug_frame`, `analyze_depth`/`describe_signals`로 신호 계산과 발행 로직 분리), `web_chat_node.py`(`/api/depth_debug.jpg`, `depthDebug` 패널), `launch/patrol.launch.py`(`enable_color: true`).
- **한계**: depth와 color 센서의 정확한 픽셀 정렬(`align_depth`)까지는 하지 않았습니다 — 좌/중/우 3등분 정도의 대략적인 방향 표시 목적에는 이 정도로 충분하다고 판단했습니다. 픽셀 단위로 정밀하게 맞추려면 `align_depth.enable:=true`를 realsense 런치 인자에 추가하고 정렬된 토픽을 구독하도록 확장이 필요합니다.
