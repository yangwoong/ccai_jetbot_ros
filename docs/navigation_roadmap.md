# 핵심 임무 로드맵: 지역 인지 순찰

이 프로젝트의 핵심 목표를 기록합니다. YOLO와 LLM(Qwen3-VL-70B)은 이 파이프라인의 핵심 구성요소이며, 아래 우선순위 원칙에 따라 개발합니다.

## 핵심 파이프라인 (목표)

```
1. 이동 가능 바닥면적/장애물 검출 및 회피 (YOLO + 카메라)
   ↓
2. 이동 가능 지역 탐색 및 라벨링 (LLM으로 현관/복도/엘리베이터실/주방/거실 등 주변 상황 인지)
   ↓
3. 내비게이션 지도 생성
   ↓
4. 관리자로부터 순찰 지역 임무 할당
   ↓
5. 순찰하며 이상 상황 감지 및 이벤트 메시지 전송
```

## 우선순위 원칙

1. **임무 수행 안정성이 최우선입니다.** 이동 범위 탐색·장애물 회피가 끊김 없이 동작하는 것이 최신 YOLO 버전을 쓰는 것보다 중요합니다. OpenCV DNN(`cv2.dnn`)으로 YOLOv8 ONNX를 이 Jetson Nano(OpenCV 4.5.0)에서 돌렸을 때 CUDA/CPU 백엔드 둘 다 실측으로 실패한 적이 있어서, **OpenCV DNN 임포터를 우회하고 NVIDIA 자체 TensorRT 런타임(`trtexec`로 빌드한 `.engine` + `ccai_jetbot_patrol/tensorrt_yolo.py`)으로 전면 전환**했습니다. TensorRT가 이 플랫폼에 맞는 정식 추론 경로이기 때문입니다. 그래도 항상 "TensorRT 엔진 → OpenCV DNN ONNX(CUDA→CPU) → HOG/엣지 밀도"의 3단계 자동 폴백을 유지합니다 — 특정 조합이 불안정하면 주저 없이 더 낮은 단계로 내려가되(필요하면 YOLOv8n → YOLOv5n처럼 모델 자체를 낮추는 것도 포함), 로봇이 어떤 상황에서도 최소한 엣지 밀도 회피 + HOG까지는 계속 동작해야 합니다. 자세한 내용은 [docs/vision_and_alerts.md](vision_and_alerts.md)를 참고하세요.
2. **LLM(Qwen3-VL-70B)을 최대한 활용합니다.** 현재는 명령 라우팅(`llm_control_node`)과 위험 요약(`vlm_client_node`) 두 곳에만 쓰이고 있습니다. 아래 2단계(지역 라벨링), 4단계(임무 할당 시 지역명 해석), 5단계(이상 상황 설명)에도 VLM 호출을 적극적으로 추가해야 합니다. 모델은 `.env`의 `CCAI_VLLM_MODEL`로 지정하며 기본값을 `Qwen/Qwen3-VL-70B-Instruct`로 맞춰뒀습니다.

## 단계별 현재 구현 상태

| 단계 | 상태 | 담당 코드 |
|---|---|---|
| 1. 바닥면적/장애물 검출·회피 | **부분 구현 (안정화됨)** | `vision_nav_node`: 엣지 밀도 기반 주행 가능 영역 추정 + YOLO 바운딩박스 + 바닥색 대비 + 프레임 급변을 결합. 방향 플래핑(커밋 `2139244`)과 회피 중 제자리 무한 회전([vision_and_alerts.md §9](vision_and_alerts.md)) 두 가지 실제 충돌 원인을 모두 수정했습니다(스터터 턴 + 최대 회피 시간 상한). 다만 픽셀 단위 "주행 가능 면적" 세그멘테이션은 아직 없고, 엣지 밀도/객체 박스로 간접 추정하는 수준입니다. 키보드 수동 조작([vision_and_alerts.md §10](vision_and_alerts.md))으로 관리자가 언제든 자율 회피를 즉시 오버라이드할 수 있습니다. |
| 2. 지역 탐색 및 라벨링 (LLM) | **부분 구현 (teach-and-repeat + 시각 특징 확인)** | 자동 탐색·자동 라벨링(로봇이 스스로 돌아다니며 이름 붙이기)은 아직 없습니다. 관리자가 수동으로 가르치는 v1: `기억 시작` → 이동 명령들 → `X로 저장해`로 그 경로를 이름 X에 저장(`locations.py`, `data/locations.json`). v1.1로 ORB 시각 특징 캡처/매칭을 추가해서, 이동 시퀀스 드리프트로 엉뚱한 곳에 도착했는지 도착 시점에 시각적으로 확인할 수 있게 됐습니다([vision_and_alerts.md §11](vision_and_alerts.md)). VLM에 의한 자동 분류(현관/복도/주방 등 스스로 인식)는 여전히 미구현이며 다음 단계입니다. |
| 3. 내비게이션 지도 생성 | **부분 구현 (teach-and-repeat)** | 오도메트리·IMU·SLAM이 없어 미터 단위 지도는 만들 수 없습니다. 위 2단계에서 저장한 이동 시퀀스 자체가 "위상 지도"의 최소 형태입니다(이름 → 그 위치까지 가는 timed move 시퀀스). 자동 탐색으로 여러 지점을 잇는 그래프 구조는 아직 없습니다. |
| 4. 관리자로부터 지역 임무 할당 | **구현됨 (v1)** | "정문앞에 택배가 있는지 보고와" 같은 자연어 → LLM이 `{"type":"inspect","target":"정문","text":"택배가 있는지 확인해줘"}`로 해석 → `patrol_node`가 "정문"이 저장된 위치면 그 경로를 재생해서 이동 후 질문에 대해 VLM에게 물어보고 결과를 이벤트로 보고합니다. 위치를 모르면 그 자리에서 확인하고 "위치를 모른다"고 알립니다. |
| 5. 순찰 중 이상 감지 및 알림 | **구현됨** | `vlm_client_node`(RISK/NORMAL 판단) → `patrol_node`(`/ccai/events` 발행) → `telegram_bridge_node`/`web_chat_node`(관리자에게 전달). [docs/vision_and_alerts.md](vision_and_alerts.md) 참고. |

## 구현된 것: 수동 teach-and-repeat 위치 (v1)

오도메트리 없이 진짜 미터 단위 SLAM 지도를 만드는 건 이 하드웨어(휠 인코더/IMU 없음)로는 현실적이지 않습니다. 그래서 "정확한 좌표"가 아니라 "이 이름으로 가려면 이렇게 움직이면 된다"는 **녹화된 이동 시퀀스**를 위치로 취급하는 v1을 구현했습니다.

- **가르치기**: 관리자가 `기억 시작` → `앞으로 가`/`좌회전해` 등으로 로봇을 실제로 이동시킴 → `정문으로 저장해`. 그동안의 timed move 목록이 `data/locations.json`에 이름별로 저장됩니다 (`ccai_jetbot_patrol/locations.py`).
- **임무 수행**: "정문앞에 택배가 있는지 보고와" → LLM이 `inspect`(target="정문", text="택배가 있는지 확인해줘")로 해석 → `patrol_node`가 "정문"이 저장돼 있으면 그 시퀀스를 재생해서 이동 → 도착 후 `vlm_client_node`에 그 질문을 그대로 물어봐서 답을 받아 `/ccai/events`로 보고(`analysis result: 정문: ...`). 텔레그램/웹채팅에 그대로 뜹니다.
- **모르는 위치**: 저장 안 된 이름이면 그 자리에서 확인하고 "위치를 모른다"고 알려줘서, 관리자가 가르쳐야 한다는 걸 바로 알 수 있습니다.
- 관련 코드: `ccai_jetbot_patrol/locations.py`, `patrol_node.py`의 `start_inspect`/`start_replay`/`save_recorded_location`, `vlm_client_node.py`의 질문 기반 분석(`/ccai/vlm_trigger`에 `{"question": "..."}`).
- **v1.1 (시각 특징 확인)**: 저장 시 ORB 키포인트 디스크립터도 함께 캡처하고, 도착 시 현재 프레임과 대조해서 "일치/불일치 가능성"을 이벤트로 보고합니다. 자세한 내용은 [vision_and_alerts.md §11](vision_and_alerts.md)을 참고하세요.

## 다음 작업 계획 (자동 탐색·라벨링으로 확장)

지금은 관리자가 수동으로 가르쳐야 합니다. 다음 단계는 로봇이 스스로 탐색하며 이름을 붙이는 것입니다.

1. **자동 탐색 모드**: `patrol_node`에 새 상태(예: `EXPLORING`)를 추가해서, 일정 거리/시간마다 정지하고 VLM에게 "이 장면이 어떤 공간인지(현관/복도/엘리베이터실/주방/거실/기타)" 분류를 요청하는 탐색 임무를 수행합니다. `vlm_client_node`에 이미 있는 질문 기반 분석(`{"question": "..."}` 트리거)을 재사용해서 "여긴 어디야?" 같은 분류 질문을 던지면 됩니다.
2. **자동 위치 저장**: 탐색 중 분류된 지점을 `LocationStore`에 자동으로 저장합니다(이미 있는 teach-and-repeat 저장 형식 그대로 재사용 — 수동으로 가르치나 자동으로 발견하나 저장 형식은 같습니다).
3. **여러 위치를 잇는 그래프**: 지금은 각 위치가 "시작 지점에서부터의" 독립적인 시퀀스입니다. 위치 A에서 위치 B로 가는 경로처럼 위치 간 상대 이동까지 다루려면 그래프 구조로 확장이 필요합니다.
4. **구역 태깅된 이상 알림**: 5단계(이미 구현된 위험 알림)에 현재 위치의 지역 라벨을 붙여서 "복도에서 이상 감지: ..." 처럼 어디서 발생했는지 알 수 있게 합니다.

다음에 진행할 때는 1번(자동 탐색 + 분류 질문)부터 시작하는 것을 권장합니다 — 이미 있는 `LocationStore`와 질문 기반 VLM 분석을 그대로 재사용할 수 있기 때문입니다.

## 카메라 하드웨어 고려사항: 모노 RGB vs T265 vs D435i

실제 충돌이 반복되면서 카메라를 Intel RealSense T265/D435i로 바꾸는 것도 검토했습니다. 결론을 기록해둡니다.

- **T265**(스테레오 어안 + IMU, visual-inertial odometry 전용)는 **장애물 회피를 해결하지 못합니다.** 깊이/거리를 재는 센서가 아니라 위치·자세(odometry)를 주는 센서입니다. 대신 이 로드맵의 3단계("내비게이션 지도 생성"이 오도메트리가 없어서 막혀있는 문제)를 T265가 직접 해결해줄 수 있습니다 — 즉 회피용이 아니라 지도/위치추정용 카메라입니다.
- **D435i**(RGB + 실측 깊이 + IMU)는 **장애물 회피 문제 자체에 맞는 센서입니다.** 지금 쓰는 엣지 밀도/바닥색 대비/YOLO 조합은 전부 깊이 센서가 없어서 어쩔 수 없이 쓰는 시각적 우회 추정입니다. 실제 거리를 픽셀 단위로 재는 D435i가 있으면 텍스처/색과 무관하게 훨씬 안정적으로 장애물을 감지할 수 있습니다.
- 현재 방침: 모노 카메라 기반 방식(엣지 밀도 + YOLO + 바닥색 대비 + 프레임 급변, [docs/vision_and_alerts.md](vision_and_alerts.md) 참고)을 먼저 안정화 시도하고, 그래도 충돌이 반복되면 D435i 도입을 권장합니다. T265는 회피 문제와는 별개로, 나중에 3단계(지도 생성)를 진행할 때 다시 고려 대상입니다.

## D435i 도입 (2026-07-23 ~): 실측 깊이 기반 장애물 회피 + CSI 카메라 역할 재정의

모노 카메라 방식을 계속 안정화해봤지만 결국 실측 깊이 센서 도입을 결정했습니다. 동시에 CSI 카메라는 천장을 보도록 마운트를 바꾸고 객체 인식 전용으로만 씁니다(바닥을 볼 수 없으니 장애물 회피용 프록시 신호들은 더 이상 의미가 없음). 아래는 이번에 구현한 것과, 앞으로 남은 것을 구분해서 기록합니다.

### 이번에 구현한 것

- **`scripts/install_realsense_d435i.sh`**: librealsense2를 소스에서 빌드(Jetson은 arm64 apt 패키지가 없고, `-DFORCE_RSUSB_BACKEND=true`로 커널 UVC 드라이버 패치 없이 유저스페이스 백엔드로 빌드 — Jetson 공식 권장 방식)하고, `realsense-ros`(ROS2 래퍼)도 소스로 빌드합니다(`.deb`가 존재하지 않는 arm64 `librealsense2` apt 패키지에 의존하므로 apt 설치는 의존성 해결이 안 됨). udev 규칙도 설치합니다. 컨테이너 안에서 `container_build.sh`와 같은 위치에서 실행합니다. **주의**: 스크립트 안의 태그/브랜치 이름(`REALSENSE_TAG`, `REALSENSE_ROS_BRANCH`)은 이 환경에서 실시간 검증된 값이 아닙니다 — 클론이 실패하면 [librealsense releases](https://github.com/IntelRealSense/librealsense/releases)와 [realsense-ros](https://github.com/IntelRealSense/realsense-ros) 저장소에서 현재 태그/브랜치를 확인하고 환경변수로 지정해서 재실행하세요.
- **`depth_nav_node`** (새 노드, `ccai_jetbot_patrol/depth_nav_node.py`): D435i의 실측 깊이 이미지(`depth_image_topic`, 기본 `/camera/camera/depth/image_rect_raw`)를 받아 전방을 좌/중/우 3분할로 나눠 중앙값 거리를 계산합니다. 중앙 거리가 `obstacle_stop_distance_m`(기본 0.45m)보다 가까우면 장애물로 판정합니다. CSI 버전에서 검증된 상태 머신(방향 커밋-유지, 클리어 확인 프레임, 스터터 턴, 최대 회피 시간 상한 — [docs/vision_and_alerts.md](vision_and_alerts.md) §9)을 그대로 재사용하되, 신호 자체가 실측 거리라서 텍스처/색/조명에 흔들리지 않습니다. 장애물이 없을 때는 좌우 중 더 열린(거리가 먼) 쪽으로 조향하는 "열린 공간 추종" 방식으로 순찰 중 자율 주행을 시도합니다. `patrol_node`가 이미 쓰는 `/ccai/vision_cmd_vel`/`/ccai/vision_status` 토픽에 그대로 발행하므로 `patrol_node`는 수정할 필요가 없습니다 — CSI용 `vision_nav_node`와 이 노드 둘 다 같은 토픽에 발행할 수 있는 구조라, 아래처럼 하나만 활성화합니다.
- **`vision_nav_node.drive_enabled`** 파라미터(기본 `true`): `false`로 두면 CSI 카메라는 YOLO 객체 인식/사람 따라가기/디버그 오버레이는 계속하되, 순찰/수동전진 시 주행 명령(`/ccai/vision_cmd_vel`) 발행은 멈춥니다. D435i가 연결되면 이 값을 `false`로, `depth_nav_node.enabled`를 `true`로 맞춰서 "CSI=객체인식 전용, D435i=주행" 역할 분리를 완성합니다.
- 기본값(D435i 미연결 상태)은 `depth_nav_node.enabled: false`, `vision_nav_node.drive_enabled: true`라서 **아무것도 바꾸지 않으면 기존 CSI+YOLO 순찰 동작이 그대로 유지**됩니다 — 이번 요청의 "기존 기능은 문제없도록 유지" 조건을 만족합니다.

### 아직 안 된 것 (다음 단계)

- **진짜 SLAM/점유 격자 지도**: 이번 구현은 어디까지나 "가까우면 피하고, 열린 쪽으로 간다"는 반응형(reactive) 주행입니다. 오도메트리나 지도가 없어서 "이 방을 다 돌아봤다", "특정 좌표로 가라" 같은 진짜 내비게이션은 아직 안 됩니다. 현실적인 다음 단계는 `rtabmap_ros`(RGB-D 카메라만으로 시각 오도메트리 + SLAM 지도 작성이 가능 — 휠 인코더 불필요)를 D435i 데이터로 돌리고, 그 위에 `nav2` 스택(costmap, 경로 계획, 컨트롤러)을 얹는 것입니다. 이건 별도의 큰 작업이라 이번에는 포함하지 않았고, 위 설치 스크립트가 librealsense/realsense-ros를 먼저 준비해두는 선행 작업 역할을 합니다.
- **자동 지역 탐색/라벨링과의 통합**: 2단계(지역 탐색·라벨링)는 여전히 관리자가 수동으로 가르치는 teach-and-repeat 방식입니다. `rtabmap_ros` 지도가 생기면 그 좌표계 위에 라벨을 앉히는 방식으로 재설계할 수 있습니다.

## 관련 문서

- 현재 구현된 YOLO 자율 주행/따라가기, VLM 위험 알림, 카메라 지연 수정 등은 [docs/vision_and_alerts.md](vision_and_alerts.md)에 정리되어 있습니다.
- 카메라/하드웨어 설정은 [docs/hardware_jetbot.md](hardware_jetbot.md), 배포/운영은 [docs/docker_host_operations.md](docker_host_operations.md)를 참고하세요.
