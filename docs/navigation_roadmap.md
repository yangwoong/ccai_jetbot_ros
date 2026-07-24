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

## 자동 탐색·라벨링 (구현됨, v1)

**v1으로 구현 완료** (2026-07-24): 관리자가 매번 수동으로 `기억 시작`부터 가르치지 않아도, "자율탐색" 임무를 주면 로봇이 스스로 돌아다니며 주기적으로 멈춰 관리자에게 위치 이름을 물어보고, 그 답변으로 위치를 저장합니다. 자세한 사용법과 구현은 [docs/vision_and_alerts.md §21](vision_and_alerts.md)을 참고하세요. 원래 계획했던 "VLM이 스스로 현관/복도/주방을 분류"하는 방식이 아니라, **관리자가 라벨을 직접 제공**하는 방식으로 구현했습니다(사용자 요청에 따른 설계 — VLM 자동 분류보다 정확하고, 관리자가 원하는 이름을 붙일 수 있음).

- `patrol_node`에 `PatrolState.EXPLORING` 추가. 주행 자체는 이미 있는 반응형 회피/열린공간추종 로직(D435i 기반, §9·§19)을 그대로 재사용합니다 — 새 주행 알고리즘은 만들지 않았습니다.
- 위치 저장은 이미 있는 `LocationStore`/§11의 시각 특징 캡처 방식을 그대로 재사용합니다(수동으로 가르치나 탐색 중 발견하나 저장 형식은 동일).

## 다음 작업 계획

1. **여러 위치를 잇는 그래프**: 지금은 각 위치가 "시작 지점에서부터의" 독립적인 시퀀스이거나(teach-and-repeat) 시각 특징만 있는 경우(탐색/빠른 저장)입니다. 위치 A에서 위치 B로 가는 경로처럼 위치 간 상대 이동까지 다루려면 그래프 구조로 확장이 필요합니다.
2. **구역 태깅된 이상 알림**: 5단계(이미 구현된 위험 알림)에 현재 위치의 지역 라벨을 붙여서 "복도에서 이상 감지: ..." 처럼 어디서 발생했는지 알 수 있게 합니다.
3. **VLM 기반 자동 분류(선택)**: 관리자가 라벨을 안 줘도 VLM이 "여긴 어디야?" 질문으로 스스로 분류하는 옵션을 자율탐색에 추가할 수 있습니다(`vlm_client_node`의 질문 기반 분석을 그대로 재사용 가능) — 지금은 관리자 라벨링만 구현했습니다.
4. **진짜 SLAM/점유 격자 지도**: D435i 도입 절에서 이미 언급한 `rtabmap_ros` + `nav2` — 여전히 별도의 큰 작업입니다.

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
- D435i가 실제로 연결된 뒤(2026-07-24)부터는 `config/robot.yaml` 기본값을 `depth_nav_node.enabled: true`, `vision_nav_node.drive_enabled: false`로 바꿔서 D435i가 순찰/수동전진 주행을 담당합니다. CSI+YOLO는 여전히 객체 인식/사람 따라가기/디버그 오버레이 용도로 계속 동작합니다("기존 기능은 문제없도록 유지" 요건 — 주행 담당만 D435i로 넘어가고 나머지는 그대로). D435i 없이 다시 CSI만으로 순찰하려면 그 두 값을 원래대로(`false`/`true`) 되돌리세요.
- `launch/patrol.launch.py`가 `CCAI_ENABLE_DEPTH_NAV=1`일 때 `depth_nav_node`뿐 아니라 `realsense2_camera` 드라이버 자체도 같이 실행합니다(`rs_launch.py`를 `enable_depth:=true enable_color:=false`로 include) — 별도로 `ros2 launch realsense2_camera ...`를 수동 실행할 필요가 없습니다. 다른 방식으로 이미 realsense 드라이버를 띄워둔 상태라면 `CCAI_ENABLE_REALSENSE_DRIVER=0`으로 이 부분만 끌 수 있습니다.
- `scripts/host_docker_run.sh`는 이제 `CCAI_ENABLE_DEPTH_NAV`(안전모드 여부에 따라 기본 0/1)를 컨테이너에 전달하고, D435i가 쓰는 raw USB 버스 접근(`-v /dev/bus/usb:/dev/bus/usb`, `--device-cgroup-rule "c 189:* rmw"`)을 자동으로 추가합니다. **기존에 떠 있는 컨테이너를 `docker restart`만 하면 이 새 환경변수/디바이스 마운트가 적용되지 않습니다** — `docker run` 시점에 고정되므로, 반드시 `scripts/host_docker_run.sh`를 다시 실행해서 컨테이너를 재생성해야 합니다.

### 아직 안 된 것 (다음 단계)

- **자동 지역 탐색/라벨링과의 통합**: SLAM 지도가 안정적으로 검증되면, 지금은 시각 특징/타이밍 기반인 위치 저장을 그 좌표계 위에 앉히는 방식으로 재설계할 수 있습니다.

## SLAM/Nav2 포기 → 경량 커스텀 좌표 컨트롤러로 전환 (2026-07-24)

`rtabmap_ros`/`nav2`(`navigation2`, `nav2_bringup`)/`robot_localization` 4개 패키지 모두 이 이미지의 apt 저장소에 **없는 것으로 확정**됐습니다(`scripts/install_slam_nav2.sh` 실행 결과, 전부 `E: Unable to locate package`). 이 이미지는 ROS2 Humble을 bionic에 백포트한 커스텀 조합이라 흔한 패키지도 종종 빠져 있는데(`xacro`/`diagnostic_updater` 전례), 이번엔 그 빠진 패키지가 각각 수십 개 하위 패키지짜리 큰 스택(`nav2`)이거나 g2o/GTSAM/PCL 같은 무거운 C++ 코어(`rtabmap`)를 필요로 해서, Jetson Nano 4GB RAM으로 소스 빌드를 시도하는 건(이미 pycuda/librealsense2 빌드에서 여러 번 겪은 메모리 문제 감안) 이번 세션의 "한 패키지씩 대응" 전례와는 위험도가 다르다고 판단했습니다.

**사용자가 명시적으로 선택한 방향**: "경량 대안: 직접 만든 좌표 컨트롤러." Nav2/rtabmap 전체를 대체하는 대신, 이 프로젝트 자체에서 다음 세 가지를 새로 구현했습니다.

1. **`visual_odom_node.py`(신규)**: D435i의 컬러+정렬된깊이 프레임으로 직접 만든 RGB-D 시각 오도메트리. ORB 특징점 매칭 → 깊이로 3D 역투영 → Kabsch 알고리즘(SVD 기반 강체변환 추정)으로 프레임 간 카메라 이동을 구해 누적 `(x, y, yaw)` 포즈를 `/ccai/odom_pose`로 발행하고 `odom`→`base_link` TF도 방송합니다. **루프 클로저도, 포즈 그래프 최적화도, 점유 격자 지도도 없는 순수 프레임 간 오도메트리라 시간이 지나면 반드시 드리프트합니다** — 한 세션 안에서 "대략 원래 자리로 돌아가는" 용도지, 튜닝된 진짜 SLAM 스택의 정확도를 대신하지 못합니다.
2. **`patrol_node`의 점-대-점 컨트롤러 (`PatrolState.POSE_GOAL`)**: 목표 좌표까지 방향 오차가 크면 제자리 회전으로 먼저 정렬하고, 아니면 목표에 가까워질수록 속도를 줄이며 전진 + 소폭 방향 보정하는 단순 비례 제어(`compute_steering_twist`/`compute_pose_goal_twist`). `depth_nav_node`의 `obstacle_now` 신호를 안전 우선순위로 확인해서, 실제 장애물이 감지되면 그 회피 명령이 이 컨트롤러의 명령을 덮어씁니다.
3. **커버리지 기반 탐색 알고리즘 (`pick_explore_subgoal`/`compute_explore_twist`)**: rtabmap 없이는 진짜 점유 격자가 없으므로, 대신 시각 오도메트리 좌표를 굵은 격자 셀로 나눠 방문 횟수를 기록하고(`self.visited_cells`), 주기적으로 로봇 주변 여러 방향/지점 후보를 샘플링해서 **가장 덜 가본 셀**로 이어지는 후보를 골라 그 지점까지 위 점-대-점 컨트롤러로 이동합니다. 이게 원래 문제("좌우로 피하기만 하고 안 가본 길을 못 찾음")를 직접 겨냥한 부분 — 이제 탐색 중 이미 지나온 곳인지 아닌지를 실제로 구분합니다.

### 활성화 절차

```bash
CCAI_ENABLE_VISUAL_ODOM=1 ./scripts/host_docker_run.sh
```
```yaml
# config/robot.yaml
patrol_node:
  ros__parameters:
    explore_frontier_mode: true   # true면 EXPLORING이 커버리지 탐색 알고리즘으로 주행, false면 기존 반응형 그대로
visual_odom_node:
  ros__parameters:
    enabled: true
```

둘 다 기본값이 꺼져 있어서, 아무것도 건드리지 않으면 기존 반응형 `depth_nav_node` 순찰이 그대로 동작합니다(안전망 유지).

"정문으로 가"/"정문에 뭐 있는지 확인해줘" 같은 임무는 `patrol_node.start_replay()`가 저장된 위치에 `pose`가 있고 최근 오도메트리 신호(`has_recent_odom()`, 기본 2초 이내)가 있으면 위 점-대-점 컨트롤러(`POSE_GOAL`)로 실제 좌표까지 이동합니다. `pose`가 없거나 오도메트리가 끊겼으면 기존 타임드 재생/시각 전용 위치 확인으로 자동 폴백합니다.

### 아직 검증 안 된 것 (정직하게)

- **카메라↔로봇 축 매핑 미검증**: 카메라 광학 좌표계(x=오른쪽, y=아래, z=전방)와 로봇 평면 좌표계(x=전방, y=왼쪽, yaw=위에서 봤을 때 반시계)의 대응이 실기에서 검증되지 않았습니다. 방향이 반대로 나오면 `visual_odom_node`의 `yaw_sign`/`lateral_sign`/`forward_sign` 파라미터(기본 전부 `1.0`)를 `-1.0`으로 뒤집어서 조정해야 합니다.
- **드리프트 정도 미측정**: ORB 매칭+Kabsch 정확도가 이 조명/텍스처 환경에서 실제로 얼마나 드리프트하는지 실기 데이터가 없습니다. `pose_goal_tolerance_m`(기본 0.15m)이 그 드리프트 대비 너무 빡빡하거나 헐거울 수 있습니다.
- **커버리지 격자 파라미터 미튜닝**: `explore_visited_cell_size_m`(0.5m), `explore_step_distance_m`(0.8m), `explore_candidate_count`(8)는 첫 합리적 추정값이지 실기로 조정된 값이 아닙니다.
- **`align_depth.enable` 대역폭/성능 영향 미확인**: D435i가 컬러+깊이+정렬 파이프라인을 동시에 돌릴 때의 USB 대역폭 여유는 실측하지 않았습니다(과거 "USB CAM overflow" 전례 있음 - 위 §D435i 도입 참고).

## 관련 문서

- 현재 구현된 YOLO 자율 주행/따라가기, VLM 위험 알림, 카메라 지연 수정 등은 [docs/vision_and_alerts.md](vision_and_alerts.md)에 정리되어 있습니다.
- 카메라/하드웨어 설정은 [docs/hardware_jetbot.md](hardware_jetbot.md), 배포/운영은 [docs/docker_host_operations.md](docker_host_operations.md)를 참고하세요.
