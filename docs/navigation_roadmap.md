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

### 호스트 Wi-Fi(iwlwifi)가 반복 크래시하는 문제 (2026-07-25, 해결됨)

호스트의 Intel AC 8265 Wi-Fi 카드가 `dmesg`에 아래 패턴을 반복하며, SSH/웹 접속이 계속 느려지거나 끊기는 문제가 실기에서 발생했습니다.

```
iwlwifi 0000:01:00.0: Queue 2 stuck for 10000 ms.
iwlwifi 0000:01:00.0: Microcode SW error detected. Restarting 0x2000000.
```

**디버깅 과정에서 배제된 원인들** (전부 실기로 확인):
- PCIe ASPM 절전 상태 — 런타임 정책(`performance`)과 커널 부트 인자(`pcie_aspm=off`) 둘 다 걸어봤지만 크래시가 똑같이 반복됨. 완전히 배제.
- CPU 기아(starvation) — 크래시 구간의 `tegrastats`가 CPU 25~35%대로 전혀 포화 상태가 아니었음. 배제.
- 약한 신호/채널 품질 — `iw dev wlan0 link`에서 `signal: -28~-29 dBm`(매우 좋음), `tx bitrate: 300 MBit/s`, `tx retries: 1`, `tx failed: 0`. 배제.
- D435i/`visual_odom_node`의 USB 부하 — `CCAI_ENABLE_VISUAL_ODOM=0`(align_depth 끔, visual_odom_node 자체를 안 띄움)으로도 재현 시도했으나 무관하다는 게 최종적으로는 아래 확정 원인으로 대체됨.

**확정 원인**: 유선 랜을 연결한 채 무선 IP로 SSH 접속해 두고 유선 케이블을 뽑는 재현 테스트로 확정했습니다. 유선이 끊기는 순간 그동안 `eth0`(낮은 metric, 우선 경로)를 타던 모든 연결(`telegram_bridge_node`의 폴링/전송, `vlm_client_node`의 클라우드 호출 등)이 한꺼번에 `wlan0`로 재시도되면서, **여러 연결이 동시에 몰리는 버스트 트래픽**이 발생합니다. 이게 이 카드/드라이버/펌웨어(로드된 펌웨어 버전 `22.391740.0`) 조합의 802.11n 프레임 집계(aggregation) 큐 처리 버그를 건드려 워치독이 걸리는 것으로 확인됐습니다. 컨테이너 최초 기동 직후(194초 시점)에 크래시가 시작됐던 것도 같은 메커니즘 — 여러 노드가 동시에 외부로 연결을 시도하며 트래픽이 몰린 것.

**적용한 수정**: `scripts/host_fix_iwlwifi_stability.sh`(신규) — `/etc/modprobe.d/iwlwifi.conf`에 `options iwlwifi 11n_disable=1 power_save=0 uapsd_disable=1`을 써서 11n 프레임 집계를 끕니다. **재부팅이 있어야 적용됩니다** (모듈 옵션이라 iwlwifi가 다시 로드될 때만 반영). `host_docker_run.sh`가 매번(D435i 활성화 여부와 무관하게, 이 문제 자체가 D435i와 무관했으므로) 이 스크립트를 실행해서 설정 파일이 없거나 다르면 새로 씁니다.

**검증**: 재부팅 후 무선 IP로 SSH 접속한 채 유선 케이블을 뽑는 재현 테스트를 다시 했고, 이번엔 크래시/연결 끊김이 전혀 발생하지 않았습니다. 확정.

이제 쓸모없어진 `host_fix_iwlwifi_aspm.sh`(ASPM 완화책)는 삭제했습니다.

### 새로 발견된 문제: `soctherm: OC ALARM` + D435i가 USB에서 다시 사라짐 (2026-07-25, 미해결)

Wi-Fi 문제를 재부팅으로 재현 테스트하던 중 콘솔에 `soctherm: OC ALARM 0x00000001`이 연속으로 찍히기 시작했고, 그 시점에 `lsusb -t`를 보면 D435i가 USB 트리에서 다시 사라져 있었습니다(`Bus 02` 5000M 허브 아래에 Video 인터페이스들이 없음).

**추정 (미검증)**: `OC ALARM`은 Jetson Nano의 SoC 전류 모니터링(soctherm)이 과전류를 감지했다는 경보입니다. Jetson Nano는 기본 마이크로 USB 전원(5V/2A, 10W)으로는 D435i(USB3 고대역폭 스트리밍) + CSI 카메라 + Wi-Fi + 모터를 동시에 감당하기 빠듯한 것으로 잘 알려져 있어서, 과전류가 감지되면 보호 차원에서 포트 전원이 끊기며 D435i가 USB에서 탈락했을 가능성이 있습니다.

**다음에 확인할 것**:
1. 현재 이 로봇이 마이크로 USB(5V/2A)로 전원을 받는지, 아니면 DC 배럴잭(5V/4A) + J48 점퍼 방식인지 확인.
2. 마이크로 USB라면 배럴잭 전원(5V/4A 이상 정격 어댑터) + J48 점퍼로 전환 — Jetson Nano에서 이 증상의 표준적인 해결책.
3. 전환 후에도 `OC ALARM`이 재발하는지, D435i가 USB에서 안정적으로 유지되는지 재확인.

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

### 버그: `explore_frontier_mode: true`로 켜도 알고리즘이 안 바뀌던 문제 (2026-07-25, 수정됨)

실기 테스트에서 `explore_frontier_mode: true` + `visual_odom_node.enabled: true`로 켰는데도 로봇이 기존과 똑같이 제자리 회전만 반복하고 새 커버리지 탐색으로 전혀 나아가지 못하는 문제가 발생했습니다("알고리즘에 변화가 없어").

**원인**: `depth_nav_node.py`의 주행 조건이 `self.mode in ("patrolling", "exploring", "pose_goal")`이라, `explore_frontier_mode`가 켜져 있어도 `patrol_node`가 `EXPLORING`/`POSE_GOAL` 상태이기만 하면 depth_nav_node는 상관없이 **자기 자신의(예전) 반응형 좌우 조향 알고리즘을 계속 계산해서 `/ccai/vision_cmd_vel`/`/ccai/vision_status`(obstacle_now 포함)에 발행**하고 있었습니다. 그런데 `patrol_node.drive_loop()`의 안전 오버라이드가 "`obstacle_now`가 true면 depth_nav_node가 최근에 발행한 twist를 그대로 채택"하는 방식이었는데, 로봇이 가구/벽으로 둘러싸인 방 안을 탐색하는 동안 `obstacle_now`는 사실상 거의 항상 true라서 — **이 오버라이드가 거의 매 tick 발동해서 새 커버리지 알고리즘의 twist를 depth_nav_node의 옛 반응형 twist로 계속 덮어썼습니다.** 즉 새 알고리즘 코드 자체는 정상 동작했지만, 안전장치가 사실상 항상 옛 알고리즘으로 되돌리고 있었던 것 — "알고리즘 변화 없음"이라는 증상과 정확히 일치합니다.

**수정**:
1. `depth_nav_node.py`에 `explore_frontier_mode` 파라미터(패트롤 노드와 동일한 값으로 `robot.yaml`에서 맞춰줌) 추가 — 이게 true면 `EXPLORING` 상태에서 더 이상 주행하지 않고(=경쟁하는 cmd_vel을 안 보냄) 장애물 센싱(`obstacle_now`)만 계속 최신 상태로 발행합니다. `POSE_GOAL`은 항상 depth_nav_node가 주행하지 않도록 이미 되어 있었습니다(패트롤 노드의 점-대-점 컨트롤러 전용).
2. `patrol_node.py`의 안전 오버라이드를 "depth_nav_node의 twist를 그대로 채택"에서 "`obstacle_now`면 그냥 정지(`stop_motion()`)"로 단순화 — EXPLORING의 경우 정지와 함께 `self.explore_sub_goal = None`으로 목표도 버려서, 다음 tick에 막힌 방향이 아닌 새 후보 방향을 다시 샘플링하도록 했습니다.

**검증**: 코드 리뷰로 원인을 특정하고 수정했습니다 — 위 두 파일 모두 컴파일 확인은 했지만, 이 특정 수정이 실기에서 실제로 "제자리 회전" 없이 전진하는지는 아직 재검증이 필요합니다.

### 아직 검증 안 된 것 (정직하게)

- **카메라↔로봇 축 매핑 미검증**: 카메라 광학 좌표계(x=오른쪽, y=아래, z=전방)와 로봇 평면 좌표계(x=전방, y=왼쪽, yaw=위에서 봤을 때 반시계)의 대응이 실기에서 검증되지 않았습니다. 방향이 반대로 나오면 `visual_odom_node`의 `yaw_sign`/`lateral_sign`/`forward_sign` 파라미터(기본 전부 `1.0`)를 `-1.0`으로 뒤집어서 조정해야 합니다.
- **드리프트 정도 미측정**: ORB 매칭+Kabsch 정확도가 이 조명/텍스처 환경에서 실제로 얼마나 드리프트하는지 실기 데이터가 없습니다. `pose_goal_tolerance_m`(기본 0.15m)이 그 드리프트 대비 너무 빡빡하거나 헐거울 수 있습니다.
- **커버리지 격자 파라미터 미튜닝**: `explore_visited_cell_size_m`(0.5m), `explore_step_distance_m`(0.8m), `explore_candidate_count`(8)는 첫 합리적 추정값이지 실기로 조정된 값이 아닙니다.
- **`align_depth.enable` 대역폭/성능 영향 미확인**: D435i가 컬러+깊이+정렬 파이프라인을 동시에 돌릴 때의 USB 대역폭 여유는 실측하지 않았습니다(과거 "USB CAM overflow" 전례 있음 - 위 §D435i 도입 참고).

## 방-단위 탐색: 45도 회전 + LLM 출입구 탐지 + 위상 그래프 지도 (2026-07-26, 실기 미검증)

커버리지 탐색(위 §)의 점-대-점 컨트롤러가 `visual_odom_node`의 yaw 추적에 의존하는데, 실기에서 회전 중 yaw가 실제 회전량을 제대로 못 따라간다는 게 확인되면서(§ "explore_frontier_mode... 버그" 참고) 그 컨트롤러 자체가 회전에서 못 벗어나는 문제가 있었습니다. 사용자 요청으로 **오도메트리에 전혀 의존하지 않는 시간 기반 방식**으로 탐색 알고리즘을 완전히 새로 만들었습니다.

### 파이프라인

```
1. 제자리에서 45도씩(설정 가능) 시간 기반 회전 - 총 8단계로 360도
   ↓ (각 단계마다)
2. D435i 정면 영상을 VLM에 보내 "문/출입구가 보이는가" 질문 → 응답 파싱해서 기억
   ↓ (한 바퀴 다 돌면)
3. CSI 영상을 VLM에 보내 "천장 모양/공간 크기" 질문 → 기억
   ↓
4. 관리자에게 이 방의 이름을 물어봄 → LocationStore에도 동일 라벨로 등록(기존 "X로 가"/순찰 명령과 호환)
   ↓
5. 감지된 출입구 중 하나로 시간 기반 회전+직진 이동 → 새 방 진입 → 1번부터 반복
   ↓ (더 갈 출입구가 없으면)
6. 시간 기반 회전+직진으로 이전 방으로 후진(역추적) → 그 방의 다음 미탐색 출입구 시도
   ↓ (루트 방까지 후진했는데 더 갈 곳이 없으면)
7. 탐색 완료
```

### 왜 시간 기반(오도메트리 미사용)인지

`compute_steering_twist`(커버리지 탐색이 쓰는 컨트롤러)는 "목표 방향으로 정렬됐는가"를 `visual_odom_node`의 yaw로 판단하는데, 실기에서 로봇이 몇 초씩 회전해도 yaw가 거의 안 바뀌는 게 확인됐습니다(빠른 회전 중 ORB 추적이 깨지거나, 카메라-로봇 축 매핑의 스케일 자체가 안 맞을 가능성 - 둘 다 아직 미해결). 그래서 이번 방-단위 탐색은 **yaw를 아예 참조하지 않고**, 시간만큼만 회전합니다.

**첫 구현(2026-07-26)은 `angular_speed` 파라미터가 실제 물리적 rad/s와 같다고 가정한 공식(`(2π/N) / angular_speed`)을 썼는데, 실기에서 45도를 목표로 2바퀴 반을 도는 걸로 확인됐습니다** — 바퀴에 인코더가 없어서 이 공식이 가정하는 "명령값 = 실제 각속도" 관계 자체가 성립하지 않았던 것입니다. 그래서(2026-07-27) 공식을 버리고, **회전 전용의 낮은 속도(`explore_room_scan_angular_speed`, 기본 0.09)와 직접 튜닝하는 초 단위 시간(`explore_room_scan_turn_seconds`, 기본 1.0초)**으로 바꿨습니다 - 실기에서 스톱워치/각도기로 직접 재서 이 값을 맞추는 것 외에는 인코더 없이 검증할 방법이 없습니다. 여러 방을 거칠수록 방향 오차가 누적되는 건 여전한 정직한 한계입니다(아래 참고).

### 구현

- **`ccai_jetbot_patrol/explore_graph.py`**(신규): `RoomGraph` 클래스 - 각 방을 노드로, 감지된 출입구들과 부모 방(어느 출입구로 들어왔는지)을 저장. **진짜 도면이 아니라 위상 그래프**입니다(어느 방이 어느 출입구로 어느 방과 연결되는지만 기록, 치수/좌표 없음) - 점유 격자 SLAM이 없는 이 로봇 센서 구성으로는 실측 도면 자체가 불가능하다고 판단해서(이번 세션 초반 rtabmap/Nav2 포기 이유와 동일) 사용자와 상의해 이 방식으로 결정했습니다. `data/room_graph.json`에 저장.
- **`ccai_jetbot_patrol/patrol_node.py`**: `explore_room_scan_mode: true`(기본값, `explore_frontier_mode`보다 우선)일 때 `EXPLORING` 상태를 `tick_room_scan()`이 전담합니다. 회전(`rotating`) → VLM 질문(`await_vlm_step`, D435i) → (한 바퀴 다 돌면) 천장 질문(`await_ceiling`, CSI) → 라벨 확인(`await_label`) → 출입구로 이동(`advance_doorway`) 또는 이전 방으로 후진(`backtrack_advance`)의 상태기계. `depth_nav_node`의 `obstacle_now`(항상 최신으로 발행되는 실측 깊이 기반 신호)를 매 틱 확인해서, 장애물이 있으면 회전/직진 모두 즉시 멈춥니다.
- **`ccai_jetbot_patrol/launch/patrol.launch.py`**: `CCAI_ENABLE_EXPLORE_LLM=1`일 때 `vlm_client_node` 실행 파일을 **두 번째 인스턴스**(`vlm_explore_node`)로 띄우고, D435i의 컬러 압축 토픽(`/camera/camera/color/image_raw/compressed`)을 보게 합니다. 기존 `vlm_client_node`(CSI를 보며 상시 위험 감시)는 그대로 두고, 응답 토픽만 launch 레벨에서 `/ccai/vlm_explore_observation`으로 리매핑했습니다(코드 수정 없이 기존 노드를 그대로 재사용).
- **후진(역추적) 각도 계산**: 자식 방에 들어가면 그 방의 "0단계" 방향은 항상 "더 안쪽"이고 진입한 출입구는 항상 정반대(180도, 즉 `explore_room_scan_headings/2`단계)에 있다는 기하학적 사실을 이용합니다. 후진해서 부모 방에 재진입하면, 원래 나갈 때 썼던 출입구 번호(`k`)를 이용해 `-(k + N/2) mod N`만큼 추가 회전해서 부모 방의 원래 0단계 기준을 복원합니다 - 오도메트리 없이도 여러 방을 오가며 방향 기준을 일관되게 유지하기 위한 결정론적 보정입니다.
- **회전/정지 후 반드시 대기(settling) 후에만 VLM 호출**(2026-07-27 추가): 회전을 멈춘 직후 바로 사진을 보내면 카메라 파이프라인에 아직 회전 중에 찍힌 블러 프레임이 남아있을 수 있습니다. 그래서 정지 → `explore_room_scan_settle_seconds`(기본 1.0초) 대기 → 그제서야 VLM에 질문을 보내는 `settling` 단계를 회전 완료와 VLM 질문 사이에 반드시 거치도록 했습니다(회전 스텝, 마지막 천장 질문 전 모두 동일하게 적용).
- **출입구 판단 + 이동가능 필드 판단을 하나로 통합**(2026-07-27): 원래는 "문이 보이는가"만 물었는데, 사용자 요청으로 "이 방향으로 이동할 수 있는 열린 공간(문/통로/넓은 바닥 등)이 있는가"로 질문을 넓혔습니다(`GO:YES`/`GO:NO`). 문이 없어도 이동할 만한 열린 공간이 있으면 그 방향도 이동 후보로 기억합니다.
- **`depth_nav_node`의 장애물 미회피 버그 수정**(2026-07-27): `region_distance()`가 "유효한 깊이 값이 하나도 없으면 무조건 완전히 열려 있다고 판단"하는 버그가 있었습니다. D435i는 약 0.2m보다 가까우면 깊이를 못 읽는데, 로봇이 장애물에 거의 붙었을 때 정확히 이 상태가 되면서 "장애물 감지"가 "완전히 열림"으로 뒤집혀 충돌했습니다. 이제는 유효 범위 미만의 양수 깊이값이 몰려있으면(너무 가까워서 못 재는 상황) 장애물로 판단하도록 구분했습니다. `obstacle_stop_distance_m`도 0.45m → 0.6m로 올려서 여유를 더 뒀습니다.

### 활성화 절차

```bash
CCAI_ENABLE_DEPTH_NAV=1 CCAI_ENABLE_EXPLORE_LLM=1 ./scripts/host_docker_run.sh
```
```yaml
# config/robot.yaml (이미 기본값으로 반영됨)
patrol_node:
  ros__parameters:
    explore_room_scan_mode: true
```

### 아직 검증 안 된 것 (정직하게, 실기 테스트 전)

- **전체 파이프라인 실기 미검증**: 이 알고리즘 자체를 아직 로봇에서 완주해보지 못했습니다. 각 단계 전환/타이밍/VLM 파싱이 실제로 맞물려 돌아가는지 확인이 필요합니다.
- **회전 각도 캘리브레이션 미완료**: 공식 기반 계산(첫 시도, 2배수 이상 오버슈트 확인됨)은 폐기하고 직접 튜닝하는 `explore_room_scan_turn_seconds`/`explore_room_scan_angular_speed`로 바꿨지만, 아직 실기에서 "정확히 45도"가 되도록 그 값 자체를 맞춰보지는 못했습니다 - 기본값(1.0초, 0.09)은 첫 추정치일 뿐입니다.
- **`explore_room_scan_advance_seconds`(기본 4.0초)/`explore_room_scan_linear_speed`(기본 0.03) 미보정**: 출입구를 통과하는 데 필요한 실제 거리/시간을 이 로봇/이 환경 기준으로 튜닝하지 않았습니다.
- **VLM 응답 형식 신뢰도**: `GO:YES`/`GO:NO` 프리픽스로 응답해달라고 프롬프트에 요청했지만, LLM이 항상 정확히 그 형식을 지킬 거란 보장은 없습니다 - 느슨한 키워드 폴백(`출입구`/`통로`/`문`/`열린`/`공간`)도 같이 넣어뒀지만 오탐/누락 가능성이 있습니다.
- **`explore_room_scan_settle_seconds`(기본 1.0초) 미보정**: 블러 없는 프레임이 나오는 데 실제로 얼마나 걸리는지 실기로 확인 안 됐습니다 - 너무 짧으면 여전히 블러 프레임을 보낼 수 있고, 너무 길면 탐색이 불필요하게 느려집니다.
- **`depth_nav_node` 수정의 실기 검증 필요**: 장애물 미회피 버그 수정과 `obstacle_stop_distance_m` 상향(0.6m)이 실제로 충돌을 막는지 실기 확인이 필요합니다.
- **다중 출입구/복잡한 위상 구조**: 한 방에 출입구가 여러 개거나 그래프에 순환(같은 방으로 두 경로가 이어지는 경우)이 있으면 지금 구현은 그걸 감지하지 못하고 같은 방을 새 방으로 중복 등록할 수 있습니다 - 위치 재인식(기존 ORB 시각 특징 매칭)과의 연동은 아직 없습니다.
- **CSI 천장 "면적"은 실측이 아님**: `vlm_explore_node`가 아니라 기존 CSI용 `vlm_client_node`에 한 번 질문해서 받은 텍스트 설명 하나일 뿐이며, 실제 면적/치수 계산은 하지 않습니다.

## 관련 문서

- 현재 구현된 YOLO 자율 주행/따라가기, VLM 위험 알림, 카메라 지연 수정 등은 [docs/vision_and_alerts.md](vision_and_alerts.md)에 정리되어 있습니다.
- 카메라/하드웨어 설정은 [docs/hardware_jetbot.md](hardware_jetbot.md), 배포/운영은 [docs/docker_host_operations.md](docker_host_operations.md)를 참고하세요.
