import json
import re
from typing import Any, Dict

SAVE_LOCATION_PATTERN = re.compile(r"(?:여기를?\s*)?([가-힣A-Za-z0-9]+?)\s*(?:으로|로)\s*(?:저장|기억)")


class MissionCommand:
    def __init__(self, type: str, target: str = "", text: str = "", raw: str = "") -> None:
        self.type = type
        self.target = target
        self.text = text
        self.raw = raw


def parse_mission_command(message: str) -> MissionCommand:
    raw = message.strip()
    if not raw:
        return MissionCommand(type="status", raw=message)

    if raw.startswith("{"):
        try:
            payload: Dict[str, Any] = json.loads(raw)
            command_type = str(payload.get("type", payload.get("command", "status")))
            return MissionCommand(
                type=normalize_command_type(command_type),
                target=str(payload.get("target", "")),
                text=str(payload.get("text", "")),
                raw=message,
            )
        except json.JSONDecodeError:
            return MissionCommand(type="say", text=raw, raw=message)

    lowered = raw.lower()
    if lowered in {"status", "state", "report", "상태", "상태 확인", "보고", "/status"}:
        return MissionCommand(type="status", raw=message)
    if lowered in {"patrol start", "start patrol", "start", "run", "순찰 시작", "시작", "/start"}:
        return MissionCommand(type="patrol_start", raw=message)
    # "탐색과 순찰을 혼동하고 있어" - 탐색(exploring for new navigable area)
    # and 순찰(patrolling already-known space, checking video for anomalies)
    # are different missions with different stop handling. A message that
    # explicitly names one ("순찰 중지"/"탐색 중지") must stop THAT one
    # specifically - checked first, before the bare/ambiguous stop words
    # below, which used to hard-code every plain "정지"/"멈춰" to
    # patrol_stop even while actively exploring (confirmed on real
    # hardware: exploring never actually got the explore-specific cleanup
    # it needed - room_phase, awaiting_label state, etc - because that
    # code path was never reached).
    if lowered in {"patrol stop", "stop patrol", "순찰 중지", "순찰 정지"}:
        return MissionCommand(type="patrol_stop", raw=message)
    if lowered in {"explore stop", "탐색 중지", "자율탐색 중지", "탐험 중지"}:
        return MissionCommand(type="explore_stop", raw=message)
    # Bare/ambiguous stop word with no 순찰/탐색 qualifier - let patrol_node
    # resolve it against whichever mission is actually running right now
    # (see MissionCommand type "stop" handling in patrol_node.on_mission_command).
    if lowered in {"stop", "halt", "정지", "멈춰", "/stop"}:
        return MissionCommand(type="stop", raw=message)
    if lowered in {"go home", "return home", "home", "복귀", "기지로", "충전소로"}:
        return MissionCommand(type="go_home", raw=message)
    if lowered in {"follow person", "person follow", "follow me", "사람 따라가", "따라와", "나를 따라와"}:
        return MissionCommand(type="follow_person", target="person", raw=message)
    if lowered in {"remember start", "위치 기억 시작", "기억 시작", "여기부터 기억해", "여기부터 기억"}:
        return MissionCommand(type="remember_start", raw=message)
    if lowered in {"explore", "explore start", "자율탐색", "자율 탐색", "탐색 시작", "자율탐색 시작", "탐험 시작"}:
        return MissionCommand(type="explore_start", raw=message)
    if ("천천히" in lowered) and ("앞으로" in lowered or "전진" in lowered):
        return MissionCommand(type="move_forward", target="slow", raw=message)
    if ("천천히" in lowered) and ("뒤로" in lowered or "후진" in lowered):
        return MissionCommand(type="move_backward", target="slow", raw=message)
    save_match = SAVE_LOCATION_PATTERN.search(raw)
    if save_match and ("저장" in lowered or "기억" in lowered) and "시작" not in lowered:
        return MissionCommand(type="remember_save", target=save_match.group(1), raw=message)
    if lowered in {"위치 목록", "저장된 위치", "위치 목록 보여줘", "저장된 위치 목록", "location list", "locations"}:
        return MissionCommand(type="location_list", raw=message)
    if "삭제" in lowered and ("지도" in lowered or "탐색 기록" in lowered or "탐색기록" in lowered):
        return MissionCommand(type="explore_map_reset", raw=message)
    if "초기화" in lowered and ("지도" in lowered or "탐색" in lowered):
        return MissionCommand(type="explore_map_reset", raw=message)
    if "삭제" in lowered and ("위치" in lowered or "라벨" in lowered or "기억" in lowered):
        # Strip the known keywords/particles to isolate the label name, same
        # style as the "점검" target-extraction below.
        target = raw
        for kw in ("위치", "라벨", "기억", "삭제", "해줘", "해줘요", "해", "를", "을", "에서", "좀"):
            target = target.replace(kw, "")
        return MissionCommand(type="location_delete", target=target.strip(), raw=message)
    if lowered in {"forward", "go forward", "move forward", "전진", "전진해", "앞으로", "앞으로 가", "직진", "직진해"}:
        return MissionCommand(type="move_forward", raw=message)
    if lowered in {"backward", "go back", "move backward", "후진", "후진해", "뒤로", "뒤로 가"}:
        return MissionCommand(type="move_backward", raw=message)
    if lowered in {"turn left", "좌회전", "좌회전해", "왼쪽으로", "왼쪽으로 돌아"}:
        return MissionCommand(type="turn_left", raw=message)
    if lowered in {"turn right", "우회전", "우회전해", "오른쪽으로", "오른쪽으로 돌아"}:
        return MissionCommand(type="turn_right", raw=message)
    if lowered in {"turn", "회전", "회전해", "제자리 회전"}:
        return MissionCommand(type="turn_right", raw=message)
    if lowered in {"speed up", "faster", "빠르게", "속도 높여", "더 빠르게", "속도를 높여"}:
        return MissionCommand(type="set_speed", target="up", raw=message)
    if lowered in {"slow down", "slower", "천천히", "속도 줄여", "더 천천히", "속도를 줄여"}:
        return MissionCommand(type="set_speed", target="down", raw=message)
    if lowered in {"analyze", "analyze image", "분석", "영상 분석", "영상분석", "카메라 분석", "지금 상황", "지금 뭐가 보여", "뭐가 보여"}:
        return MissionCommand(type="analyze", raw=message)
    if lowered.startswith("inspect "):
        return MissionCommand(type="inspect", target=raw.split(" ", 1)[1], text=raw, raw=message)
    if lowered.startswith("점검 "):
        return MissionCommand(type="inspect", target=raw.split(" ", 1)[1], text=raw, raw=message)
    if "순찰" in lowered and ("시작" in lowered or "출발" in lowered):
        return MissionCommand(type="patrol_start", raw=message)
    # "계속" (keep going/continue) overrides a stop-word match below - without
    # this, "왜 멈춰? 계속 탐색해" (why did you stop? keep exploring) matched
    # the stop-word branch purely because "멈춰" is in there too, and did the
    # opposite of what the admin actually asked for (confirmed on real
    # hardware). A stop word next to "계속" almost always means "don't stop"/
    # "keep going", not "stop".
    if "순찰" in lowered and "계속" in lowered:
        return MissionCommand(type="patrol_start", raw=message)
    if "순찰" in lowered and ("중지" in lowered or "정지" in lowered or "멈" in lowered):
        return MissionCommand(type="patrol_stop", raw=message)
    if ("탐색" in lowered or "탐험" in lowered) and "계속" in lowered:
        return MissionCommand(type="explore_start", raw=message)
    if ("탐색" in lowered or "탐험" in lowered) and ("중지" in lowered or "정지" in lowered or "멈" in lowered):
        return MissionCommand(type="explore_stop", raw=message)
    if "탐색" in lowered or "탐험" in lowered:
        return MissionCommand(type="explore_start", raw=message)
    if "복귀" in lowered or "충전소" in lowered:
        return MissionCommand(type="go_home", raw=message)
    if "따라" in lowered and ("사람" in lowered or "나" in lowered or "대상" in lowered):
        return MissionCommand(type="follow_person", target=raw, raw=message)
    if "점검" in lowered:
        target = raw.split("점검", 1)[0].strip()
        target = target.replace("를", "").replace("을", "").strip()
        return MissionCommand(type="inspect", target=target or "requested target", text=raw, raw=message)
    # Short-message only: "보고" especially is a generic verb ending ("...확인해서
    # 보고와") that shows up in plenty of task-specific sentences (e.g. "정문앞에
    # 택배가 있는지 보고와"). Those need to reach the LLM as an inspect-style
    # request, not get force-classified as a plain status query here.
    if len(raw) <= 10 and ("상태" in lowered or "보고" in lowered):
        return MissionCommand(type="status", raw=message)
    if "앞으로" in lowered or "전진" in lowered or "직진" in lowered:
        return MissionCommand(type="move_forward", raw=message)
    if "뒤로" in lowered or "후진" in lowered:
        return MissionCommand(type="move_backward", raw=message)
    if "좌회전" in lowered or ("왼쪽" in lowered and ("회전" in lowered or "돌" in lowered)):
        return MissionCommand(type="turn_left", raw=message)
    if "우회전" in lowered or ("오른쪽" in lowered and ("회전" in lowered or "돌" in lowered)):
        return MissionCommand(type="turn_right", raw=message)
    # "돌아" alone is ambiguous in Korean - it's the stem of both 돌다/돌리다
    # ("turn") and 돌아오다/돌아가다 ("come back"/"return"), and confirmed on
    # real hardware that "작은방으로 돌아와" (come back to 작은방) matched
    # here purely because it contains "돌아", spinning the robot in place
    # continuously instead of doing anything resembling "return" - the
    # opposite of what was asked. Exclude the return-verb conjugations so
    # those fall through to the LLM router instead of a false-positive turn.
    if "회전" in lowered or ("돌아" in lowered and not any(
        k in lowered for k in ("돌아와", "돌아옴", "돌아왔", "돌아갈", "돌아가", "복귀")
    )):
        return MissionCommand(type="turn_right", raw=message)
    if "빠르게" in lowered or ("속도" in lowered and ("높여" in lowered or "올려" in lowered)):
        return MissionCommand(type="set_speed", target="up", raw=message)
    if "천천히" in lowered or ("속도" in lowered and ("줄여" in lowered or "낮춰" in lowered)):
        return MissionCommand(type="set_speed", target="down", raw=message)
    if "분석" in lowered or "뭐가 보여" in lowered:
        return MissionCommand(type="analyze", raw=message)

    return MissionCommand(type="say", text=raw, raw=message)


def normalize_command_type(command_type: str) -> str:
    lowered = command_type.strip().lower().replace("-", "_").replace(" ", "_")
    aliases = {
        "patrol": "patrol_start",
        "start": "patrol_start",
        "start_patrol": "patrol_start",
        "stop": "patrol_stop",
        "stop_patrol": "patrol_stop",
        "return_home": "go_home",
        "home": "go_home",
        "follow": "follow_person",
        "follow_me": "follow_person",
        "follow_person": "follow_person",
        "person_follow": "follow_person",
        "report": "status",
        "state": "status",
        "forward": "move_forward",
        "move_forward": "move_forward",
        "backward": "move_backward",
        "move_backward": "move_backward",
        "back": "move_backward",
        "left": "turn_left",
        "turn_left": "turn_left",
        "right": "turn_right",
        "turn_right": "turn_right",
        "turn": "turn_right",
        "speed_up": "set_speed",
        "speed_down": "set_speed",
        "set_speed": "set_speed",
        "analyze": "analyze",
        "analyse": "analyze",
        "remember_start": "remember_start",
        "remember_save": "remember_save",
        "save_location": "remember_save",
        "explore": "explore_start",
        "explore_start": "explore_start",
        "start_explore": "explore_start",
        "explore_stop": "explore_stop",
        "stop_explore": "explore_stop",
    }
    return aliases.get(lowered, lowered)


def is_direct_robot_command(command: MissionCommand) -> bool:
    return command.type in {
        "status", "patrol_start", "patrol_stop", "stop", "go_home", "inspect", "follow_person",
        "move_forward", "move_backward", "turn_left", "turn_right", "set_speed", "analyze",
        "remember_start", "remember_save", "explore_start", "explore_stop",
        "location_list", "location_delete", "explore_map_reset",
    }


def command_to_json(command: MissionCommand) -> str:
    return json.dumps(
        {
            "type": command.type,
            "target": command.target,
            "text": command.text,
            "raw": command.raw,
        },
        ensure_ascii=False,
    )
