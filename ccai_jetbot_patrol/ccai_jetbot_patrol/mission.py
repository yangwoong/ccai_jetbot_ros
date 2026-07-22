import json
from typing import Any, Dict


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
    if lowered in {"patrol stop", "stop patrol", "stop", "halt", "순찰 중지", "순찰 정지", "정지", "멈춰", "/stop"}:
        return MissionCommand(type="patrol_stop", raw=message)
    if lowered in {"go home", "return home", "home", "복귀", "기지로", "충전소로"}:
        return MissionCommand(type="go_home", raw=message)
    if lowered in {"follow person", "person follow", "follow me", "사람 따라가", "따라와", "나를 따라와"}:
        return MissionCommand(type="follow_person", target="person", raw=message)
    if lowered.startswith("inspect "):
        return MissionCommand(type="inspect", target=raw.split(" ", 1)[1], raw=message)
    if lowered.startswith("점검 "):
        return MissionCommand(type="inspect", target=raw.split(" ", 1)[1], raw=message)
    if "순찰" in lowered and ("시작" in lowered or "출발" in lowered):
        return MissionCommand(type="patrol_start", raw=message)
    if "순찰" in lowered and ("중지" in lowered or "정지" in lowered or "멈" in lowered):
        return MissionCommand(type="patrol_stop", raw=message)
    if "복귀" in lowered or "충전소" in lowered:
        return MissionCommand(type="go_home", raw=message)
    if "따라" in lowered and ("사람" in lowered or "나" in lowered or "대상" in lowered):
        return MissionCommand(type="follow_person", target=raw, raw=message)
    if "점검" in lowered:
        target = raw.split("점검", 1)[0].strip()
        target = target.replace("를", "").replace("을", "").strip()
        return MissionCommand(type="inspect", target=target or "requested target", raw=message)
    if "상태" in lowered or "보고" in lowered:
        return MissionCommand(type="status", raw=message)

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
    }
    return aliases.get(lowered, lowered)


def is_direct_robot_command(command: MissionCommand) -> bool:
    return command.type in {"status", "patrol_start", "patrol_stop", "go_home", "inspect", "follow_person"}


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
