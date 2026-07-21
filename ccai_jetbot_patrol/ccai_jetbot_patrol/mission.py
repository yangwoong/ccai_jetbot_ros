import json
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class MissionCommand:
    type: str
    target: str = ""
    text: str = ""
    raw: str = ""


def parse_mission_command(message: str) -> MissionCommand:
    raw = message.strip()
    if not raw:
        return MissionCommand(type="status", raw=message)

    if raw.startswith("{"):
        try:
            payload: dict[str, Any] = json.loads(raw)
            return MissionCommand(
                type=str(payload.get("type", "status")),
                target=str(payload.get("target", "")),
                text=str(payload.get("text", "")),
                raw=message,
            )
        except json.JSONDecodeError:
            return MissionCommand(type="say", text=raw, raw=message)

    lowered = raw.lower()
    if lowered in {"status", "state", "report"}:
        return MissionCommand(type="status", raw=message)
    if lowered in {"patrol start", "start patrol", "순찰 시작"}:
        return MissionCommand(type="patrol_start", raw=message)
    if lowered in {"patrol stop", "stop patrol", "순찰 중지", "순찰 정지"}:
        return MissionCommand(type="patrol_stop", raw=message)
    if lowered in {"go home", "return home", "복귀"}:
        return MissionCommand(type="go_home", raw=message)
    if lowered.startswith("inspect "):
        return MissionCommand(type="inspect", target=raw.split(" ", 1)[1], raw=message)

    return MissionCommand(type="say", text=raw, raw=message)


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

