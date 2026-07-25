import json
from pathlib import Path
from typing import Dict, List, Optional


class RoomGraph:
    """Topological (not geometric) map built by patrol_node's room-scan
    explorer: each room is a node with a list of doorways (detected via VLM
    during a 45-degree rotation sweep) and a pointer to its parent room (the
    doorway it was entered through). This is deliberately NOT a floor plan
    with real dimensions/coordinates - this robot has no occupancy-grid SLAM
    (see docs/navigation_roadmap.md for why rtabmap/Nav2 were dropped this
    session), so "which room connects to which through which doorway" is the
    most this sensor set can honestly support. Persisted to JSON so the map
    survives a restart, the same way locations.json does.
    """

    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.rooms: Dict[str, dict] = {}
        self.next_room_id = 1
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            raw = {}
        self.rooms = raw.get("rooms", {})
        self.next_room_id = int(raw.get("next_room_id", 1))

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(
            json.dumps({"rooms": self.rooms, "next_room_id": self.next_room_id}, ensure_ascii=False, indent=2),
            encoding="utf-8",
        )

    def start_room(self, parent_room_id: Optional[str], entered_via_step: Optional[int]) -> str:
        room_id = str(self.next_room_id)
        self.next_room_id += 1
        self.rooms[room_id] = {
            "label": None,
            "doorways": [],  # [{"step": int, "description": str, "tried": bool}]
            "ceiling_description": "",
            "parent_room_id": parent_room_id,
            "entered_via_step": entered_via_step,
        }
        self.save()
        return room_id

    def add_doorway(self, room_id: str, step: int, description: str) -> None:
        self.rooms[room_id]["doorways"].append({"step": step, "description": description, "tried": False})
        self.save()

    def set_label(self, room_id: str, label: str) -> None:
        self.rooms[room_id]["label"] = label
        self.save()

    def set_ceiling_description(self, room_id: str, description: str) -> None:
        self.rooms[room_id]["ceiling_description"] = description
        self.save()

    def next_untried_doorway(self, room_id: str) -> Optional[dict]:
        for doorway in self.rooms.get(room_id, {}).get("doorways", []):
            if not doorway["tried"]:
                return doorway
        return None

    def mark_doorway_tried(self, room_id: str, step: int) -> None:
        for doorway in self.rooms.get(room_id, {}).get("doorways", []):
            if doorway["step"] == step:
                doorway["tried"] = True
        self.save()

    def parent_of(self, room_id: str) -> Optional[str]:
        return self.rooms.get(room_id, {}).get("parent_room_id")

    def entered_via_step_of(self, room_id: str) -> Optional[int]:
        return self.rooms.get(room_id, {}).get("entered_via_step")

    def label_of(self, room_id: str) -> Optional[str]:
        return self.rooms.get(room_id, {}).get("label")

    def summary_lines(self) -> List[str]:
        lines = []
        for room_id, room in self.rooms.items():
            label = room.get("label") or f"(이름없음#{room_id})"
            doorway_count = len(room.get("doorways", []))
            lines.append(f"{label}: 출입구 {doorway_count}개, 천장 {room.get('ceiling_description') or '미기록'}")
        return lines
