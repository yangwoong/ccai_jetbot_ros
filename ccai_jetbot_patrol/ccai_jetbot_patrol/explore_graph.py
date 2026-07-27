import json
from pathlib import Path
from typing import Dict, List, Optional


class RoomGraph:
    """Topological (not geometric) map built by patrol_node's room-scan
    explorer: each room is a node with a list of per-heading observations
    (features seen during the 45-degree rotation sweep, each tagged movable
    or not) and a pointer to its parent room (the doorway/opening it was
    entered through). This is deliberately NOT a floor plan with real
    dimensions/coordinates - this robot has no occupancy-grid SLAM (see
    docs/navigation_roadmap.md for why rtabmap/Nav2 were dropped this
    session), so "what does each direction from this spot look like, and
    which room connects to which through which opening" is the most this
    sensor set can honestly support. Persisted to JSON so the map survives a
    restart, the same way locations.json does.
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
            # every heading observed during the sweep, whether movable or
            # not - see patrol_node.py's room_scan_process_vlm_step_result.
            # [{"step": int, "description": str, "movable": bool, "tried": bool}]
            "observations": [],
            "ceiling_description": "",
            "parent_room_id": parent_room_id,
            "entered_via_step": entered_via_step,
        }
        self.save()
        return room_id

    def add_observation(self, room_id: str, step: int, description: str, movable: bool, category: str = "floor") -> None:
        # category: "door" (leads to another room/corridor - the primary
        # exploration target), "floor" (just open floor in the same room),
        # or "none" (blocked). See untried_movable's door-first ordering.
        self.rooms[room_id]["observations"].append(
            {"step": step, "description": description, "movable": movable, "tried": False, "category": category}
        )
        self.save()

    def set_label(self, room_id: str, label: str) -> None:
        self.rooms[room_id]["label"] = label
        self.save()

    def set_ceiling_description(self, room_id: str, description: str) -> None:
        self.rooms[room_id]["ceiling_description"] = description
        self.save()

    def untried_movable(self, room_id: str) -> List[dict]:
        # Doors (leads out of this room) sort before plain open floor - per
        # user request, finding an actual exit/doorway is the primary
        # exploration goal, not just wandering the current room's open area.
        obs = [
            o
            for o in self.rooms.get(room_id, {}).get("observations", [])
            if o.get("movable") and not o.get("tried")
        ]
        obs.sort(key=lambda o: 0 if o.get("category") == "door" else 1)
        return obs

    def mark_observation_tried(self, room_id: str, step: int) -> None:
        for obs in self.rooms.get(room_id, {}).get("observations", []):
            if obs["step"] == step:
                obs["tried"] = True
        self.save()

    def parent_of(self, room_id: str) -> Optional[str]:
        return self.rooms.get(room_id, {}).get("parent_room_id")

    def entered_via_step_of(self, room_id: str) -> Optional[int]:
        return self.rooms.get(room_id, {}).get("entered_via_step")

    def label_of(self, room_id: str) -> Optional[str]:
        return self.rooms.get(room_id, {}).get("label")
