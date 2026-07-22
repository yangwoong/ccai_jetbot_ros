import json
from pathlib import Path
from typing import Dict, List


class LocationStore:
    """Teach-and-repeat location memory: a named location is just the recorded
    sequence of timed move steps used to reach it, since this robot has no
    odometry/IMU for real coordinates. Persisted to a JSON file so labels
    survive a restart.
    """

    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.locations: Dict[str, List[dict]] = {}
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            self.locations = json.loads(self.path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            self.locations = {}

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.locations, ensure_ascii=False, indent=2), encoding="utf-8")

    def has(self, label: str) -> bool:
        return bool(label) and label in self.locations

    def set(self, label: str, steps: List[dict]) -> None:
        self.locations[label] = steps
        self.save()

    def get(self, label: str) -> List[dict]:
        return list(self.locations.get(label, []))

    def names(self) -> List[str]:
        return list(self.locations.keys())
