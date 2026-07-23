import json
from pathlib import Path
from typing import Dict, List, Optional


class LocationStore:
    """Teach-and-repeat location memory: a named location is the recorded
    sequence of timed move steps used to reach it (no odometry/IMU for real
    coordinates), plus an optional set of ORB visual features captured at that
    spot so arrival can be visually confirmed instead of trusted blindly from
    dead-reckoning alone (which drifts). Persisted to a JSON file so labels
    survive a restart.
    """

    def __init__(self, path: str) -> None:
        self.path = Path(path)
        self.locations: Dict[str, dict] = {}
        self.load()

    def load(self) -> None:
        if not self.path.exists():
            return
        try:
            raw = json.loads(self.path.read_text(encoding="utf-8"))
        except (json.JSONDecodeError, OSError):
            raw = {}
        self.locations = {}
        for label, value in raw.items():
            if isinstance(value, list):
                # Pre-existing file from before visual features were added:
                # a bare step list with no features yet.
                self.locations[label] = {"steps": value, "features": "", "keypoints": 0}
            elif isinstance(value, dict):
                self.locations[label] = {
                    "steps": value.get("steps", []),
                    "features": value.get("features", ""),
                    "keypoints": value.get("keypoints", 0),
                }

    def save(self) -> None:
        self.path.parent.mkdir(parents=True, exist_ok=True)
        self.path.write_text(json.dumps(self.locations, ensure_ascii=False, indent=2), encoding="utf-8")

    def has(self, label: str) -> bool:
        return bool(label) and label in self.locations

    def set(self, label: str, steps: List[dict]) -> None:
        existing = self.locations.get(label, {})
        self.locations[label] = {
            "steps": steps,
            "features": existing.get("features", ""),
            "keypoints": existing.get("keypoints", 0),
        }
        self.save()

    def set_features(self, label: str, features: str, keypoints: int) -> None:
        if label not in self.locations:
            self.locations[label] = {"steps": [], "features": "", "keypoints": 0}
        self.locations[label]["features"] = features
        self.locations[label]["keypoints"] = keypoints
        self.save()

    def get(self, label: str) -> List[dict]:
        return list(self.locations.get(label, {}).get("steps", []))

    def get_features(self, label: str) -> Optional[str]:
        return self.locations.get(label, {}).get("features") or None

    def names(self) -> List[str]:
        return list(self.locations.keys())
