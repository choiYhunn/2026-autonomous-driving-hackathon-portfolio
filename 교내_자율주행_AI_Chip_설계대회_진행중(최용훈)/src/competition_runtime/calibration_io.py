"""Calibration file loader shared by production code and calibration notebook."""
from __future__ import annotations

import json
from pathlib import Path
from typing import Any, Dict

BASE_DIR = Path(__file__).resolve().parent
CALIBRATION_FILE = BASE_DIR / "calibration_result.json"


def _deep_merge(base: Dict[str, Any], override: Dict[str, Any]) -> Dict[str, Any]:
    result = dict(base)
    for key, value in override.items():
        if isinstance(value, dict) and isinstance(result.get(key), dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    return result


def load_calibration(defaults: Dict[str, Any]) -> Dict[str, Any]:
    """Load calibration_result.json when present, otherwise return defaults.

    Unknown or malformed files are ignored deliberately so the vehicle can still
    boot with conservative settings.  The caller should print the returned source
    path to make the active configuration obvious.
    """
    if not CALIBRATION_FILE.exists():
        return dict(defaults)
    try:
        with CALIBRATION_FILE.open("r", encoding="utf-8") as handle:
            payload = json.load(handle)
        if not isinstance(payload, dict):
            raise ValueError("calibration root must be an object")
        return _deep_merge(defaults, payload)
    except Exception as exc:
        print(f"[WARN] calibration_result.json ignored: {exc}")
        return dict(defaults)


def save_calibration(payload: Dict[str, Any], path: Path | None = None) -> Path:
    target = CALIBRATION_FILE if path is None else Path(path)
    target.parent.mkdir(parents=True, exist_ok=True)
    tmp = target.with_suffix(target.suffix + ".tmp")
    with tmp.open("w", encoding="utf-8") as handle:
        json.dump(payload, handle, ensure_ascii=False, indent=2)
    tmp.replace(target)
    return target
