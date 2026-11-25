"""Loads config.yaml into plain dicts with a couple of helpers."""
from __future__ import annotations

import copy
import os
from pathlib import Path
from typing import Any

import yaml

DEFAULT_PATH = Path(__file__).resolve().parent.parent / "config.yaml"
# Written by `python -m robodog.calibrate`; overrides servo dir/trim from config.yaml.
CALIBRATION_PATH = DEFAULT_PATH.with_name("calibration.yaml")
LEG_NAMES = ("FR", "FL", "RR", "RL")
JOINTS = ("pitch", "knee")


def load_config(path: str | os.PathLike | None = None) -> dict[str, Any]:
    path = Path(path) if path else DEFAULT_PATH
    with open(path) as f:
        cfg = yaml.safe_load(f)
    cal = path.with_name("calibration.yaml")
    if cal.exists():
        with open(cal) as f:
            _deep_update(cfg, yaml.safe_load(f) or {})
    return cfg


def _deep_update(dst: dict, src: dict):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            _deep_update(dst[k], v)
        else:
            dst[k] = v


def sim_config(cfg: dict[str, Any]) -> dict[str, Any]:
    """Returns a copy of cfg with every hardware backend swapped for a mock."""
    cfg = copy.deepcopy(cfg)
    cfg["servos"]["driver"] = "mock"
    cfg["sensors"]["tof"] = "none"
    cfg["sensors"]["imu"] = "none"
    cfg["vision"]["camera"] = "mock"
    cfg["vision"]["detector"] = "none"
    cfg["speech"]["stt"] = "text"
    cfg["speech"]["tts"] = "print"
    return cfg


def expand(p: str | None) -> str | None:
    return os.path.expanduser(p) if p else p
