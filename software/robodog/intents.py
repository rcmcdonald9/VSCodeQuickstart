"""Offline keyword commands. Used when there's no internet / no API key, and
always used for "stop" so that stopping never waits on the network."""
from __future__ import annotations

import re

NUMBER_WORDS = {"one": 1, "two": 2, "three": 3, "four": 4, "five": 5, "six": 6, "ten": 10,
                "fifteen": 15, "twenty": 20, "thirty": 30, "forty": 40, "fifty": 50, "sixty": 60,
                "ninety": 90, "a hundred": 100, "hundred": 100, "a": 1, "a little": 1}


def _num(text: str, default: float) -> float:
    m = re.search(r"(\d+(?:\.\d+)?)", text)
    if m:
        return float(m.group(1))
    for w, n in sorted(NUMBER_WORDS.items(), key=lambda kv: -len(kv[0])):
        if re.search(rf"\b{w}\b", text):
            return float(n)
    return default


def parse(text: str) -> tuple[str, dict] | None:
    """Returns (tool_name, args) using the same tool names as the Claude brain."""
    t = text.lower().strip().rstrip(".!?")
    if re.fullmatch(r"(please )?(stop|halt|freeze|stay|wait)( now)?( please)?", t):
        return "stop", {}
    if re.search(r"\b(sit)\b", t):
        return "set_pose", {"pose": "sit"}
    if re.search(r"\b(lie|lay) down\b|\bdown\b", t):
        return "set_pose", {"pose": "lie"}
    if re.search(r"\b(stand|get up)\b", t):
        return "set_pose", {"pose": "stand"}
    if re.search(r"\b(bow|play)\b", t):
        return "set_pose", {"pose": "bow"}
    m = re.search(r"\bturn (left|right|around)\b", t)
    if m:
        deg = 180 if m.group(1) == "around" else _num(t, 90)
        return "turn", {"degrees": deg if m.group(1) != "right" else -deg}
    m = re.search(r"\b(?:walk|go|move|come)\s+(forward|backward|back|left|right)\b", t)
    if m:
        d = {"back": "backward"}.get(m.group(1), m.group(1))
        cm = _num(t, 30)
        if re.search(r"\bmet(er|re)s?\b", t):
            cm *= 100
        return "walk", {"distance_cm": cm, "direction": d}
    if re.search(r"\bcome( here)?\b", t):
        return "go_to_object", {"label": "person"}
    m = re.search(r"\bfollow (?:me|the (\w+(?: \w+)?))", t)
    if m:
        return "follow", {"label": m.group(1) or "person", "duration_s": 60}
    m = re.search(r"\b(?:find|look for|where is|where's) (?:the |a |my )?(\w+(?: \w+)?)", t)
    if m:
        return "find_object", {"label": m.group(1)}
    m = re.search(r"\bgo to (?:the |a |my )?(\w+(?: \w+)?)", t)
    if m:
        return "go_to_object", {"label": m.group(1)}
    if re.search(r"\b(explore|wander|look around)\b", t):
        return "explore", {"duration_s": 30}
    if re.search(r"what (do you|can you) see|what's in front", t):
        return "look", {}
    return None
