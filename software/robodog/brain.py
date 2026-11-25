"""The dog's "brain": turns a spoken command into actions.

Two providers share the same tool set:
  * ClaudeBrain — sends the command (and, on request, camera images) to Claude,
    which plans and calls the robot tools below, then replies in a short
    sentence that the dog speaks. Needs internet + an Anthropic API key.
  * LocalBrain  — offline keyword matching (robodog.intents). Always used for
    "stop" so stopping never waits on the network.
"""
from __future__ import annotations

import base64
import json
import logging
import time

from . import intents
from .gait import POSE_NAMES

log = logging.getLogger(__name__)


def _tool(name, description, props=None, required=None):
    return {
        "name": name,
        "description": description,
        "strict": True,
        "input_schema": {
            "type": "object",
            "properties": props or {},
            "required": required or [],
            "additionalProperties": False,
        },
    }


TOOLS = [
    _tool("stop", "Immediately stop moving and cancel the current behaviour."),
    _tool("set_pose", "Change body pose. Use 'stand' before walking.",
          {"pose": {"type": "string", "enum": list(POSE_NAMES)}}, ["pose"]),
    _tool("walk", "Walk a distance (dead-reckoned, roughly accurate). Stops early for obstacles. "
                  "To go left or right, turn first.",
          {"distance_cm": {"type": "number", "description": "1-300"},
           "direction": {"type": "string", "enum": ["forward", "backward"]}},
          ["distance_cm", "direction"]),
    _tool("turn", "Turn in place. Positive degrees = left, negative = right.",
          {"degrees": {"type": "number", "description": "-360 to 360"}}, ["degrees"]),
    _tool("look", "Take a photo with the head camera. Returns the image plus the object-detector's "
                  "labels. Use this to answer questions about the surroundings."),
    _tool("move_head", "Point the head (and camera). pan: + = left, - = right (-60..60). "
                       "tilt: + = down, - = up (-25..25). Use it to look around without walking.",
          {"pan_deg": {"type": "number"}, "tilt_deg": {"type": "number"}}, ["pan_deg", "tilt_deg"]),
    _tool("find_object", "Turn around until an object is in view, then face it. Labels are COCO "
                         "classes (person, dog, cat, chair, couch, bottle, cup, sports ball, ...).",
          {"label": {"type": "string"}}, ["label"]),
    _tool("go_to_object", "Find an object and walk up to it, stopping about 30 cm away.",
          {"label": {"type": "string"}}, ["label"]),
    _tool("follow", "Follow an object (usually 'person') for a while.",
          {"label": {"type": "string"}, "duration_s": {"type": "number", "description": "5-300"}},
          ["label", "duration_s"]),
    _tool("explore", "Wander around avoiding obstacles; reports what it saw.",
          {"duration_s": {"type": "number", "description": "5-120"}}, ["duration_s"]),
    _tool("status", "Battery-free status: pose, current behaviour, distance ahead, odometry."),
    _tool("say", "Say something out loud right now (e.g. a progress update before a long action).",
          {"text": {"type": "string"}}, ["text"]),
]

SYSTEM_PROMPT = """You are {name}, a small four-legged robot dog built from 3D-printed parts, \
a Raspberry Pi 5 and 10 smart servos (8 in the legs, 2 in a neck that pans and tilts your head). You hear your owner through a microphone (speech-to-text, so \
expect small transcription mistakes) and you speak through a small speaker.

How to behave:
- Act through the tools. Chain them when a request needs several steps (for example: \
find_object, then go_to_object). Use `look` when you need to see.
- You are about 25 cm long and walk slowly (about 15 cm per second); you cannot step sideways, \
only forward, backward and turning. Your camera and distance sensor are in your head, which you can \
point with move_head. You have no arms and cannot climb stairs or pick things up.
- Safety first: if asked to do something unsafe for you or others (walking off a table, \
going near stairs), decline briefly.
- Your final reply is spoken aloud: keep it to one or two short, friendly sentences, plain \
words, no lists, no markdown, no emoji. Be a bit playful, like a good dog."""


class RobotTools:
    """Executes tool calls against the real robot. Returns (text, jpeg_bytes | None)."""

    def __init__(self, motion, nav, vision, tof, speaker):
        self.motion, self.nav, self.vision, self.tof, self.speaker = motion, nav, vision, tof, speaker

    def execute(self, name: str, args: dict) -> tuple[str, bytes | None]:
        m, nav = self.motion, self.nav
        if name == "stop":
            nav.stop()
            return "stopped", None
        if name == "set_pose":
            nav.stop()
            m.set_pose(args["pose"])
            time.sleep(1.0)
            return f"now in pose '{args['pose']}'", None
        if name == "walk":
            if m.pose_name != "stand":
                m.set_pose("stand"); time.sleep(1.0)
            cm = max(1.0, min(300.0, float(args["distance_cm"])))
            return nav.run("walk", nav.walk, cm, args["direction"], timeout=60), None
        if name == "turn":
            if m.pose_name != "stand":
                m.set_pose("stand"); time.sleep(1.0)
            deg = max(-360.0, min(360.0, float(args["degrees"])))
            return nav.run("turn", nav.turn, deg, timeout=30), None
        if name == "move_head":
            m.set_head(float(args["pan_deg"]), float(args["tilt_deg"]))
            time.sleep(0.6)
            return f"head at pan {m.head[0]:.0f}°, tilt {m.head[1]:.0f}°", None
        if name == "look":
            dets = self.vision.detections()
            labels = ", ".join(d.describe() for d in dets) or "no known objects detected"
            dist = self.tof.distance_mm()
            text = (f"detector: {labels}. distance ahead: {f'{dist:.0f} mm' if dist else 'unknown'}. "
                    f"head pan {m.head[0]:.0f}°, tilt {m.head[1]:.0f}°")
            return text, self.vision.jpeg()
        if name == "find_object":
            return nav.run("find", nav.find, args["label"], timeout=40), None
        if name == "go_to_object":
            return nav.run("go_to", nav.go_to, args["label"], timeout=70), None
        if name == "follow":
            dur = max(5.0, min(300.0, float(args["duration_s"])))
            return nav.run("follow", nav.follow, args["label"], dur, timeout=dur + 10), None
        if name == "explore":
            dur = max(5.0, min(120.0, float(args["duration_s"])))
            return nav.run("explore", nav.explore, dur, timeout=dur + 10), None
        if name == "status":
            d = self.tof.distance_mm()
            x, y, h = m.odometry
            return json.dumps({"pose": m.pose_name, "behaviour": nav.current, "head_pan_tilt": m.head,
                               "distance_ahead_mm": round(d) if d else None,
                               "odometry_cm": [round(x / 10), round(y / 10)],
                               "heading_deg": round(h * 57.3)}), None
        if name == "say":
            self.speaker.say(args["text"])
            return "said it", None
        raise ValueError(f"unknown tool {name}")


class LocalBrain:
    def __init__(self, tools: RobotTools):
        self.tools = tools

    def handle(self, command: str) -> str:
        parsed = intents.parse(command)
        if not parsed:
            return "Sorry, I don't know that trick yet."
        name, args = parsed
        text, _ = self.tools.execute(name, args)
        if name == "look":
            seen = text.split(". distance")[0].removeprefix("detector: ")
            if seen.startswith("no known objects"):
                return "I don't see anything I recognise."
            return "I see " + ", ".join(part.split(" (")[0] for part in seen.split(", ") if " (" in part) + "."
        return {"stop": "Okay, stopping.", "set_pose": "Okay!"}.get(name, text.capitalize() + ".")


class ClaudeBrain:
    def __init__(self, cfg: dict, tools: RobotTools):
        import anthropic

        b = cfg["brain"]
        self.anthropic = anthropic
        self.client = anthropic.Anthropic()          # reads ANTHROPIC_API_KEY
        self.model = b.get("model", "claude-opus-5-5")
        self.effort = b.get("effort", "low")
        self.max_rounds = int(b.get("max_tool_rounds", 12))
        self.idle_reset = float(b.get("conversation_idle_reset_s", 300))
        self.system = SYSTEM_PROMPT.format(name=cfg["robot"]["name"])
        self.tools = tools
        self.local = LocalBrain(tools)
        self.messages: list = []
        self._last = 0.0

    def _create(self):
        return self.client.beta.messages.create(
            model=self.model,
            max_tokens=4096,
            system=self.system,
            tools=TOOLS,
            messages=self.messages,
            output_config={"effort": self.effort},
            cache_control={"type": "ephemeral"},      # caches system prompt + tools + history
            betas=["server-side-fallback-2026-07-01"],
            fallbacks="default",                      # retry declined requests on a fallback model
        )

    def handle(self, command: str) -> str:
        # Stop never waits on the network.
        parsed = intents.parse(command)
        if parsed and parsed[0] == "stop":
            return self.local.handle(command)

        # Start a fresh conversation after a quiet spell (keeps requests small and fast).
        if time.monotonic() - self._last > self.idle_reset:
            self.messages = []
        self._last = time.monotonic()
        checkpoint = len(self.messages)
        self.messages.append({"role": "user", "content": command})

        try:
            for _ in range(self.max_rounds):
                resp = self._create()
                if resp.stop_reason == "refusal":
                    del self.messages[checkpoint:]
                    return "Sorry, I can't help with that one."
                self.messages.append({"role": "assistant", "content": resp.content})
                if resp.stop_reason == "pause_turn":
                    continue
                if resp.stop_reason != "tool_use":
                    return " ".join(b.text for b in resp.content if b.type == "text").strip() or "Done!"

                results = []
                for block in resp.content:
                    if block.type != "tool_use":
                        continue
                    log.info("tool %s %s", block.name, block.input)
                    try:
                        text, jpeg = self.tools.execute(block.name, dict(block.input))
                        content = [{"type": "text", "text": text}]
                        if jpeg:
                            content.insert(0, {"type": "image", "source": {
                                "type": "base64", "media_type": "image/jpeg",
                                "data": base64.standard_b64encode(jpeg).decode()}})
                        results.append({"type": "tool_result", "tool_use_id": block.id, "content": content})
                    except Exception as e:
                        results.append({"type": "tool_result", "tool_use_id": block.id,
                                        "content": f"error: {e}", "is_error": True})
                self.messages.append({"role": "user", "content": results})
            return "I got a bit lost doing that. Let's try something simpler."
        except self.anthropic.APIConnectionError:
            del self.messages[checkpoint:]
            log.warning("no connection to Claude; using offline commands")
            return self.local.handle(command)
        except self.anthropic.APIStatusError as e:
            del self.messages[checkpoint:]
            log.error("Claude API error %s: %s", e.status_code, e.message)
            return self.local.handle(command)


def make_brain(cfg: dict, tools: RobotTools):
    import os

    if cfg["brain"].get("provider") == "claude":
        if os.environ.get("ANTHROPIC_API_KEY") or os.environ.get("ANTHROPIC_AUTH_TOKEN"):
            return ClaudeBrain(cfg, tools)
        log.warning("brain.provider is 'claude' but ANTHROPIC_API_KEY is not set; using offline commands")
    return LocalBrain(tools)
