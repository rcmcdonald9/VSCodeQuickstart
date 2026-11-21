"""Brain tests with a fake Anthropic client — no network, no API key needed."""
from types import SimpleNamespace as NS

from robodog.brain import TOOLS, ClaudeBrain, LocalBrain, RobotTools
from robodog.config import load_config, sim_config


class FakeTools:
    def __init__(self):
        self.calls = []

    def execute(self, name, args):
        self.calls.append((name, args))
        return f"{name} ok", None


def test_tool_schemas_are_strict():
    for t in TOOLS:
        assert t["strict"] is True
        s = t["input_schema"]
        assert s["additionalProperties"] is False
        assert set(s["required"]) <= set(s["properties"])


def test_local_brain():
    tools = FakeTools()
    assert LocalBrain(tools).handle("sit") == "Okay!"
    assert tools.calls == [("set_pose", {"pose": "sit"})]


def test_claude_brain_tool_loop(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test")
    cfg = sim_config(load_config())
    tools = FakeTools()
    brain = ClaudeBrain(cfg, tools)
    responses = iter([
        NS(stop_reason="tool_use", content=[NS(type="tool_use", id="t1", name="find_object", input={"label": "cup"})]),
        NS(stop_reason="end_turn", content=[NS(type="text", text="Found your cup!")]),
    ])
    sent = []

    def fake_create(**kw):
        sent.append({**kw, "messages": list(kw["messages"])})
        return next(responses)

    monkeypatch.setattr(brain.client.beta.messages, "create", fake_create)
    assert brain.handle("where is my cup") == "Found your cup!"
    assert tools.calls == [("find_object", {"label": "cup"})]
    assert sent[0]["model"] == "claude-opus-5-5"
    assert sent[0]["fallbacks"] == "default"
    # second request carries the tool result
    last = sent[1]["messages"][-1]
    assert last["role"] == "user" and last["content"][0]["type"] == "tool_result"


def test_stop_bypasses_network(monkeypatch):
    monkeypatch.setenv("ANTHROPIC_API_KEY", "test")
    tools = FakeTools()
    brain = ClaudeBrain(sim_config(load_config()), tools)
    monkeypatch.setattr(brain.client.beta.messages, "create", lambda **kw: (_ for _ in ()).throw(AssertionError))
    brain.handle("stop")
    assert tools.calls == [("stop", {})]


def test_robot_tools_in_sim():
    from robodog.motion import MotionController
    from robodog.navigation import Navigator
    from robodog.sensors import NullRangeSensor
    from robodog.servos import MockServoDriver
    from robodog.speech import Speaker
    from robodog.vision import Vision

    cfg = sim_config(load_config())
    mc = MotionController(cfg, MockServoDriver(cfg["servos"]))
    mc.start()
    try:
        vis, tof = Vision(cfg), NullRangeSensor()
        rt = RobotTools(mc, Navigator(mc, vis, tof), vis, tof, Speaker(cfg))
        text, _ = rt.execute("set_pose", {"pose": "stand"})
        assert "stand" in text
        text, _ = rt.execute("walk", {"distance_cm": 10, "direction": "forward"})
        assert text.startswith("walked about")
        text, _ = rt.execute("status", {})
        assert '"pose": "stand"' in text
    finally:
        mc._running = False
