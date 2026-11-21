import pytest

from robodog.intents import parse

CASES = {
    "stop": ("stop", {}),
    "sit down": ("set_pose", {"pose": "sit"}),
    "lie down": ("set_pose", {"pose": "lie"}),
    "stand up": ("set_pose", {"pose": "stand"}),
    "turn left": ("turn", {"degrees": 90}),
    "turn right 45 degrees": ("turn", {"degrees": -45}),
    "turn around": ("turn", {"degrees": 180}),
    "walk forward 50 centimeters": ("walk", {"distance_cm": 50, "direction": "forward"}),
    "go back": ("walk", {"distance_cm": 30, "direction": "backward"}),
    "follow me": ("follow", {"label": "person", "duration_s": 60}),
    "find the ball": ("find_object", {"label": "ball"}),
    "go to the chair": ("go_to_object", {"label": "chair"}),
    "come here": ("go_to_object", {"label": "person"}),
    "what do you see": ("look", {}),
    "explore": ("explore", {"duration_s": 30}),
}


@pytest.mark.parametrize("text,expected", CASES.items())
def test_parse(text, expected):
    assert parse(text) == expected


def test_unknown():
    assert parse("recite a poem") is None
