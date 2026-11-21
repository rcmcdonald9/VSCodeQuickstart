# Software

Everything is in [`software/`](../software), a Python package called `robodog`.

```
 mic ─► Vosk (offline STT) ─► wake word ─► Brain ─────────────────────────┐
                                           │  ClaudeBrain: Claude picks    │
                                           │  tools; can see camera frames │
                                           │  LocalBrain: offline keywords │
                                           ▼                               ▼
                                   RobotTools (walk, turn, look,    Speaker (Piper TTS)
                                   find/go_to/follow, explore, pose)
                                           │
              Vision thread ◄──────────────┤ Navigator (one behaviour at a time)
   Camera Module 3 + YOLO11n (NCNN)        │
                                           ▼
   VL53L1X ToF ─► obstacle guard ─► MotionController (50 Hz): trot gait ─► IK ─► PCA9685 ─► 12 servos
```

| Module | What it does |
|--------|--------------|
| `kinematics.py` | 3-DOF leg inverse/forward kinematics plus body roll/pitch/yaw. The angle conventions match `cad/assembly.scad`. |
| `gait.py` | Trot gait (diagonal pairs) with cycloid swing, plus named poses: stand, sit, lie, bow, tall, crouch. |
| `motion.py` | 50 Hz control thread. Ramps velocity, blends poses, blocks forward motion when the ToF sees something closer than `stop_distance_mm`, and keeps dead-reckoning odometry. |
| `servos.py` | Maps joint angle to servo angle per joint (`dir`, `center`, `trim`, limits). Drives a PCA9685, or a mock. |
| `vision.py` | Camera thread plus YOLO11n detector (80 everyday objects). Gives the latest detections and a JPEG for the AI. |
| `navigation.py` | Behaviours: walk/turn a distance, explore, find, go to, and follow an object. |
| `speech.py` | Offline wake word and speech-to-text (Vosk), text-to-speech (Piper, falls back to espeak-ng). |
| `brain.py` | Tool definitions; `ClaudeBrain` (tool-use loop with Claude) and `LocalBrain` (keywords). |
| `intents.py` | Offline command parser. Also makes **"stop" always instant**, with no network round trip. |
| `calibrate.py` | Centres the servos, then interactive direction/trim calibration, saved to `calibration.yaml`. |

## Run it

```bash
# Anywhere (laptop included): mock hardware, type commands
cd software && pip install -e .[dev] && python -m robodog --sim --local

# On the Pi, type commands, real hardware
.venv/bin/python -m robodog --text

# On the Pi, full voice: "Hey Pip, go find the ball"
.venv/bin/python -m robodog

# Tests (no hardware needed)
python -m pytest
```

To start at boot, see [`scripts/robodog.service`](../software/scripts/robodog.service).

## The AI brain (Claude)

With an API key, commands go to **Claude** (`claude-opus-5-5` by default, set in `config.yaml`). It gets these robot tools: `walk`, `turn`, `set_pose`, `look` (a camera photo plus the detector's labels), `find_object`, `go_to_object`, `follow`, `explore`, `status`, `say`, `stop`. It chains them on its own. For example, "is my water bottle in the kitchen? go check" turns into explore → look → go_to_object → a spoken answer.

Setup:

1. Create an API key at <https://console.anthropic.com> → API Keys. Add a few dollars of credit.
2. On the Pi, put it in `~/.config/robodog.env` as `ANTHROPIC_API_KEY=sk-ant-...`. The systemd service reads this file. For manual runs, `export` it.

Details:

- `brain.effort: low` keeps replies quick for voice. Raise it to `medium` for trickier multi-step requests.
- **Cost:** A typical command, including a camera image, is a few thousand input tokens. That works out to roughly 1–3 cents per command at Opus 5.5 rates. Prompt caching (on) cuts repeated context. You can set a different model in `config.yaml` if you want it cheaper.
- **No internet or no key:** It automatically falls back to the offline keyword commands: sit, stand, lie down, walk forward 50 cm, turn left/right/around, follow me, find the ball, go to the chair, come here, explore, what do you see, stop.
- **Server-side fallback** is enabled (`fallbacks="default"`). If a request is declined, the API retries it on a fallback model automatically.
- "stop" / "halt" / "freeze" are handled locally and instantly. They work even without the wake word.

## Tuning cheat-sheet (`config.yaml`)

| Symptom | Try |
|---------|-----|
| Feet scuff the floor | `step_height` 30 → 40 |
| Wobbly / falls over | `max_speed_mm_s` ↓, `gait_period_s` 0.45 → 0.55 |
| Body leans to one side standing | Re-trim that leg's roll/pitch in `calibrate` |
| Detector too slow | `vision.fps` ↓, or add the AI HAT+ |
| Wake word misses | Add variants to `robot.wake_words` (e.g. "hey pep"). Vosk's small model is not perfect. |
| Hears itself talking | It already mutes the mic while speaking. Point the speaker away from the mic. |
