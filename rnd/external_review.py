"""Send the Pip v4 design-review packet to Gemini 4 Neon (OpenAI-compatible endpoint; the key is proxy-injected)."""
import base64, datetime, json, sys, urllib.request
from pathlib import Path

# Owner rule (2026-11-26): at most ONE Gemini 4 Neon run per day. A run costs ≈ $1.67 (207 kB packet, reasoning high).
STAMP = Path(__file__).resolve().parent.parent / "docs" / "review" / "last_neon_run.txt"
_now = datetime.datetime.now(datetime.timezone.utc)
if STAMP.exists():
    _last = datetime.datetime.fromisoformat(STAMP.read_text().strip())
    if _now - _last < datetime.timedelta(hours=24):
        raise SystemExit(f"Refusing: last Gemini 4 Neon run was {_last.isoformat()} (owner rule: at most one run per day).")
R = Path(__file__).resolve().parent.parent
OUT = Path(sys.argv[1])
docs = ["docs/rnd/SCORECARD.md", "docs/RND_PLAN.md", "docs/rnd/BOM_v4.md", "docs/rnd/electrical.md",
        "docs/rnd/fea.md", "docs/rnd/integration.md", "docs/rnd/harness.md", "docs/rnd/mc.md",
        "docs/rnd/thermal.md", "docs/rnd/belts.md", "docs/rnd/spring.md",
        "docs/rnd/assembly.md", "docs/BUILD_v4.md", "docs/COUPONS.md",
        "docs/rnd/joints.md", "docs/rnd/stiffness.md", "docs/rnd/assembly_seq.md", "docs/rnd/roll_inline.md",
        "docs/M1_BENCH_ARM.md", "docs/M1_READINESS.md", "docs/review/fable_review_2026-11-27.md", "docs/sw/sim2real_bench.md"]
code = ["rnd/params.py"]   # docs only + params: the 622 kB and 477 kB packets both got HTTP 502 (the 207 kB one on 2026-11-26 worked)
packet = ["# Pip v4: design-review packet", "",
          "A general-purpose home utility robot: an omni base, a lift on a mast, two 6-DOF arms with parallel grippers, and a head with a 7\" face screen. "
          "It is 3D-printed (PETG / ASA / PETG-CF) on a Bambu P2S (256 mm bed) around off-the-shelf actuators: SteadyWin GIM6010-8 on belt stages, an inline "
          "GIM8108-36 at shoulder roll, optional GIM8108-8 (D20), Damiao DM-J4310, Feetech STS3215. Requirements include a 20 lb two-handed carry and a "
          "laundry basket, consumer pinch safety (R17). Budget limit $5,500. A Fable review the same day is included with its fixes; check them. Everything is generated from rnd/params.py by build123d CAD, "
          "MuJoCo sims, FEA and Monte Carlo checks; numbers marked ESTIMATE are unmeasured.", ""]
for f in docs:
    p = R / f
    if p.exists():
        packet += [f"\n\n=== FILE: {f} ===\n", p.read_text()]
for f in code:
    packet += [f"\n\n=== FILE: {f} ===\n```python\n", (R / f).read_text(), "\n```"]
text = "\n".join(packet)
(OUT.parent / "packet.md").write_text(text)
prompt = """You are a senior mechanical/mechatronics design reviewer (robot arms, mobile manipulators, FDM-printed structures).
Review the design in the packet below critically and independently. Do not restate what it already says; find what is wrong, missing, risky or over-optimistic.

Cover at least:
- the D24 bolted motor cartridges (ear flanges, through-bolts into nuts, Oldham drive) and the D19 one-piece shaft-pulleys
- the inline GIM8108-36 shoulder roll (output adapter, dowels, rear bearing, hard stops) and the C1 tie-rod joints (rnd/joints.py)
- the belt stages and the pulleys
- printed-part strength and stiffness assumptions (FEA method, allowables, layer orientation)
- actuator sizing, thermal behaviour and the electrical system (24 V, BMS, e-stop)
- cable routing (clock springs, yaw torsion)
- stability and tip-over, the base on thresholds and carpet
- docking
- manufacturability on a 256 mm FDM printer and the assembly time
- cost realism
- safety: mechanical pinch (R17, rnd/pinch.py), the e-stop / pre-charge / clamp / watchdog chain, the software supervisor and collision detection
- whether the M1 bench-arm Step 1 / Step 2 purchase plan is sound

For each finding give, as a numbered list:
1. a title
2. severity (critical / major / minor)
3. the evidence: file and section, with the numbers
4. why it matters physically
5. a concrete fix or test
6. your confidence (0–100 %)

Then list the 5 assumptions you think are most likely wrong, and the 5 strongest aspects of the design. Be specific and quantitative."""
content = [{"type": "text", "text": prompt + "\n\n" + text}]
for img in ("docs/rnd/img/robot_v8_covers_concept.png", "docs/rnd/img/roll_inline_vs_belt.png"):
    p = R / img
    if p.exists():
        import io
        from PIL import Image
        im = Image.open(p).convert("RGB"); im.thumbnail((1000, 1000))
        buf = io.BytesIO(); im.save(buf, "JPEG", quality=80)
        content.append({"type": "image_url", "image_url": {"url": "data:image/jpeg;base64," + base64.b64encode(buf.getvalue()).decode()}})
body = {"model": "gemini-4-neon", "messages": [{"role": "user", "content": content}], "reasoning_effort": "high", "max_tokens": 40000, "stream": True}
req = urllib.request.Request("https://generativelanguage.googleapis.com/v1beta/openai/chat/completions", data=json.dumps(body).encode(),
                             headers={"Authorization": "Bearer proxy-injected", "Content-Type": "application/json"})
msg, finish, usage = [], None, None
import urllib.error
try:
    resp_cm = urllib.request.urlopen(req, timeout=1500)
except urllib.error.HTTPError as e:
    print("HTTP", e.code, e.read()[:800]); raise SystemExit(1)
with resp_cm as resp:
    for raw in resp:
        line = raw.decode("utf-8", "ignore").strip()
        if not line.startswith("data:"):
            continue
        data = line[5:].strip()
        if data == "[DONE]":
            break
        ev = json.loads(data)
        if ev.get("usage"):
            usage = ev["usage"]
        for ch in ev.get("choices", []):
            msg.append((ch.get("delta") or {}).get("content") or "")
            finish = ch.get("finish_reason") or finish
OUT.write_text("".join(msg))
STAMP.parent.mkdir(parents=True, exist_ok=True)
STAMP.write_text(_now.isoformat())
print("finish:", finish, "usage:", usage, "chars:", len("".join(msg)), "packet chars:", len(text))
