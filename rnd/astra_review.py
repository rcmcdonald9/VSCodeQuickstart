"""Owner's external review #2b: OpenAI Astra (gpt-6.2-astra) on the M1 bench-arm packet. The key is proxy-injected.
python -m rnd.astra_review [packet]   (default docs/review/packet_m1_arm.md)

Owner rule (2026-11-26): at most ONE Astra run per day, enforced here with a 24 h stamp.
"""
import base64, datetime, json, os, sys, urllib.request
from pathlib import Path

R = Path(__file__).resolve().parent.parent
STAMP = R / "docs" / "review" / "last_astra_run.txt"
MODEL = "gpt-6.2-astra"
now = datetime.datetime.now(datetime.timezone.utc)
if STAMP.exists() and now - datetime.datetime.fromisoformat(STAMP.read_text().strip()) < datetime.timedelta(hours=24):
    raise SystemExit(f"Refusing: last Astra run was {STAMP.read_text().strip()} (owner rule: at most one run per day).")
packet = Path(sys.argv[1]) if len(sys.argv) > 1 else R / "docs/review/packet_m1_arm.md"
prompt = (R / "docs/review/PROMPT_review2.md").read_text()
content = [{"type": "text", "text": prompt + "\n\n" + packet.read_text()}]
img = R / "docs/rnd/img/robot_v4.png"
if img.exists():
    content.append({"type": "image_url", "image_url": {"url": "data:image/png;base64," + base64.b64encode(img.read_bytes()).decode()}})
body = {"model": MODEL, "messages": [{"role": "user", "content": content}], "stream": True, "reasoning_effort": "high",
        "stream_options": {"include_usage": True}}
STAMP.write_text(now.isoformat())                      # stamp before the call: a failed call still counts as the day's run
req = urllib.request.Request("https://api.openai.com/v1/chat/completions", data=json.dumps(body).encode(),
                             headers={"Content-Type": "application/json", "Authorization": "Bearer proxy-injected"})
msg, usage, finish = [], None, None
with urllib.request.urlopen(req, timeout=3600) as r:
    for line in r:
        line = line.decode().strip()
        if not line.startswith("data:") or line == "data: [DONE]":
            continue
        ev = json.loads(line[5:])
        usage = ev.get("usage") or usage
        for ch in ev.get("choices", []):
            msg.append(ch.get("delta", {}).get("content") or "")
            finish = ch.get("finish_reason") or finish
out = R / "docs" / "review" / f"astra_review_{now.date()}.md"
out.write_text(f"# External review #2b: {MODEL} ({now.date()})\n\nPacket: {packet.name}; prompt: PROMPT_review2.md. Raw output below.\n\n" + "".join(msg))
print("finish:", finish, "usage:", usage, "chars:", len("".join(msg)), "→", out)
