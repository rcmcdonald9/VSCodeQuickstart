"""Run only the checks that cover what changed, in parallel (sprint mode, RND_PLAN §1).

python -m rnd.check                 # files changed vs origin/<branch> (or HEAD~1)
python -m rnd.check --since <rev>   # files changed since a commit
python -m rnd.check rnd/parts/arm.py sw/safety.py   # explicit files

Light checks run up to JOBS at a time; heavy ones (FEA, Monte Carlo, full twin) run one at a time after them.
Each check's output goes to rnd/out/check_logs/<name>.log; the summary lists pass/fail and wall time.
"""
from __future__ import annotations

import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from fnmatch import fnmatch
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent
LOGS = ROOT / "rnd" / "out" / "check_logs"
JOBS = 3
HEAVY = {"rnd.stiffness", "rnd.kx_mc", "rnd.mc", "rnd.threshold_mc", "rnd.run_all"}
CAD = ["rnd.assembly_seq", "rnd.spacing", "rnd.integration", "rnd.assembly_check", "rnd.swept"]
MAP = [  # (glob, checks)
    ("rnd/parts/roll.py", CAD + ["rnd.dexterity", "rnd.harness", "rnd.joints"]),
    ("rnd/parts/arm.py", CAD + ["rnd.stiffness"]),
    ("rnd/parts/shoulder.py", CAD + ["rnd.balancer", "rnd.joints"]),
    ("rnd/joints.py", ["rnd.joints"]),
    ("rnd/parts/kx.py", CAD + ["rnd.kx_check"]),
    ("rnd/parts/covers.py", ["rnd.pinch"]),
    ("rnd/parts/*.py", CAD),
    ("rnd/pinch.py", ["rnd.pinch"]),
    ("rnd/params.py", ["rnd.run_all --quick", "rnd.basket", "rnd.test_params"]),
    ("rnd/test_params.py", ["rnd.test_params"]),
    ("rnd/electrical.py", ["rnd.electrical"]),
    ("rnd/harness.py", ["rnd.harness"]),
    ("rnd/balancer.py", ["rnd.balancer"]),
    ("rnd/bom.py", ["rnd.bom"]),
    ("rnd/basket.py", ["rnd.basket"]),
    ("rnd/payload_study.py", ["rnd.payload_study"]),
    ("rnd/specs/*", ["rnd.run_all --quick"]),
    ("sw/*", ["sw.test_s1", "sw.test_s2"]),
]


def changed(since=None):
    if since is None:
        br = subprocess.run(["git", "rev-parse", "--abbrev-ref", "HEAD"], cwd=ROOT, capture_output=True, text=True).stdout.strip()
        r = subprocess.run(["git", "rev-parse", "--verify", f"origin/{br}"], cwd=ROOT, capture_output=True, text=True)
        since = f"origin/{br}" if r.returncode == 0 else "HEAD~1"
    out = subprocess.run(["git", "diff", "--name-only", since], cwd=ROOT, capture_output=True, text=True).stdout.split()
    out += subprocess.run(["git", "ls-files", "--others", "--exclude-standard"], cwd=ROOT, capture_output=True, text=True).stdout.split()
    return sorted(set(out))


def checks_for(files):
    want = []
    for f in files:
        for g, cs in MAP:
            if fnmatch(f, g):
                want += [c for c in cs if c not in want]
                break
    if "rnd.run_all" in want:                       # the full run covers the quick one
        want = [c for c in want if c != "rnd.run_all --quick"]
    return want


def run_one(c):
    LOGS.mkdir(parents=True, exist_ok=True)
    name = c.replace(" ", "_").replace("--", "")
    t0 = time.time()
    with open(LOGS / f"{name}.log", "w") as fh:
        r = subprocess.run([sys.executable, "-m", *c.split()], cwd=ROOT, stdout=fh, stderr=subprocess.STDOUT,
                           env={**__import__("os").environ, "MUJOCO_GL": "egl"})
    return c, r.returncode, time.time() - t0


def main(argv):
    since = argv[argv.index("--since") + 1] if "--since" in argv else None
    files = [a for a in argv if not a.startswith("--") and a != since] or changed(since)
    cs = checks_for(files)
    if not cs:
        print("no checks cover:", files)
        return 0
    light, heavy = [c for c in cs if c.split()[0] not in HEAVY], [c for c in cs if c.split()[0] in HEAVY]
    print(f"{len(files)} files → {len(light)} light (×{JOBS} parallel) + {len(heavy)} heavy checks")
    t0 = time.time()
    with ThreadPoolExecutor(JOBS) as ex:
        res = list(ex.map(run_one, light))
    res += [run_one(c) for c in heavy]
    for c, rc, dt in res:
        print(f"{'✅' if rc == 0 else '❌'} {c:28s} {dt:6.0f} s   rnd/out/check_logs/{c.replace(' ', '_').replace('--', '')}.log")
    print(f"total {time.time() - t0:.0f} s")
    return int(any(rc for _, rc, _ in res))


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
