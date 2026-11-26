"""Pip v4 hype video, step 3: MuJoCo twin footage at HD (read-only use of rnd/sim.py and sw/bench_env.py).

MUJOCO_GL=egl python -m rnd.video.twin_footage OUT_DIR
  OUT_DIR/bench_ext/*.jpg   bench arm scripted pick-and-place, cinematic free camera, 1920x1080, ~30 fps of sim time
  OUT_DIR/bench_wrist/*.jpg the same rollout from the wrist camera, 640x640
  OUT_DIR/fetch/*.jpg       whole-robot fetch task (rnd/sim.py run), 1920x1080, one frame per 0.1 s of sim time
Neither module is modified: their `mujoco` reference is swapped for a thin proxy that renders bigger / more often.
"""
from __future__ import annotations

import os
import shutil
import sys
from pathlib import Path

os.environ.setdefault("MUJOCO_GL", "egl")
import mujoco  # noqa: E402
import numpy as np  # noqa: E402
from PIL import Image  # noqa: E402

W, H = 1920, 1080


class _Proxy:
    """Stands in for the `mujoco` module inside one of the repo modules."""
    def __init__(self, **over):
        self.__dict__.update(over)

    def __getattr__(self, k):
        return getattr(mujoco, k)


def _big_renderer(m, h, w, scale=None):
    m.vis.global_.offwidth, m.vis.global_.offheight = max(W, m.vis.global_.offwidth), max(H, m.vis.global_.offheight)
    m.vis.quality.shadowsize = 4096
    m.vis.quality.offsamples = 8
    return mujoco.Renderer(m, h, w)


def bench(out: Path, seed=3):
    from sw import bench_env as be
    env = be.BenchEnv(render=False)
    ext = _big_renderer(env.m, H, W)
    wr = _big_renderer(env.m, 640, 640)
    cam = mujoco.MjvCamera(); cam.type = mujoco.mjtCamera.mjCAMERA_FREE
    opt = mujoco.MjvOption()
    for sub in ("bench_ext", "bench_wrist"):
        shutil.rmtree(out / sub, ignore_errors=True); (out / sub).mkdir(parents=True)
    state = dict(n=0, k=0)

    def step(m, d):
        mujoco.mj_step(m, d)
        state["n"] += 1
        if state["n"] % 17:                       # 17 x 2 ms ~ 30 fps of sim time
            return
        k = state["k"]; state["k"] += 1
        t = k / 30.0
        cam.lookat[:] = [0.36, -0.06, be.TABLE_Z + 0.16]
        cam.distance = 1.15 - 0.012 * t                       # slow push-in
        cam.azimuth = -128 + 4.0 * t                          # slow orbit
        cam.elevation = -16
        ext.update_scene(d, camera=cam, scene_option=opt)
        Image.fromarray(ext.render()).save(out / "bench_ext" / f"{k:04d}.jpg", quality=94)
        wr.update_scene(d, camera="wrist")
        Image.fromarray(wr.render()).save(out / "bench_wrist" / f"{k:04d}.jpg", quality=94)

    be.mujoco = _Proxy(mj_step=step)
    try:
        ok, _ = be.rollout_expert(env, seed)
    finally:
        be.mujoco = mujoco
    print("bench rollout seed", seed, "success", ok, "frames", state["k"], flush=True)
    return ok


def fetch(out: Path):
    from rnd import sim

    class R:
        def __init__(self, m, h, w):
            self.r = _big_renderer(m, H, W); self.k = 0

        def update_scene(self, d, camera=None, **kw):
            c = camera
            c.distance, c.elevation = 2.2, -14
            c.azimuth = -75 + 0.35 * self.k                    # slow drift across the run
            c.lookat[2] = 0.75
            self.k += 1
            self.r.update_scene(d, camera=c)

        def render(self):
            return self.r.render()

    sim.mujoco = _Proxy(Renderer=R)
    try:
        res = sim.run(video=True)
    finally:
        sim.mujoco = mujoco
    (out / "fetch").mkdir(parents=True, exist_ok=True)
    for i, (_, f) in enumerate(res["frames"]):
        Image.fromarray(f).save(out / "fetch" / f"{i:04d}.jpg", quality=94)
    print("fetch frames", len(res["frames"]), "ok", res.get("ok"), flush=True)


if __name__ == "__main__":
    o = Path(sys.argv[1])
    for s in (3, 1, 2, 4, 5):
        if bench(o, s):
            break
    fetch(o)
