"""S0.3: synthetic demonstrations from the bench twin, written as a LeRobot dataset (v2.1 layout).
python -m sw.gen_demos --episodes 50 --out sw/out/pip_bench_pick_place [--workers 4] [--img 224]

Scripted expert (sw/bench_env.py: top-down pick, carry, place), domain-randomised per episode:
- light colour/intensity, cube colour (bench_env.reset)
- cube pose (x 0.40–0.50, y −0.20…0, yaw ±0.6 rad)
- exterior camera jitter: position ±15 mm, field of view ±3° (this file)
- table colour (this file)
Failed expert episodes are dropped (SW6: expert success ≥ 95 %).

Layout (LeRobot v2.1; loads with `lerobot.datasets.lerobot_dataset.LeRobotDataset(root=...)`):
    meta/info.json, meta/episodes.jsonl, meta/tasks.jsonl, meta/stats.json
    data/chunk-000/episode_000000.parquet
    videos/chunk-000/observation.images.{wrist,exterior}/episode_000000.mp4
Features: observation.state = [6 joint angles, gripper opening, EEF xyz, EEF quaternion wxyz] (14), action = EEF delta
[dx dy dz drx dry drz] (m, rad, on the commanded pose) + gripper [0 open … 1 closed] (7), the DROID/OXE convention
(SOFTWARE_PLAN.md).
"""
from __future__ import annotations

import argparse
import json
import math
import os
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

FPS = 15
TASK = "pick up the cube and place it on the target"


def _episode(args):
    seed, img = args
    os.environ.setdefault("MUJOCO_GL", "egl")
    import mujoco
    from sw.bench_env import BenchEnv, rollout_expert
    env = BenchEnv(render=True, img=img)
    rng = np.random.default_rng(10_000 + seed)
    cam = mujoco.mj_name2id(env.m, mujoco.mjtObj.mjOBJ_CAMERA, "exterior")
    env.m.cam_pos[cam] += rng.uniform(-0.015, 0.015, 3)
    env.m.cam_fovy[cam] += rng.uniform(-3, 3)
    tg = [g for g in range(env.m.ngeom) if mujoco.mj_id2name(env.m, mujoco.mjtObj.mjOBJ_GEOM, g) == "table"]
    for g in tg:
        env.m.geom_rgba[g][:3] = rng.uniform(0.3, 0.8, 3)
    ok, frames = rollout_expert(env, seed, record=True)
    if not ok:
        return seed, None
    state = np.array([np.concatenate([o["qpos"], [o["gripper"]], o["eef_pos"], o["eef_quat"]]) for o, _ in frames], np.float32)
    action = np.array([a for _, a in frames], np.float32)
    wrist = np.stack([o["image_wrist"] for o, _ in frames])
    ext = np.stack([o["image_exterior"] for o, _ in frames])
    return seed, dict(state=state, action=action, wrist=wrist, exterior=ext)


def _write_video(path, frames):
    import imageio.v2 as iio
    path.parent.mkdir(parents=True, exist_ok=True)
    w = iio.get_writer(str(path), fps=FPS, codec="libx264", quality=8, macro_block_size=1, ffmpeg_log_level="error")
    for f in frames:
        w.append_data(f)
    w.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--out", default="sw/out/pip_bench_pick_place")
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--img", type=int, default=224)
    a = ap.parse_args()
    import pandas as pd
    root = Path(a.out)
    (root / "meta").mkdir(parents=True, exist_ok=True)
    t0 = time.time()
    eps, tried, gidx = [], 0, 0
    allS, allA = [], []
    seed = 0
    with Pool(a.workers) as pool:
        while len(eps) < a.episodes:
            batch = [(s, a.img) for s in range(seed, seed + a.workers * 2)]
            seed += len(batch)
            for s, ep in pool.imap(_episode, batch):
                tried += 1
                if ep is None or len(eps) >= a.episodes:
                    continue
                ei = len(eps)
                n = len(ep["action"])
                df = pd.DataFrame({
                    "observation.state": list(ep["state"]), "action": list(ep["action"]),
                    "timestamp": np.arange(n, dtype=np.float32) / FPS, "frame_index": np.arange(n), "episode_index": np.full(n, ei),
                    "index": np.arange(gidx, gidx + n), "task_index": np.zeros(n, int),
                    "next.done": np.r_[np.zeros(n - 1, bool), [True]]})
                p = root / "data" / "chunk-000" / f"episode_{ei:06d}.parquet"
                p.parent.mkdir(parents=True, exist_ok=True)
                df.to_parquet(p)
                for cam in ("wrist", "exterior"):
                    _write_video(root / "videos" / "chunk-000" / f"observation.images.{cam}" / f"episode_{ei:06d}.mp4", ep[cam])
                eps.append(dict(episode_index=ei, tasks=[TASK], length=n, seed=s))
                allS.append(ep["state"]); allA.append(ep["action"])
                gidx += n
    dt = time.time() - t0
    S, A = np.concatenate(allS), np.concatenate(allA)
    st = lambda x: dict(mean=x.mean(0).tolist(), std=x.std(0).tolist(), min=x.min(0).tolist(), max=x.max(0).tolist())
    vid = lambda: {"dtype": "video", "shape": [a.img, a.img, 3], "names": ["height", "width", "channels"],
                   "info": {"video.fps": FPS, "video.codec": "h264", "video.pix_fmt": "yuv420p", "has_audio": False}}
    info = {"codebase_version": "v2.1", "robot_type": "pip_v4_bench_arm", "total_episodes": len(eps), "total_frames": gidx,
            "total_tasks": 1, "total_videos": 2 * len(eps), "total_chunks": 1, "chunks_size": 1000, "fps": FPS,
            "splits": {"train": f"0:{len(eps)}"},
            "data_path": "data/chunk-{episode_chunk:03d}/episode_{episode_index:06d}.parquet",
            "video_path": "videos/chunk-{episode_chunk:03d}/{video_key}/episode_{episode_index:06d}.mp4",
            "features": {
                "observation.state": {"dtype": "float32", "shape": [14], "names": ["pitch", "roll", "yaw", "elbow", "wpitch", "wroll", "gripper",
                                                                                   "eef_x", "eef_y", "eef_z", "eef_qw", "eef_qx", "eef_qy", "eef_qz"]},
                "action": {"dtype": "float32", "shape": [7], "names": ["dx", "dy", "dz", "drx", "dry", "drz", "gripper"]},
                "observation.images.wrist": vid(), "observation.images.exterior": vid(),
                "timestamp": {"dtype": "float32", "shape": [1], "names": None}, "frame_index": {"dtype": "int64", "shape": [1], "names": None},
                "episode_index": {"dtype": "int64", "shape": [1], "names": None}, "index": {"dtype": "int64", "shape": [1], "names": None},
                "task_index": {"dtype": "int64", "shape": [1], "names": None}, "next.done": {"dtype": "bool", "shape": [1], "names": None}},
            "pip": {"generator": "sw/gen_demos.py", "expert_success": len(eps) / tried, "seconds": dt, "demos_per_hour": len(eps) / dt * 3600}}
    (root / "meta" / "info.json").write_text(json.dumps(info, indent=1))
    (root / "meta" / "tasks.jsonl").write_text(json.dumps({"task_index": 0, "task": TASK}) + "\n")
    (root / "meta" / "episodes.jsonl").write_text("".join(json.dumps(e) + "\n" for e in eps))
    (root / "meta" / "stats.json").write_text(json.dumps({"observation.state": st(S), "action": st(A)}))
    print(f"{len(eps)} episodes ({gidx} frames) from {tried} tries: expert {len(eps) / tried:.0%}; {dt:.0f} s → {len(eps) / dt * 3600:.0f} demos/h "
          f"with {a.workers} workers at {a.img} px")


if __name__ == "__main__":
    main()
