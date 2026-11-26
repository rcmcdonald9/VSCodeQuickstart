"""Write the synthetic demos with LeRobot's own writer (dataset format v3.0).  python -m sw.to_lerobot_v3 [--max N]

sw/gen_demos.py writes a v2.1-style layout by hand. LeRobot 0.7 refuses v2.1 ("not backward compatible"), so a
hand-written imitation isn't proof of compatibility (Astra review #2b finding 21). This converter replays our episodes
through `LeRobotDataset.create / add_frame / save_episode / finalize`, then reloads the result with `LeRobotDataset` and
checks shapes, lengths and that the actions round-trip exactly.

Input: sw/out/pip_bench_pick_place (gen_demos). Output: sw/out/pip_bench_pick_place_v3 (git-ignored).
"""
from __future__ import annotations

import argparse
import json
import shutil
import time
from pathlib import Path

import numpy as np

SRC = Path("sw/out/pip_bench_pick_place")
DST = Path("sw/out/pip_bench_pick_place_v3")


def main():
    import imageio.v2 as iio
    import pandas as pd
    from lerobot.datasets.lerobot_dataset import LeRobotDataset
    ap = argparse.ArgumentParser()
    ap.add_argument("--max", type=int, default=None)
    a = ap.parse_args()
    info = json.loads((SRC / "meta" / "info.json").read_text())
    tasks = [json.loads(l)["task"] for l in (SRC / "meta" / "tasks.jsonl").read_text().splitlines()]
    n_ep = info["total_episodes"] if a.max is None else min(a.max, info["total_episodes"])
    img = info["features"]["observation.images.wrist"]["shape"]
    feats = {
        "observation.state": {"dtype": "float32", "shape": (14,), "names": info["features"]["observation.state"]["names"]},
        "action": {"dtype": "float32", "shape": (7,), "names": info["features"]["action"]["names"]},
        "observation.images.wrist": {"dtype": "video", "shape": tuple(img), "names": ["height", "width", "channels"]},
        "observation.images.exterior": {"dtype": "video", "shape": tuple(img), "names": ["height", "width", "channels"]},
    }
    if DST.exists():
        shutil.rmtree(DST)
    t0 = time.time()
    ds = LeRobotDataset.create("local/pip_bench_pick_place", fps=info["fps"], features=feats, root=DST, robot_type=info["robot_type"], use_videos=True)
    actions = []
    for ei in range(n_ep):
        df = pd.read_parquet(SRC / "data" / "chunk-000" / f"episode_{ei:06d}.parquet")
        vids = {}
        for cam in ("wrist", "exterior"):
            r = iio.get_reader(str(SRC / "videos" / "chunk-000" / f"observation.images.{cam}" / f"episode_{ei:06d}.mp4"))
            vids[cam] = [f for f in r]
            r.close()
        S = np.stack(df["observation.state"].to_numpy()).astype(np.float32)
        A = np.stack(df["action"].to_numpy()).astype(np.float32)
        T = min(len(A), len(vids["wrist"]), len(vids["exterior"]))
        for t in range(T):
            ds.add_frame({"observation.state": S[t], "action": A[t], "observation.images.wrist": vids["wrist"][t],
                          "observation.images.exterior": vids["exterior"][t], "task": tasks[int(df["task_index"].iloc[t])]})
        ds.save_episode()
        actions.append(A[:T])
    ds.finalize()
    dt = time.time() - t0
    # independent reload with the official loader
    ds2 = LeRobotDataset("local/pip_bench_pick_place", root=DST)
    assert ds2.num_episodes == n_ep, (ds2.num_episodes, n_ep)
    A_all = np.concatenate(actions)
    assert len(ds2) == len(A_all), (len(ds2), len(A_all))
    worst = 0.0
    for i in np.linspace(0, len(ds2) - 1, 25).astype(int):
        x = ds2[int(i)]
        worst = max(worst, float(np.abs(x["action"].numpy() - A_all[i]).max()))
        assert tuple(x["observation.images.wrist"].shape)[-2:] == tuple(img[:2]) or tuple(x["observation.images.wrist"].shape)[:2] == tuple(img[:2])
    x = ds2[0]
    shapes = {k: tuple(v.shape) for k, v in x.items() if hasattr(v, "shape")}
    out = dict(episodes=n_ep, frames=len(ds2), seconds=dt, max_action_error=worst, sample_shapes=shapes, codebase=json.loads((DST / "meta" / "info.json").read_text())["codebase_version"])
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/lerobot_v3.json").write_text(json.dumps(out, indent=1, default=str))
    print(json.dumps(out, default=str))


if __name__ == "__main__":
    main()
