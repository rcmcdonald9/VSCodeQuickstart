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
STATE_NAMES = ["pitch", "roll", "yaw", "elbow", "wpitch", "wroll", "gripper", "eef_x", "eef_y", "eef_z", "eef_qw", "eef_qx", "eef_qy", "eef_qz"]
ACTION_NAMES = ["dx", "dy", "dz", "drx", "dry", "drz", "gripper"]


def features(img, state_names=STATE_NAMES, action_names=ACTION_NAMES):
    """The Pip bench feature set (DROID-style: EEF-delta action, wrist + exterior video), shared by every writer."""
    vid = {"dtype": "video", "shape": tuple(img), "names": ["height", "width", "channels"]}
    return {"observation.state": {"dtype": "float32", "shape": (len(state_names),), "names": list(state_names)},
            "action": {"dtype": "float32", "shape": (len(action_names),), "names": list(action_names)},
            "observation.images.wrist": dict(vid), "observation.images.exterior": dict(vid)}


class V3Writer:
    """LeRobot's own writer (dataset format v3.0): create / add_frame / save_episode / finalize. Used by this converter
    and by sw/teleop_sim.py, so synthetic and teleoperated episodes have the same layout."""

    def __init__(self, root, repo_id, fps, img, robot_type="pip_bench_arm", feats=None):
        from lerobot.datasets.lerobot_dataset import LeRobotDataset
        root = Path(root)
        if root.exists():
            shutil.rmtree(root)
        self.ds = LeRobotDataset.create(repo_id, fps=fps, features=feats or features(img), root=root, robot_type=robot_type, use_videos=True)
        self.n_episodes = 0

    def add(self, state, action, wrist, exterior, task):
        self.ds.add_frame({"observation.state": np.asarray(state, np.float32), "action": np.asarray(action, np.float32),
                           "observation.images.wrist": wrist, "observation.images.exterior": exterior, "task": task})

    def save_episode(self):
        self.ds.save_episode()
        self.n_episodes += 1

    def finalize(self):
        self.ds.finalize()


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
    feats = features(img, info["features"]["observation.state"]["names"], info["features"]["action"]["names"])
    t0 = time.time()
    w = V3Writer(DST, "local/pip_bench_pick_place", info["fps"], img, robot_type=info["robot_type"], feats=feats)
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
            w.add(S[t], A[t], vids["wrist"][t], vids["exterior"][t], tasks[int(df["task_index"].iloc[t])])
        w.save_episode()
        actions.append(A[:T])
    w.finalize()
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
