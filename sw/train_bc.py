"""S0.4a: training-pipeline dry run, from scratch on CPU.  python -m sw.train_bc --data sw/out/pip_bench_pick_place

What it proves: the whole loop works end to end on our own data format before any GPU time is bought:
LeRobot-format dataset (sw/gen_demos.py) → decode videos → train a policy → run it **closed-loop** in the bench twin →
success rate. The policy is deliberately small (a CNN on both cameras at 64 px + the proprio state → a 4-step action
chunk, behaviour cloning with an L1 loss, ACT-style chunking with temporal ensembling). It is a pipeline test, not the
policy we will deploy: S3.1 benchmarks pretrained checkpoints (π0 / SmolVLA / OpenVLA-OFT) on the same data.

S0.4b (pretrained checkpoint + open-data slice) is blocked here: this environment's network policy refuses huggingface.co.
"""
from __future__ import annotations

import argparse
import json
import os
import time
from pathlib import Path

import numpy as np

RES, CHUNK = 64, 4


def load(root: Path, max_eps=None):
    import imageio.v2 as iio
    import pandas as pd
    info = json.loads((root / "meta" / "info.json").read_text())
    n = info["total_episodes"] if max_eps is None else min(max_eps, info["total_episodes"])
    S, A, I, E = [], [], [], []
    for ei in range(n):
        df = pd.read_parquet(root / "data" / "chunk-000" / f"episode_{ei:06d}.parquet")
        frames = []
        for cam in ("wrist", "exterior"):
            r = iio.get_reader(str(root / "videos" / "chunk-000" / f"observation.images.{cam}" / f"episode_{ei:06d}.mp4"))
            fr = np.stack([f for f in r])
            r.close()
            k = fr.shape[1] // RES
            frames.append(fr[:, ::k, ::k][:, :RES, :RES])          # nearest downsample to 64 px
        img = np.concatenate(frames, axis=-1)                       # (T, 64, 64, 6)
        s = np.stack(df["observation.state"].to_numpy()).astype(np.float32)
        a = np.stack(df["action"].to_numpy()).astype(np.float32)
        T = len(a)
        chunk = np.stack([a[np.minimum(np.arange(t, t + CHUNK), T - 1)] for t in range(T)])   # (T, CHUNK, 7)
        S.append(s); A.append(chunk); I.append(img[:T]); E.append(np.full(T, ei))
    return np.concatenate(S), np.concatenate(A), np.concatenate(I), np.concatenate(E), info


def model():
    import torch.nn as nn

    class Net(nn.Module):
        def __init__(self):
            super().__init__()
            self.cnn = nn.Sequential(nn.Conv2d(6, 32, 5, 2, 2), nn.ReLU(), nn.Conv2d(32, 64, 3, 2, 1), nn.ReLU(),
                                     nn.Conv2d(64, 64, 3, 2, 1), nn.ReLU(), nn.Conv2d(64, 64, 3, 2, 1), nn.ReLU(), nn.Flatten())
            self.head = nn.Sequential(nn.Linear(64 * 4 * 4 + 14, 256), nn.ReLU(), nn.Linear(256, CHUNK * 7))

        def forward(self, img, s):
            import torch
            return self.head(torch.cat([self.cnn(img), s], 1)).view(-1, CHUNK, 7)
    return Net()


def evaluate(net, norm, episodes=20, seed0=5000):
    import torch
    os.environ.setdefault("MUJOCO_GL", "egl")
    from sw.bench_env import BenchEnv, TABLE_Z
    env = BenchEnv(render=True, img=224)
    ok = 0
    for ep in range(episodes):
        o = env.reset(seed0 + ep)
        buf = []                                                     # temporal ensemble over overlapping chunks
        for t in range(200):
            k = 224 // RES
            img = np.concatenate([o["image_wrist"][::k, ::k][:RES, :RES], o["image_exterior"][::k, ::k][:RES, :RES]], -1)
            s = np.concatenate([o["qpos"], [o["gripper"]], o["eef_pos"], o["eef_quat"]]).astype(np.float32)
            with torch.no_grad():
                x = torch.from_numpy(img).permute(2, 0, 1)[None].float() / 255
                sn = torch.from_numpy((s - norm["s_mu"]) / norm["s_sd"])[None]
                ch = net(x, sn)[0].numpy() * norm["a_sd"] + norm["a_mu"]
            buf.append((t, ch))
            acts = [c[t - t0] for t0, c in buf if 0 <= t - t0 < CHUNK]
            w = np.exp(-0.5 * np.arange(len(acts)))[::-1]
            a = (np.array(acts) * w[:, None]).sum(0) / w.sum()
            buf = [(t0, c) for t0, c in buf if t - t0 < CHUNK]
            a[6] = 1.0 if a[6] > 0.5 else 0.0
            o, r, done, info = env.step(a)
            if info["success"]:
                ok += 1
                break
    return ok / episodes


def main():
    import torch
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", default="sw/out/pip_bench_pick_place")
    ap.add_argument("--epochs", type=int, default=30)
    ap.add_argument("--eval", type=int, default=20)
    a = ap.parse_args()
    torch.set_num_threads(4)
    t0 = time.time()
    S, A, I, E, info = load(Path(a.data))
    n_ep = int(E.max()) + 1
    val = E >= int(n_ep * 0.9)
    norm = dict(s_mu=S.mean(0), s_sd=S.std(0) + 1e-6, a_mu=A.reshape(-1, 7).mean(0), a_sd=A.reshape(-1, 7).std(0) + 1e-6)
    Xi = torch.from_numpy(I).permute(0, 3, 1, 2).contiguous()
    Xs = torch.from_numpy((S - norm["s_mu"]) / norm["s_sd"]).float()
    Y = torch.from_numpy((A - norm["a_mu"]) / norm["a_sd"]).float()
    tr = np.where(~val)[0]
    va = np.where(val)[0]
    net = model()
    opt = torch.optim.AdamW(net.parameters(), 1e-3, weight_decay=1e-4)
    t_load = time.time() - t0
    hist = []
    for ep in range(a.epochs):
        net.train()
        perm = np.random.permutation(tr)
        tot = 0.0
        for b in range(0, len(perm), 128):
            idx = perm[b:b + 128]
            x = Xi[idx].float() / 255
            x = x + 0.03 * torch.randn_like(x)                      # light augmentation
            loss = (net(x, Xs[idx]) - Y[idx]).abs().mean()
            opt.zero_grad(); loss.backward(); opt.step()
            tot += float(loss) * len(idx)
        net.eval()
        with torch.no_grad():
            vl = float((net(Xi[va].float() / 255, Xs[va]) - Y[va]).abs().mean())
        hist.append((ep, tot / len(tr), vl))
        print(f"epoch {ep:2d} train L1 {tot / len(tr):.3f} val L1 {vl:.3f}", flush=True)
    t_train = time.time() - t0 - t_load
    sr = evaluate(net, norm, a.eval)
    out = dict(episodes=n_ep, frames=len(S), load_s=t_load, train_s=t_train, epochs=a.epochs, final_train=hist[-1][1], final_val=hist[-1][2],
               closed_loop_success=sr, eval_episodes=a.eval)
    Path("docs/sw").mkdir(parents=True, exist_ok=True)
    Path("docs/sw/train_bc.json").write_text(json.dumps(out, indent=1))
    L = ["# S0.4a training-pipeline dry run (generated by `python -m sw.train_bc`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Data:** {n_ep} synthetic episodes, {len(S):,} frames (90/10 episode split). Load + decode {t_load:.0f} s; {a.epochs} epochs on 4 CPU threads in {t_train / 60:.1f} min.", "",
         f"**Result:** validation L1 (normalised) {hist[-1][2]:.3f}; **closed-loop success in the twin {sr:.0%}** over {a.eval} unseen cube poses.", "",
         "| Epoch | Train L1 | Val L1 |", "|---|---|---|"] + [f"| {e} | {t:.3f} | {v:.3f} |" for e, t, v in hist[::5] + [hist[-1]]]
    Path("docs/sw/train_bc.md").write_text("\n".join(L))
    print(L[6])


if __name__ == "__main__":
    main()
