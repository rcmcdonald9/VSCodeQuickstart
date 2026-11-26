"""Pip v4 hype video, step 4: synthesise the trailer soundtrack (numpy only, no samples).

python -m rnd.video.soundtrack OUT.wav
120 BPM four-on-the-floor kick with sidechained sub bass, A-minor trailer progression, detuned-saw pads,
filtered-noise risers, braams + sub drops on the two drops, a big impact on every title card, a click on the KX lock.
"""
from __future__ import annotations

import sys
import wave

import numpy as np
from scipy.signal import butter, fftconvolve, sosfilt

from .timeline import BEAT, CLICK, DROPS, DURATION, IMPACTS, KICK_ON, RISERS, SMALL_HITS

SR = 48000
N = int(DURATION * SR) + SR
rng = np.random.default_rng(7)


def t_(dur):
    return np.arange(int(dur * SR)) / SR


def lp(x, f, order=2):
    return sosfilt(butter(order, f, "low", fs=SR, output="sos"), x)


def hp(x, f, order=2):
    return sosfilt(butter(order, f, "high", fs=SR, output="sos"), x)


def bp(x, lo, hi, order=2):
    return sosfilt(butter(order, [lo, hi], "band", fs=SR, output="sos"), x)


def place(buf, x, t, gain=1.0):
    i = int(t * SR)
    if i >= len(buf):
        return
    n = min(len(x), len(buf) - i)
    buf[i:i + n] += gain * x[:n]


def saw(f, t, phase=0.0):
    ph = np.cumsum(np.broadcast_to(f, t.shape) / SR) + phase
    return 2 * (ph % 1.0) - 1


def in_ranges(t, ranges):
    return any(a <= t < b for a, b in ranges)


# ------------------------------------------------------------------ instruments
def kick():
    t = t_(0.45)
    f = 45 + 110 * np.exp(-t * 28)
    body = np.sin(2 * np.pi * np.cumsum(f) / SR) * np.exp(-t * 7.5)
    click = hp(rng.standard_normal(len(t)), 2500) * np.exp(-t * 300) * 0.35
    return np.tanh(1.8 * (body + click))


def clap():
    t = t_(0.35)
    n = bp(rng.standard_normal(len(t)), 900, 5000) * (np.exp(-t * 22) + 0.4 * np.exp(-((t - 0.012) * 400) ** 2))
    tone = np.sin(2 * np.pi * 190 * t) * np.exp(-t * 30) * 0.5
    return 0.6 * (n + tone)


def hat(open_=False):
    t = t_(0.25 if open_ else 0.06)
    return hp(rng.standard_normal(len(t)), 7000) * np.exp(-t * (14 if open_ else 70)) * 0.25


def impact(big=True):
    dur = 4.5 if big else 1.6
    t = t_(dur)
    f = 32 + 70 * np.exp(-t * 6)
    boom = np.sin(2 * np.pi * np.cumsum(f) / SR) * np.exp(-t * (1.1 if big else 3.0))
    crack = lp(rng.standard_normal(len(t)), 6000) * np.exp(-t * (9 if big else 18))
    metal = sum(np.sin(2 * np.pi * fr * t + p) for fr, p in ((233, 0), (377, 1), (521, 2), (811, 3))) * np.exp(-t * 3.5) * 0.08
    x = np.tanh(2.2 * boom) * 0.9 + 0.45 * crack + (metal if big else 0)
    return x


def braam(root=55.0, dur=3.2):
    t = t_(dur)
    env = np.minimum(1, t / 0.08) * np.exp(-t * 0.7)
    x = sum(saw(root * m * (1 + d), t, p) for m, d, p in ((1, 0, 0), (1, 0.006, 0.3), (2, -0.004, 0.6), (1.5, 0.003, 0.1), (0.5, 0, 0.2)))
    x = lp(x, 900, 2)
    return np.tanh(1.6 * x * env) * 0.9


def riser(dur):
    t = t_(dur)
    u = t / dur
    noise = rng.standard_normal(len(t))
    # sweep a band-pass up by filtering in short blocks
    out = np.zeros_like(noise)
    blk = 2048
    for i in range(0, len(t), blk):
        fc = 300 * (40 ** u[i])
        seg = noise[max(0, i - blk):i + blk]
        y = bp(seg, fc * 0.6, min(fc * 1.6, SR / 2 - 100), 1)
        out[i:i + blk] = y[-len(noise[i:i + blk]):] if i else y[:blk]
    tone = np.sin(2 * np.pi * np.cumsum(110 * 2 ** (u * 3)) / SR) * 0.25
    env = u ** 2.2
    return (out * 0.8 + tone) * env


def whoosh(dur=0.5):
    t = t_(dur); u = t / dur
    x = bp(rng.standard_normal(len(t)), 400, 4000) * np.sin(np.pi * u) ** 2
    return x * 0.5


def reverb(x, secs=2.2, mix=0.25):
    t = t_(secs)
    ir = rng.standard_normal(len(t)) * np.exp(-t * 6.9 / secs)
    ir = lp(ir, 5000); ir /= np.sqrt(np.sum(ir ** 2))
    wet = fftconvolve(x, ir)[:len(x)]
    return x + mix * wet


# ------------------------------------------------------------------ arrangement
PROG = [(55.00, (220.0, 261.63, 329.63)),   # Am
        (43.65, (174.61, 220.0, 261.63)),   # F
        (36.71, (146.83, 174.61, 220.0)),   # Dm
        (41.20, (164.81, 207.65, 246.94))]  # E (harmonic minor tension)


def main(path):
    drums = np.zeros(N); bass = np.zeros(N); pads = np.zeros(N); fx = np.zeros(N)
    duck = np.ones(N)
    K, C = kick(), clap()
    nb = int(DURATION / BEAT)
    for b in range(nb):
        t = b * BEAT
        if in_ranges(t, KICK_ON):
            place(drums, K, t, 1.25)
            i = int(t * SR); n = int(0.22 * SR)
            duck[i:i + n] = np.minimum(duck[i:i + n], 1 - 0.75 * np.exp(-np.arange(min(n, N - i)) / SR * 14))
            if t >= 16 and b % 2 == 1:
                place(drums, C, t, 0.7)
            if t >= 10:
                place(drums, hat(open_=(b % 4 == 3)), t + BEAT / 2, 0.9)
            if t >= 34:
                place(drums, hat(), t + BEAT / 4, 0.45); place(drums, hat(), t + 3 * BEAT / 4, 0.45)
        elif t < 8.0 and b % 2 == 0 and t >= 2.0:            # intro heartbeat
            place(drums, lp(K, 120), t, 0.8); place(drums, lp(K, 120), t + 0.22, 0.5)
    # bass + pads: one chord per bar (2 s)
    for bar in range(int(DURATION / (4 * BEAT)) + 1):
        t0 = bar * 4 * BEAT
        root, chord = PROG[bar % 4]
        tt = t_(4 * BEAT)
        if 8.0 <= t0 < 60.0 or 62.0 <= t0 < 72.0:
            sub = np.sin(2 * np.pi * root * tt) * 0.9
            mid = lp(saw(root * 2, tt), 500) * 0.35
            # eighth-note pulse
            pulse = 0.55 + 0.45 * (np.sin(2 * np.pi * tt / (BEAT / 2)) > -0.2)
            place(bass, (sub + mid) * pulse * np.minimum(1, tt / 0.01), t0)
        level = 0.06 if t0 < 8 else (0.12 if t0 < 60 else (0.05 if t0 < 62 else 0.2))
        if t0 < 78:
            p = sum(saw(f * (1 + d), tt, ph) for f in chord for d, ph in ((-0.004, 0.1), (0.0, 0.5), (0.005, 0.8)))
            p = lp(p, 1200 if t0 >= 8 else 500)
            env = np.minimum(1, tt / 0.6) * np.minimum(1, (4 * BEAT - tt) / 0.3)
            place(pads, p * env, t0, level)
    # intro drone
    tt = t_(8.0)
    drone = lp(saw(55.0, tt) + saw(55.3, tt, 0.4) + saw(82.4, tt, 0.2), 250) * np.minimum(1, tt / 3.0) * 0.35
    place(pads, drone, 0.0)
    # outro: sustained A minor + ring-out
    tt = t_(6.0)
    out = lp(sum(saw(f, tt, i * 0.3) for i, f in enumerate((55, 110, 220, 261.63, 329.63, 440))), 900) * np.exp(-tt * 0.55) * 0.25
    place(pads, out, 72.0)
    # fx
    for a, b in RISERS:
        place(fx, riser(b - a), a, 0.55 if b - a > 1.5 else 0.35)
    for t in IMPACTS:
        big = t in (8.0, 62.0, 72.0) or t % 2 == 0
        place(fx, impact(big), t, 0.9 if big else 0.6)
        place(fx, whoosh(0.4), t - 0.4, 0.5)
    for t in SMALL_HITS:
        place(fx, impact(False), t, 0.35)
    for t in DROPS + [72.0]:
        place(fx, braam(55.0), t, 0.8)
    tc = t_(0.08)
    click = hp(rng.standard_normal(len(tc)), 1500) * np.exp(-tc * 120) + np.sin(2 * np.pi * 2400 * tc) * np.exp(-tc * 90) * 0.6
    place(fx, click, CLICK, 0.9); place(fx, click, CLICK + 0.06, 0.6)
    # mix
    mix = drums + bass * duck * 0.45 + pads * duck ** 0.5 + reverb(fx, 2.5, 0.35)
    mix = reverb(mix, 1.2, 0.08)
    mix = hp(mix, 25)
    # fade the tail
    tail = int(76.5 * SR)
    mix[tail:] *= np.linspace(1, 0, len(mix) - tail) ** 2
    mix = mix[:int(DURATION * SR)]
    tt = np.arange(len(mix)) / SR                       # keep the intro low so the first drop lands
    mix *= np.interp(tt, [0, 5.5, 7.9, 8.0], [0.45, 0.5, 0.8, 1.0])
    mix = np.tanh(1.3 * mix / np.max(np.abs(mix)) * 1.4) * 0.92
    mix /= np.max(np.abs(mix)) / 0.95
    # light stereo: haas-delayed pads/fx on the right
    d = int(0.012 * SR)
    side = np.concatenate([np.zeros(d), (pads * duck ** 0.5 + fx * 0.3)[:len(mix) - d]])
    side = side / (np.max(np.abs(side)) + 1e-9) * 0.08
    L, R = mix + side * 0.0, mix * 0.97 + side
    st = np.stack([L, R], 1)
    st /= np.max(np.abs(st)) / 0.95
    pcm = (st * 32767).astype(np.int16)
    with wave.open(path, "wb") as w:
        w.setnchannels(2); w.setsampwidth(2); w.setframerate(SR); w.writeframes(pcm.tobytes())
    print("wrote", path, f"{len(pcm) / SR:.1f} s")


if __name__ == "__main__":
    main(sys.argv[1])
