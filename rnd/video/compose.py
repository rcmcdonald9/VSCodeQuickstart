"""Pip v4 hype video, step 5: edit, grade, kinetic type and mux (numpy + PIL -> ffmpeg).

python -m rnd.video.compose SHOTS_DIR TWIN_DIR MUSIC.wav OUT.mp4 [POSTER.png]
SHOTS_DIR/<shot>/####.png from render_shots.py (any resolution; frames missing from a step-2 render are blended
from their neighbours), TWIN_DIR from twin_footage.py, MUSIC.wav from soundtrack.py. The edit lives in timeline.py.
"""
from __future__ import annotations

import math
import subprocess
import sys
from functools import lru_cache
from multiprocessing import Pool
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFilter, ImageFont

from .timeline import CARDS, DURATION, FPS, IMPACTS, SHOTS

W, H = 1920, 1080
BAR = 138                                    # 2.39:1 letterbox
FONT_BLACK = "/usr/share/fonts/opentype/inter/InterDisplay-Black.otf"
FONT_BOLD = "/usr/share/fonts/opentype/inter/InterDisplay-Bold.otf"
FONT_MED = "/usr/share/fonts/opentype/inter/InterDisplay-Medium.otf"
FONT_MONO = "/usr/share/fonts/truetype/jetbrains-mono/JetBrainsMono-Bold.ttf"
COL = {"w": (245, 244, 240), "o": (255, 118, 22), "b": (90, 150, 255)}
ORANGE = np.array([1.0, 0.46, 0.09], np.float32)
SHOTS_DIR = TWIN_DIR = None


def clamp(x, a=0.0, b=1.0):
    return max(a, min(b, x))


def ease_out_back(t, s=1.9):
    t = clamp(t); c3 = s + 1
    return 1 + c3 * (t - 1) ** 3 + s * (t - 1) ** 2


def ease_out(t):
    t = clamp(t); return 1 - (1 - t) ** 3


@lru_cache(maxsize=64)
def font(path, size):
    try:
        return ImageFont.truetype(path, max(8, int(size)))
    except OSError:
        return ImageFont.truetype(FONT_BOLD, max(8, int(size)))


# ------------------------------------------------------------------ sources
def _seq(d: Path):
    return sorted(d.glob("*.png")) + sorted(d.glob("*.jpg"))


@lru_cache(maxsize=None)
def seq_index(d: str):
    files = _seq(Path(d))
    return {int(f.stem): f for f in files}


def load(path):
    im = Image.open(path).convert("RGB")
    if im.size != (W, H):
        im = im.resize((W, H), Image.LANCZOS)
    return np.asarray(im, np.float32) / 255.0


def shot_frame(name, i, loop=False):
    idx = seq_index(str(SHOTS_DIR / name))
    if not idx:
        return np.zeros((H, W, 3), np.float32)
    keys = sorted(idx)
    hi = keys[-1]
    if loop:
        i = i % (hi + 1)
    i = int(clamp(i, keys[0], hi))
    if i in idx:
        return load(idx[i])
    a = max(k for k in keys if k < i); b = min(k for k in keys if k > i)        # step-2 render: blend neighbours
    w = (i - a) / (b - a)
    return load(idx[a]) * (1 - w) + load(idx[b]) * w


def twin_frame(sub, i):
    idx = seq_index(str(TWIN_DIR / sub))
    keys = sorted(idx)
    return load(idx[int(clamp(i, keys[0], keys[-1]))])


def last_index(name):
    idx = seq_index(str(SHOTS_DIR / name))
    return max(idx) if idx else 0


# ------------------------------------------------------------------ effects
def blur(a, r, down=4):
    """Gaussian blur of an HxWxC float image, done at 1/down scale."""
    h, w = a.shape[:2]
    im = Image.fromarray((np.clip(a, 0, 1) * 255).astype(np.uint8))
    im = im.resize((w // down, h // down), Image.BILINEAR).filter(ImageFilter.GaussianBlur(r / down))
    return np.asarray(im.resize((w, h), Image.BILINEAR), np.float32) / 255.0


def glow_and_streak(img, strength=1.0):
    small = np.asarray(Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8)).resize((W // 4, H // 4), Image.BILINEAR), np.float32) / 255
    lum = small @ np.array([0.3, 0.55, 0.15], np.float32)
    m = np.clip((lum - 0.72) / 0.28, 0, 1)[..., None] * small
    # anamorphic streak: wide horizontal blur
    k = np.ones(121, np.float32) / 121
    streak = np.apply_along_axis(lambda r: np.convolve(r, k, "same"), 1, m.mean(2))
    streak = np.repeat(streak[..., None], 3, 2) * np.array([0.55, 0.75, 1.0], np.float32)
    bloom = np.asarray(Image.fromarray((m * 255).astype(np.uint8)).filter(ImageFilter.GaussianBlur(6)), np.float32) / 255
    add = np.asarray(Image.fromarray((np.clip(streak * 2.2 + bloom * 0.9, 0, 1) * 255).astype(np.uint8)).resize((W, H), Image.BILINEAR), np.float32) / 255
    return img + add * 0.45 * strength


_VIG = None
_SIDE = None


def _side_mask():
    """Darken the left half under a side card."""
    global _SIDE
    if _SIDE is None:
        x = np.arange(W, dtype=np.float32) / W
        _SIDE = (0.28 + 0.72 * np.clip((x - 0.18) / 0.45, 0, 1) ** 1.5)[None, :, None]
    return _SIDE


def vignette(img, k=0.55):
    global _VIG
    if _VIG is None:
        y, x = np.mgrid[0:H, 0:W].astype(np.float32)
        r = np.sqrt(((x - W / 2) / (W / 2)) ** 2 + ((y - H / 2) / (H / 2)) ** 2) / math.sqrt(2)
        _VIG = (1 - k * r ** 2.2)[..., None]
    return img * _VIG


def grade(img):
    lum = img @ np.array([0.3, 0.55, 0.15], np.float32)
    sh = (1 - np.clip(lum, 0, 1))[..., None] ** 2
    img = img + sh * np.array([-0.004, 0.006, 0.02], np.float32)          # cool shadows
    hl = np.clip(lum - 0.6, 0, 1)[..., None]
    img = img + hl * np.array([0.03, 0.01, -0.02], np.float32)            # warm highlights
    return img


def grain(img, n, amt=0.022):
    rng = np.random.default_rng(n)
    g = rng.standard_normal((H // 2, W // 2), dtype=np.float32)
    g = np.repeat(np.repeat(g, 2, 0), 2, 1)[..., None]
    return img + g * amt


def transform(img, scale=1.0, dx=0.0, dy=0.0):
    if abs(scale - 1) < 1e-3 and abs(dx) < 0.5 and abs(dy) < 0.5:
        return img
    im = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
    a = 1 / scale
    cx, cy = W / 2, H / 2
    im = im.transform((W, H), Image.AFFINE, (a, 0, cx - a * cx - dx * a, 0, a, cy - a * cy - dy * a), resample=Image.BILINEAR)
    return np.asarray(im, np.float32) / 255


def chroma(img, k):
    k = int(round(k))
    if k <= 0:
        return img
    out = img.copy()
    out[:, k:, 0] = img[:, :-k, 0]
    out[:, :-k, 2] = img[:, k:, 2]
    return out


def letterbox(img):
    img[:BAR] = 0; img[H - BAR:] = 0
    return img


# ------------------------------------------------------------------ type
def text_layer():
    return Image.new("L", (W, H), 0), Image.new("RGB", (W, H), (0, 0, 0))


def draw_text(layer, text, path, size, xy, color, alpha=1.0, anchor="mm", tracking=0.0):
    a, c = layer
    f = font(path, size)
    da, dc = ImageDraw.Draw(a), ImageDraw.Draw(c)
    if not tracking:
        da.text(xy, text, font=f, fill=int(255 * alpha), anchor=anchor)
        dc.text(xy, text, font=f, fill=color, anchor=anchor)
        return f.getlength(text)
    # letter-spaced: lay out manually
    widths = [f.getlength(ch) for ch in text]
    total = sum(widths) + tracking * size * (len(text) - 1)
    x0 = xy[0] - total / 2 if anchor[0] == "m" else xy[0]
    for ch, w_ in zip(text, widths):
        da.text((x0, xy[1]), ch, font=f, fill=int(255 * alpha), anchor="l" + anchor[1])
        dc.text((x0, xy[1]), ch, font=f, fill=color, anchor="l" + anchor[1])
        x0 += w_ + tracking * size
    return total


def apply_text(img, layer, glow=0.9, glow_col=ORANGE):
    a, c = layer
    A = np.asarray(a, np.float32)[..., None] / 255
    if A.max() == 0:
        return img
    C = np.asarray(c, np.float32) / 255
    if glow > 0:
        g = np.asarray(a.resize((W // 4, H // 4), Image.BILINEAR).filter(ImageFilter.GaussianBlur(7)).resize((W, H), Image.BILINEAR), np.float32)[..., None] / 255
        img = img + g * glow_col * glow * 1.4
    return img * (1 - A) + C * A


def cards_at(t):
    return [c for c in CARDS if c[0] <= t < c[1]]


def draw_cards(img, t):
    act = cards_at(t)
    if not act:
        return img
    layer = text_layer()
    glow = 0.8
    stack = [c for c in act if c[3] not in ("title", "hero_title", "side")]
    # stacked slam/quiet cards: the cards that leave together form one centred block, laid out up front so lines
    # that arrive later do not shove the earlier ones around
    lines = []
    if stack:
        group = [c for c in CARDS if c[1] == stack[0][1] and c[3] not in ("title", "hero_title", "side")]
        for c in group:
            for txt, size, col in c[2]:
                lines.append((c, txt, size, col))
    if lines:
        total = sum(l[2] * 1.02 for l in lines) + 0.18 * sum(l[2] for l in lines[1:])
        y = H / 2 - total / 2
        for c, txt, size, col in lines:
            dt = t - c[0]
            if dt < 0:
                y += size * 1.02 + size * 0.18
                continue
            style = c[3]
            yc = y + size * 0.51
            if style.startswith("quiet"):
                al = clamp(dt / 0.35) * clamp((c[1] - t) / 0.25)
                draw_text(layer, txt, FONT_BOLD, size, (W / 2, yc), COL[col], al, tracking=0.32 + 0.04 * dt)
                glow = 0.35
            else:
                s = 1.0 + 0.9 * (1 - ease_out_back(dt / 0.22)) + 0.035 * dt                  # slam, then a slow push
                if style == "count":
                    try:
                        txt = str(int(round(int(txt) * ease_out(dt / 0.7))))
                    except ValueError:
                        pass
                al = clamp(dt / 0.05)
                if size >= 140 and dt < 0.2:                                                 # ghost echo
                    draw_text(layer, txt, FONT_BLACK, size * (1.0 + 1.6 * dt), (W / 2, yc), COL[col], 0.16 * (1 - dt / 0.2), tracking=0.02)
                draw_text(layer, txt, FONT_BLACK if size >= 90 else FONT_BOLD, size * s, (W / 2, yc), COL[col], al,
                          tracking=0.02 if size >= 90 else 0.12)
            y += size * 1.02 + size * 0.18
    for c in act:
        dt = t - c[0]; style = c[3]
        if style == "side":
            x = 150 - 260 * (1 - ease_out(dt / 0.3))
            al = clamp(dt / 0.12) * clamp((c[1] - t) / 0.2)
            yy = H * 0.5 - sum(l[1] for l in c[2]) * 0.55
            for txt, size, col in c[2]:
                draw_text(layer, txt, FONT_BLACK if size >= 90 else FONT_BOLD, size, (x, yy + size * 0.5), COL[col], al, anchor="lm",
                          tracking=0.0 if size >= 90 else 0.1)
                yy += size * 1.12
        elif style in ("title", "hero_title"):
            hero = style == "hero_title"
            s = (1.0 + 0.7 * (1 - ease_out_back(dt / 0.25))) * (1 + 0.025 * dt)
            al = clamp(dt / (0.06 if not hero else 0.5)) * (clamp((c[1] - t) / 0.3) if hero else 1)
            (t1, s1, c1), (t2, s2, c2) = c[2]
            f1 = font(FONT_BLACK, s1 * s); f2 = font(FONT_BLACK, s2 * s)
            w1, w2 = f1.getlength(t1), f2.getlength(t2)
            gap = 30 * s
            x0 = W / 2 - (w1 + gap + w2) / 2
            if hero:
                s *= 0.62
                f1 = font(FONT_BLACK, s1 * s); f2 = font(FONT_BLACK, s2 * s)
                w1, w2 = f1.getlength(t1), f2.getlength(t2); gap = 30 * s
                x0 = W / 2 - (w1 + gap + w2) / 2
            yb = H - BAR - 50 if hero else H / 2 + s1 * s * 0.36
            draw_text(layer, t1, FONT_BLACK, s1 * s, (x0, yb), COL[c1], al, anchor="ls", tracking=0.0)
            draw_text(layer, t2, FONT_BLACK, s2 * s, (x0 + w1 + gap, yb), COL[c2], al, anchor="ls")
            glow = 1.0
    return apply_text(img, layer, glow)


def draw_hud(img, t, label, twin=False):
    if not label:
        return img
    seg = [s for s in SHOTS if s[0] <= t < s[1]][0]
    dt = t - seg[0]
    n = int(len(label) * clamp(dt / 0.6))
    layer = text_layer()
    draw_text(layer, label[:n] + ("_" if n < len(label) and int(t * 8) % 2 else ""), FONT_MONO, 26, (150, H - BAR / 2), COL["w"], 0.9, anchor="lm")
    ImageDraw.Draw(layer[0]).rectangle((110, H - BAR / 2 - 12, 118, H - BAR / 2 + 12), fill=255)
    ImageDraw.Draw(layer[1]).rectangle((110, H - BAR / 2 - 12, 118, H - BAR / 2 + 12), fill=COL["o"])
    tag = "● DIGITAL TWIN  //  MUJOCO" if twin else "PIP v4  //  CAD RENDER"
    if not twin or int(t * 2) % 2 == 0 or True:
        draw_text(layer, tag, FONT_MONO, 22, (W - 110, BAR / 2), COL["o"] if twin else (150, 150, 150), 0.85, anchor="rm")
    return apply_text(img, layer, 0.0)


def logo(t):
    dt = t - 72.0
    img = np.zeros((H, W, 3), np.float32)
    layer = text_layer()
    s = (1.0 + 0.5 * (1 - ease_out_back(dt / 0.3))) * (1 + 0.012 * dt)
    f1, f2 = font(FONT_BLACK, 260 * s), font(FONT_BLACK, 130 * s)
    w1, w2 = f1.getlength("PIP"), f2.getlength("v4")
    x0 = W / 2 - (w1 + 26 * s + w2) / 2
    yb = H * 0.47
    draw_text(layer, "PIP", FONT_BLACK, 260 * s, (x0, yb), COL["w"], 1, anchor="ls")
    draw_text(layer, "v4", FONT_BLACK, 130 * s, (x0 + w1 + 26 * s, yb), COL["o"], 1, anchor="ls")
    rule = int(700 * ease_out((dt - 0.3) / 0.6))
    if rule > 0:
        ImageDraw.Draw(layer[0]).rectangle((W / 2 - rule / 2, yb + 40, W / 2 + rule / 2, yb + 46), fill=255)
        ImageDraw.Draw(layer[1]).rectangle((W / 2 - rule / 2, yb + 40, W / 2 + rule / 2, yb + 46), fill=COL["o"])
    al = clamp((dt - 0.6) / 0.5)
    draw_text(layer, "COMING TO A KITCHEN NEAR YOU", FONT_BOLD, 54, (W / 2, yb + 120), COL["w"], al, tracking=0.22)
    al2 = clamp((dt - 1.4) / 0.6)
    draw_text(layer, "A 3D-printed two-arm home robot, designed and tested in simulation. CAD renders and digital-twin footage;",
              FONT_MED, 24, (W / 2, H - 120), (160, 160, 165), al2)
    draw_text(layer, "no physical unit exists yet and nothing is for sale. All numbers are design targets or estimates from the R&D loop.",
              FONT_MED, 24, (W / 2, H - 86), (160, 160, 165), al2)
    img = apply_text(img, layer, 1.1)
    fade = clamp((DURATION - 0.1 - t) / 1.6)
    return img * fade


# ------------------------------------------------------------------ frame
def frame(n):
    t = n / FPS
    seg = [s for s in SHOTS if s[0] <= t < s[1]]
    t0, t1, src, first, speed, label = seg[0] if seg else SHOTS[-1]
    i = first + int(round((t - t0) * FPS * speed))
    footage = True
    if src == "black":
        img = np.zeros((H, W, 3), np.float32); footage = False
    elif src == "logo":
        img = logo(t); footage = False
    elif src.endswith("_end"):
        base = src[:-4]
        img = shot_frame(base, last_index(base))
        u = (t - t0) / (t1 - t0)
        img = transform(img, 1.04 + 0.08 * u)
        img = blur(img, 10, 2) * 0.32
    elif src == "bench":
        img = twin_frame("bench_ext", i)
        wr = Image.fromarray((twin_frame("bench_wrist", i) * 255).astype(np.uint8)).resize((360, 360), Image.LANCZOS)
        pil = Image.fromarray((np.clip(img, 0, 1) * 255).astype(np.uint8))
        d = ImageDraw.Draw(pil)
        x0, y0 = W - 110 - 360, BAR + 40
        d.rectangle((x0 - 4, y0 - 4, x0 + 364, y0 + 364), fill=COL["o"])
        pil.paste(wr, (x0, y0))
        d.text((x0 + 12, y0 + 326), "WRIST CAM", font=font(FONT_MONO, 22), fill=(255, 255, 255))
        img = np.asarray(pil, np.float32) / 255
    elif src == "fetch":
        img = twin_frame("fetch", i)
    else:
        img = shot_frame(src, i, loop=(src == "glimpse"))
    twin = src in ("bench", "fetch")
    if twin:
        img = np.clip(img * 0.82, 0, 1) ** 1.35 * np.array([0.92, 0.97, 1.05], np.float32)     # tame the bright sim lighting
    act = cards_at(t)
    if footage and any(c[3] not in ("side", "hero_title") for c in act):
        img = img * 0.42
    if footage and any(c[3] == "side" for c in act):
        img = img * _side_mask()
    # impact punch: zoom, shake, chromatic aberration, flash
    past = [x for x in IMPACTS if x <= t + 1e-6]
    di = t - past[-1] if past else 9
    if footage or cards_at(t):
        if di < 0.5:
            e = math.exp(-di * 9)
            rng = np.random.default_rng(n)
            img = transform(img, 1 + 0.06 * e, *(rng.uniform(-1, 1, 2) * 22 * e))
        if footage and not twin:
            img = glow_and_streak(img)
    img = grade(img)
    img = draw_cards(img, t)
    if di < 0.5:
        e = math.exp(-di * 9)
        img = chroma(img, 12 * e)
        img = img + 0.55 * math.exp(-di * 16)
    if footage:
        img = vignette(img)
        img = letterbox(img)
        img = draw_hud(img, t, label, twin)
    img = grain(img, n, 0.016 if footage else 0.01)
    return (np.clip(img, 0, 1) * 255).astype(np.uint8).tobytes()


def _init(shots, twin):
    global SHOTS_DIR, TWIN_DIR
    SHOTS_DIR, TWIN_DIR = Path(shots), Path(twin)


def main(shots, twin, music, out, poster=None, first=0, last=None, workers=4):
    _init(shots, twin)
    nf = int(DURATION * FPS)
    last = nf if last is None else last
    if poster:
        _init(shots, twin)
        Image.frombytes("RGB", (W, H), frame(int(69.0 * FPS))).save(poster)
        print("poster", poster)
    cmd = ["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS), "-i", "-",
           "-ss", f"{first / FPS:.3f}", "-i", music, "-map", "0:v", "-map", "1:a", "-c:v", "libx264", "-preset", "slow", "-crf", "20", "-maxrate", "4500k", "-bufsize", "9000k",
           "-pix_fmt", "yuv420p", "-profile:v", "high", "-movflags", "+faststart", "-c:a", "aac", "-b:a", "192k", "-shortest", out]
    p = subprocess.Popen(cmd, stdin=subprocess.PIPE)
    with Pool(workers, initializer=_init, initargs=(shots, twin)) as pool:
        for k, buf in enumerate(pool.imap(frame, range(first, last), chunksize=4)):
            p.stdin.write(buf)
            if k % 150 == 0:
                print("frame", first + k, "/", last, flush=True)
    p.stdin.close(); p.wait()
    print("wrote", out)


if __name__ == "__main__":
    a = sys.argv[1:]
    kw = {}
    for x in list(a):
        if x.startswith("--first="): kw["first"] = int(x[8:]); a.remove(x)
        elif x.startswith("--last="): kw["last"] = int(x[7:]); a.remove(x)
    main(*a, **kw)
