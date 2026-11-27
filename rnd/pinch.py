"""Pinch / shear / draw-in check over the joint sweeps (R17, D23).  python -m rnd.pinch [--quick] [--joints a,b]

Works on the assembled robot (robot.build(..., covers=True)) and knows nothing about the parts' geometry, so it keeps
working while the arm CAD changes:

1. **Kinematics by experiment.** For every pose key robot.build understands (arm pitch / elbow / wrist per side, head
   pan / tilt, the lift) the robot is rebuilt with that one joint nudged. The parts that moved, grouped by side, give
   the joint's rigid motion (Kabsch fit on part centroids → axis and angle), so a joint is swept by moving the reference
   surface samples, not by rebuilding the CAD. Roll and yaw are not pose keys of robot.build: they come from the
   rnd/swept.py joint definitions (roll-module frame), mapped into each arm by a fit of the roll module onto the robot.
   The lift is a rebuild per height (its bellows change shape). CAD functions are memoised during the run.
2. **Finger probe.** Every part is sampled on its surface (≈ 0.35 points/mm², outward normals). At each pose the free
   space is voxelised (4 mm); a voxel is free when no surface sample lies within 6 mm of its centre (a Ø12 mm probe
   tip fits there), and it is *reachable* when its free region connects to the outside (not the floor). A surface
   point is accessible when a reachable probe centre touches it from its outward side. The probe can bend round
   corners, so this is stricter than a straight ISO 13857 test finger (ESTIMATE: the 4 mm lattice makes the effective
   probe Ø12–16 mm).
3. **Gaps.** For each swept joint, each 20 mm cell of a moving part is tracked against the nearest static part
   (anything that does not move with that joint) at every step. A cell counts if a finger can reach it (the cell and
   its opposite surface both accessible at some step). Rule (R17; ISO 13854 finger 25 mm, ISO 13857 openings ≤ 8 mm,
   commonly cited values, confirm against the standards): the gap is ≤ 8 mm at every step **or** ≥ 25 mm at every
   step; a gap that closes from > 8 into < 25 fails. A gap held constant in 8…25 (within SHEAR_TOL over the sweep:
   two surfaces sliding past each other, nothing closes) is listed as a shear point, not counted. Gaps > 40 mm read
   as open.
4. **Soft parts** (names with bellows / boot / gaiter: TPU 95A or fabric) block the probe but are not pinch
   surfaces; in a rigid joint sweep they keep their reference shape (their folding is not simulated, ESTIMATE).
   **Compliant-cover rule** (wave 5): a TPU 95A boot counts as closing the gaps it covers only if its wall is
   ≥ TPU_WALL_MIN (1.2 mm) and, over the full range of the joint it spans, every free span between where it is fixed
   to one body and where it is fixed to the other stays within ±TPU_STRAIN (40 %) of its as-printed (free) length.
   Basis (ESTIMATE until a coupon test): a pleated elastomer bellows takes its stroke by bending the pleat flanks at the
   folds, not by stretching the wall; moulded rubber and printed-TPU bellows are commonly rated at about 0.5–0.6 × free
   length compressed and 1.4–1.5 × extended, beyond which the folds go flat (stretch) or stack solid (crush, and the boot
   itself becomes a pinch); 40 % keeps the fold-root bending strain of a 1.2–2 mm TPU 95A wall with ≥ 1.5 mm fold radii
   to roughly 10–15 %, well inside its elongation at break (> 400 %) for cyclic life. Each cover declares its spans
   (covers.compliant()); a boot that fails is taken out of the model (it does not block the probe anywhere) and is
   listed with the stroke it would need. A soft part with no declared spans is kept as drawn and listed as not judged.
5. **Draw-in / exposed drive parts.** Belts, pulleys and idlers (draw-in), motors and cartridge cans (R17 "no exposed
   belt or motor") fail if any accessible patch ≥ 30 mm² shows at any step; exposed shafts are listed as warnings.

Not covered: the gripper's own jaws (a gripper is meant to pinch; force-limited in software), wheel–floor nip,
two-joint combinations (each joint is swept with the others at the reference pose), hair/clothing entanglement.
"""
from __future__ import annotations

import argparse
import functools
import inspect
import json
import re
import sys
import time
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
CLOSED, OPEN = 8.0, 25.0          # R17: finger can't enter / finger can't be crushed (mm)
PROBE_R = 6.0                     # Ø12 mm probe
VOX = 4.0                         # voxel (mm)
CELL = 20.0                       # gap-tracking cell on moving parts (mm)
BOUND = 40.0                      # gaps beyond this read as open (mm)
SHEAR_TOL = 1.0                   # a gap varying less than this over the sweep is constant (sliding), mm
DENS = 0.35                       # surface samples per mm²
MIN_PATCH = 30.0                  # mm²: smallest exposed patch counted (noise floor, ESTIMATE)
NUDGE = 7.0                       # deg / mm for the kinematics experiment

REF = {"l": dict(pitch=-30.0, elbow=-60.0, wrist=-20.0), "r": dict(pitch=-30.0, elbow=-60.0, wrist=-20.0),
       "head": dict(pan=0.0, tilt=0.0)}           # a typical working pose; each joint is swept from here
REF_LIFT = 0.75

EXPOSE = [(re.compile(r"belt|pulley|idler"), "belt / pulley (draw-in)", True),
          (re.compile(r"motor"), "motor", True),
          (re.compile(r"(^|_)can($|_)"), "motor can", True),
          (re.compile(r"shaft|spindle"), "rotating shaft", False)]
SKIP_GAP = re.compile(r"jaw|finray|pad_fixed|finger")        # gripper fingers vs each other: gripping, not pinching
COVER = re.compile(r"duct|cover|liner|belly|shield|mast_cap|collar_(top|bot)|spine_pod|back_plate")   # our own covers
SOFT = re.compile(r"bellows|boot|gaiter")      # TPU / fabric: blocks the probe, gives under a finger (not a pinch surface)
TPU_WALL_MIN = 1.2                # mm: thinnest TPU 95A wall that counts as a cover (≈ 3 perimeters of a 0.4 nozzle)
TPU_STRAIN = 0.40                 # free spans of a compliant cover stay within ±40 % of the as-printed length over the stroke


def _ranges():
    from .params import BASELINE
    from .twin import JR
    from .parts import upper
    w = BASELINE.arm.wrist_pitch_range_deg
    return {"pitch": JR["pitch"], "elbow": JR["elbow"], "wrist": (-w, w), "roll": JR["roll"], "yaw": JR["yaw"],
            "pan": getattr(upper, "PAN_RANGE", (-90, 90)), "tilt": getattr(upper, "TILT_RANGE", (-30, 30))}


def judge_compliant(ranges=None):
    """Judge every compliant cover declared by the CAD (covers.compliant()) against the rule in the module docstring.
    Rows: name, part (robot part-name suffix, or None for a design study), joint, wall, worst span (free, min, max),
    compression / extension of the worst span, ok, note."""
    from .parts import covers
    ranges = ranges or _ranges()
    rows = []
    for c in covers.compliant():
        lo, hi = ranges[c["joint"]]
        th = np.linspace(lo, hi, int(round(hi - lo)) + 1)
        worst = (0.0, "none: clears the joint's sweep (no deflection)", 0.0, 0.0, 0.0, 0.0, 0.0)
        for name, L0, f in c["spans"]:
            L = np.array([f(t) for t in th])
            comp, ext = 1 - L.min() / L0, L.max() / L0 - 1
            k = max(comp, ext)
            if k >= worst[0]:
                worst = (k, name, L0, float(L.min()), float(L.max()), comp, ext)
        k, sname, L0, lmin, lmax, comp, ext = worst
        ok = bool(c["wall"] >= TPU_WALL_MIN and k <= TPU_STRAIN + 1e-9)
        rows.append(dict(name=c["name"], part=c.get("part"), joint=c["joint"], range=(lo, hi), wall=c["wall"], span=sname,
                         free=float(L0), min=lmin, max=lmax, comp=float(comp), ext=float(ext), ok=ok, note=c.get("note", "")))
    return rows


# ---------------------------------------------------------------------------------------------- CAD memo (speed only)
class _Memo:
    """Memoise every function in rnd.parts.* for the run (CAD builders are pure); lists are copied on return."""

    def __enter__(self):
        import importlib
        import pkgutil
        from . import parts
        self.saved = []
        for m in pkgutil.iter_modules(parts.__path__):
            mod = importlib.import_module(f"{parts.__name__}.{m.name}")
            for k, f in list(vars(mod).items()):
                if inspect.isfunction(f) and f.__module__ == mod.__name__ and not k.startswith("__"):
                    self.saved.append((mod, k, f))
                    setattr(mod, k, self._wrap(f))
        return self

    @staticmethod
    def _wrap(f):
        cache = {}

        @functools.wraps(f)
        def g(*a, **kw):
            try:
                key = (a, tuple(sorted(kw.items())))
                hash(key)
                if not all(isinstance(v, (int, float, str, bool, type(None), tuple)) for v in (*a, *kw.values())):
                    raise TypeError
            except TypeError:
                return f(*a, **kw)
            if key not in cache:
                cache[key] = f(*a, **kw)
            r = cache[key]
            return list(r) if isinstance(r, list) else r
        return g

    def __exit__(self, *exc):
        for mod, k, f in self.saved:
            setattr(mod, k, f)


# ---------------------------------------------------------------------------------------------- geometry helpers
def _sample(part, seed):
    import trimesh
    v, t = part.tessellate(0.4, 0.3)
    if not t:
        return np.zeros((0, 3)), np.zeros((0, 3))
    m = trimesh.Trimesh(np.array([(p.X, p.Y, p.Z) for p in v]), np.array(t), process=False)
    n = max(40, int(m.area * DENS))
    pts, fi = trimesh.sample.sample_surface(m, n, seed=seed)
    nr = m.face_normals[fi]
    if m.volume < 0:
        nr = -nr
    return np.asarray(pts), np.asarray(nr)


def _centroid(part):
    from build123d import CenterOf
    c = part.center(CenterOf.MASS)
    return np.array([c.X, c.Y, c.Z]), part.volume


def _vsig(part):
    return np.array([(v.X, v.Y, v.Z) for v in part.vertices()]).reshape(-1, 3)


def _kabsch(A, B, proper=True):
    """Best rigid (or, proper=False, orthogonal) map A → B: B ≈ A @ R.T + t."""
    ca, cb = A.mean(0), B.mean(0)
    H = (A - ca).T @ (B - cb)
    U, _, Vt = np.linalg.svd(H)
    R = Vt.T @ U.T
    if proper and np.linalg.det(R) < 0:
        Vt[-1] *= -1
        R = Vt.T @ U.T
    t = cb - ca @ R.T
    return R, t, float(np.sqrt(((A @ R.T + t - B) ** 2).sum(1).max()))


def _rot(axis, deg):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    th = np.radians(deg)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(th) * K + (1 - np.cos(th)) * K @ K


def _axis_angle(R, t):
    """Rotation axis direction, angle (deg) and a point on the axis of x → R x + t."""
    ang = np.degrees(np.arccos(np.clip((np.trace(R) - 1) / 2, -1, 1)))
    w = np.array([R[2, 1] - R[1, 2], R[0, 2] - R[2, 0], R[1, 0] - R[0, 1]])
    u = w / (np.linalg.norm(w) + 1e-12)
    c = np.linalg.lstsq(np.eye(3) - R, t - u * (u @ t), rcond=None)[0]
    return u, ang, c


# ---------------------------------------------------------------------------------------------- the model
class Robot:
    def __init__(self, log=print):
        from .parts import robot
        self.robot, self.log = robot, log
        t0 = time.time()
        self.parts = robot.build(REF_LIFT, REF, covers=True)
        self.compliant = judge_compliant()
        bad = [r["part"] for r in self.compliant if r["part"] and not r["ok"]]
        self.dropped = [n for n, _, _ in self.parts if any(_strip(n) == b for b in bad)]
        judged = {r["part"] for r in self.compliant if r["part"]}
        self.unjudged = sorted({_strip(n) for n, _, _ in self.parts if SOFT.search(n) and _strip(n) not in judged})
        self.names = [n for n, _, _ in self.parts]
        self.idx = {n: i for i, n in enumerate(self.names)}
        self.cen, self.sig = {}, {}
        for n, p, _ in self.parts:
            self.cen[n] = _centroid(p)
            self.sig[n] = _vsig(p)
        self.pts, self.nrm, self.pid = [], [], []
        for i, (n, p, _) in enumerate(self.parts):
            P, N = _sample(p, i) if n not in self.dropped else (np.zeros((0, 3)), np.zeros((0, 3)))   # a boot that fails the
            self.pts.append(P); self.nrm.append(N); self.pid.append(np.full(len(P), i))
        self.offs = np.cumsum([0] + [len(p) for p in self.pts])
        self.P = np.concatenate(self.pts); self.N = np.concatenate(self.nrm); self.I = np.concatenate(self.pid)
        # rule closes nothing: it stays in the part list (same sampling seeds for the rest) with no surface samples
        self.log(f"reference build + {len(self.P):,} surface samples on {len(self.parts)} parts in {time.time() - t0:.0f} s"
                 + (f"; compliant covers taken out: {', '.join(self.dropped)}" if self.dropped else ""))

    def moved(self, parts, tol=0.05):
        """{name: (centroid, volume)} of the parts that differ from the reference: compared by their topological
        vertices, so a part turning about its own axis (centroid and box unchanged) still counts as moving."""
        from scipy.spatial import cKDTree
        out = {}
        for n, p, _ in parts:
            s0 = self.sig.get(n)
            s = _vsig(p)
            if s0 is None or len(s) != len(s0) or (len(s) and cKDTree(s0).query(s)[0].max() > tol):
                out[n] = _centroid(p)
        return out

    def groups(self, moved, sides):
        g = {}
        for n in moved:
            k = next((s for s in sides if n.startswith(f"{s}_")), "other")
            g.setdefault(k, []).append(n)
        return g


def rotary_joints(rb, ranges, wanted):
    """[(joint, [(side, member names, u, c, sense)], (lo, hi), {side: ref angle})] from nudge builds."""
    out, skipped = [], []
    arm_sides = [s for s in REF if s != "head"]
    for key in ("pitch", "elbow", "wrist", "pan", "tilt"):
        if wanted and key not in wanted:
            continue
        holders = [s for s, d in REF.items() if key in d]
        poses = {s: dict(d) for s, d in REF.items()}
        lo, hi = ranges[key]
        for s in holders:
            poses[s][key] = REF[s][key] + (NUDGE if REF[s][key] + NUDGE <= hi else -NUDGE)
        mv = rb.moved(rb.robot.build(REF_LIFT, poses, covers=True))
        if not mv:
            skipped.append(key)
            continue
        bodies = []
        for side, members in rb.groups(mv, arm_sides).items():
            if len(members) < 3:
                continue
            A = np.array([rb.cen[n][0] for n in members]); B = np.array([mv[n][0] for n in members])
            R, t, res = _kabsch(A, B)
            u, ang, c = _axis_angle(R, t)
            ref_side = side if side in holders else holders[0]
            d = poses[ref_side][key] - REF[ref_side][key]
            if res > 0.5 or ang < 0.2:
                rb.log(f"  {key}/{side}: not a clean rotation (residual {res:.2f} mm, {ang:.2f}°): skipped")
                continue
            bodies.append((side, members, u, c, ang / d, REF[ref_side][key]))
            rb.log(f"  {key}/{side}: {len(members)} parts, axis {np.round(u, 3)} through {np.round(c, 1)}, fit {res:.3f} mm")
        out.append((key, bodies, (lo, hi)))
    return out, skipped


def swept_joints(rb, ranges, wanted):
    """Roll / yaw from rnd/swept.py's joint sets (roll-module frame), mapped into each arm by a fit of the module."""
    from . import swept
    from .parts import covers, roll
    out = []
    try:
        J = swept.joints()
    except Exception as e:                                  # the arm CAD is being changed: report, don't crash
        rb.log(f"  swept.joints() failed ({e}); roll / yaw not swept")
        return out
    mod = {n: _centroid(p)[0] for n, p, _ in roll.module()}
    riders = {n for n, _, _ in covers.roll_frame()}         # covers riding on the roll output
    for key, (mv, st, o, d, (lo, hi)) in J.items():
        if wanted and key not in wanted:
            continue
        mnames = {n for n, _ in mv}
        bodies = []
        for side in [s for s in REF if s != "head"]:
            common = [n for n in mod if f"{side}_roll_{n}" in rb.cen]
            A = np.array([mod[n] for n in common]); B = np.array([rb.cen[f"{side}_roll_{n}"][0] for n in common])
            R, t, res = _kabsch(A, B, proper=False)
            if res > 0.5:
                rb.log(f"  {key}/{side}: roll module does not map onto the robot (residual {res:.2f} mm): skipped")
                continue
            members = [f"{side}_roll_{n}" for n in mnames if not n.startswith("arm_") and f"{side}_roll_{n}" in rb.idx]
            if any(n.startswith("arm_") for n in mnames):
                members += [n for n in rb.names if n.startswith(f"{side}_arm_")]
            if key == "roll":
                members += [f"{side}_roll_{n}" for n in riders if f"{side}_roll_{n}" in rb.idx]
            u = R @ np.asarray(d, float); c = R @ np.asarray(o, float) + t
            sense = float(np.linalg.det(R))                 # a mirrored arm turns the other way about the mapped axis
            bodies.append((side, sorted(set(members)), u, c, sense, 0.0))
            rb.log(f"  {key}/{side}: {len(set(members))} parts from rnd/swept.py, fit {res:.3f} mm")
        out.append((key, bodies, (lo, hi)))
    return out


# ---------------------------------------------------------------------------------------------- probe accessibility
def accessible(P, N, test_idx, extra_bound=None):
    """Bool per test_idx: a reachable Ø12 probe centre touches the point from its outward side."""
    from scipy import ndimage
    from scipy.spatial import cKDTree
    lo = P.min(0) - 4 * VOX; hi = P.max(0) + 4 * VOX
    lo[2] = max(lo[2], 0.0)                                                   # the floor
    shape = np.ceil((hi - lo) / VOX).astype(int) + 1
    ijk = np.floor((P - lo) / VOX).astype(int)
    ijk = np.clip(ijk, 0, shape - 1)
    occ = np.zeros(shape, bool)
    occ[ijk[:, 0], ijk[:, 1], ijk[:, 2]] = True
    ball = np.array([[[(i * i + j * j + k * k) <= 4 for k in range(-2, 3)] for j in range(-2, 3)] for i in range(-2, 3)])
    band = ndimage.binary_dilation(occ, ball) & ~occ                          # centres within ≈ 8 mm of a surface
    bi = np.argwhere(band)
    bc = lo + (bi + 0.5) * VOX
    tree = cKDTree(P)
    dist, _ = tree.query(bc, distance_upper_bound=PROBE_R)
    blocked = np.zeros(shape, bool)
    bb = bi[np.isfinite(dist)]
    blocked[bb[:, 0], bb[:, 1], bb[:, 2]] = True
    free = ~occ & ~blocked
    lab, _ = ndimage.label(free)
    edge = np.unique(np.concatenate([lab[0].ravel(), lab[-1].ravel(), lab[:, 0].ravel(), lab[:, -1].ravel(), lab[:, :, -1].ravel()]))
    edge = edge[edge > 0]
    reach = np.isin(lab, edge)
    ri = np.argwhere(reach & band)
    if not len(ri):
        return np.zeros(len(test_idx), bool)
    rc = lo + (ri + 0.5) * VOX
    rt = cKDTree(rc)
    q, nq = P[test_idx], N[test_idx]
    d, j = rt.query(q, k=6, distance_upper_bound=PROBE_R + VOX * 0.87)
    ok = np.zeros(len(q), bool)
    for c in range(d.shape[1]):
        v = np.isfinite(d[:, c])
        jj = np.where(v, j[:, c], 0)
        ok |= v & (((rc[jj] - q) * nq).sum(1) > 0)
    return ok


# ---------------------------------------------------------------------------------------------- sweep
def _steps(lo, hi, step):
    s = list(np.arange(lo, hi, step)) + [hi]
    return [float(v) for v in s]


class Tracker:
    """Per joint: gap history of every (moving cell, static part), probe reach, and exposure of drive parts."""

    def __init__(self, rb):
        self.rb = rb
        self.gaps = {}      # joint -> {(cell key, static part): [list of (pose, gap, reach)]}
        self.expo = {}      # part name -> (max accessible area mm², joint, pose)
        self.sets = {}      # (joint, side) -> names moving with it (the kinematic tree, from the experiments)
        self.deform = set()

    def expose_idx(self):
        rb = self.rb
        out = []
        for i, n in enumerate(rb.names):
            if not COVER.search(n) and any(rx.search(_strip(n)) for rx, _, _ in EXPOSE):
                out.append(np.arange(rb.offs[i], rb.offs[i + 1]))
        return np.concatenate(out) if out else np.zeros(0, int)

    def record_exposure(self, acc_idx, acc, joint, pose):
        rb = self.rb
        ids = rb.I[acc_idx[acc]]
        cnt = np.bincount(ids, minlength=len(rb.names))
        for i in np.nonzero(cnt)[0]:
            a = cnt[i] / DENS
            n = rb.names[i]
            if a > self.expo.get(n, (0,))[0]:
                self.expo[n] = (a, joint, pose)


def run_joint(rb, tr, joint, bodies, angles, quick):
    """Sweep one rotary joint (all sides together) and record gaps + exposure."""
    from scipy.spatial import cKDTree
    mov_ids = sorted({rb.idx[n] for _, mem, *_ in bodies for n in mem})
    if not mov_ids:
        return
    mmask = np.isin(rb.I, mov_ids)
    soft = np.array([bool(SOFT.search(n)) for n in rb.names])
    skip = np.array([bool(SKIP_GAP.search(n)) for n in rb.names]) | soft
    sidx = np.nonzero(~mmask & ~soft[rb.I])[0]
    stree = cKDTree(rb.P[sidx])
    midx = np.nonzero(mmask & ~skip[rb.I])[0]
    cell = np.floor(rb.P[midx] / CELL).astype(np.int64)
    ckey = (rb.I[midx].astype(np.int64) << 42) ^ ((cell[:, 0] & 0x3FFF) << 28) ^ ((cell[:, 1] & 0x3FFF) << 14) ^ (cell[:, 2] & 0x3FFF)
    side_of = {}
    for b, (side, mem, u, c, sense, ref) in enumerate(bodies):
        for n in mem:
            side_of[rb.idx[n]] = b
    eidx = tr.expose_idx()
    hist = tr.gaps.setdefault(joint, {})
    for th in angles:
        P = rb.P.copy(); N = rb.N.copy()
        for b, (side, mem, u, c, sense, ref) in enumerate(bodies):
            sel = np.isin(rb.I, [rb.idx[n] for n in mem])
            R = _rot(u, (th - ref) * sense)
            P[sel] = (rb.P[sel] - c) @ R.T + c
            N[sel] = rb.N[sel] @ R.T
        d, j = stree.query(P[midx], distance_upper_bound=BOUND)
        near = np.isfinite(d)
        sj = sidx[np.where(near, j, 0)]
        test = np.unique(np.concatenate([midx[near], sj[near], eidx]))
        acc_all = accessible(P, N, test)
        acc = dict(zip(test.tolist(), acc_all.tolist()))
        tr.record_exposure(test, acc_all, joint, th)
        am = np.array([acc.get(i, False) for i in midx[near]])
        as_ = np.array([acc.get(i, False) for i in sj[near]])
        k = ckey[near]; spart = rb.I[sj[near]]; dd = d[near]
        order = np.lexsort((dd, k))                         # per cell: the nearest static surface, whichever part
        k, spart, dd, am, as_ = k[order], spart[order], dd[order], am[order], as_[order]
        brk = np.nonzero(np.r_[True, k[1:] != k[:-1]])[0]
        ra = np.logical_or.reduceat(am, brk) & np.logical_or.reduceat(as_, brk) if len(brk) else []
        for b0, r in zip(brk, ra):
            hist.setdefault(int(k[b0]), []).append((th, float(dd[b0]), int(spart[b0]), bool(r)))
        hist.setdefault("_poses", []).append(th)
        rb.log(f"    {joint} {th:7.1f}: {near.sum():,} moving samples within {BOUND:.0f} mm, {int(acc_all.sum()):,} accessible")


def lift_membership(rb, tr, lifts):
    """Rebuild per height: which parts ride the lift (rigid), which change shape or come and go (bellows)."""
    builds, rigid_n, deform_n = {}, set(), set()
    for L in lifts:
        parts = rb.robot.build(L, REF, covers=True)
        builds[L] = {n: p for n, p, _ in parts}
        dz = (L - REF_LIFT) * 1000
        for n, (c, v) in rb.moved(parts).items():
            if n not in rb.cen:
                continue
            c0, v0 = rb.cen[n]
            if abs(v - v0) < 1e-3 * max(v0, 1) and np.allclose(c - c0, (0, 0, dz), atol=0.2):
                rigid_n.add(n)
            else:
                deform_n.add(n)
        deform_n |= {n for n in rb.names if n not in builds[L]}            # present at some heights only
    rigid_n -= deform_n
    tr.sets[("lift", "-")] = set(rigid_n)
    tr.deform = set(deform_n)
    return builds, rigid_n, deform_n


def run_lift(rb, tr, lifts, membership):
    """Prismatic lift: rigid movers translate, deformers are resampled at each height."""
    from scipy.spatial import cKDTree
    hist = tr.gaps.setdefault("lift", {})
    soft = np.array([bool(SOFT.search(n)) for n in rb.names])
    skip = np.array([bool(SKIP_GAP.search(n)) for n in rb.names]) | soft
    builds, rigid_n, deform_n = membership
    rigid = [rb.idx[n] for n in rigid_n]
    for L in lifts:
        dz = (L - REF_LIFT) * 1000
        P = rb.P.copy(); N = rb.N.copy()
        P[np.isin(rb.I, rigid)] += (0, 0, dz)
        dsel = np.isin(rb.I, [rb.idx[n] for n in deform_n])
        newP, newN, newI = [P[~dsel]], [N[~dsel]], [rb.I[~dsel]]
        deform = sorted(n for n in deform_n if n in builds[L])
        for n in deform:                                    # deformers: fresh samples at this height
            Pd, Nd = _sample(builds[L][n], rb.idx[n])
            newP.append(Pd); newN.append(Nd); newI.append(np.full(len(Pd), rb.idx[n]))
        P, N, I = np.concatenate(newP), np.concatenate(newN), np.concatenate(newI)
        movers = np.isin(I, rigid)
        dmask = np.isin(I, [rb.idx[n] for n in deform])
        statm = ~movers & ~dmask
        # pairs: mover vs static, mover vs deformer, deformer vs static (each a "body" against the rest)
        pairs = [(movers, statm | dmask), (dmask, statm)]
        expo_ids = [rb.idx[n] for n in rb.names if not COVER.search(n) and any(rx.search(_strip(n)) for rx, _, _ in EXPOSE)]
        tests = [np.nonzero(np.isin(I, expo_ids))[0]]
        recs = []
        for a, b in pairs:
            ai = np.nonzero(a & ~skip[I])[0]; bi = np.nonzero(b & ~soft[I])[0]
            if not len(ai) or not len(bi):
                continue
            d, j = cKDTree(P[bi]).query(P[ai], distance_upper_bound=BOUND)
            near = np.isfinite(d)
            recs.append((ai[near], bi[np.where(near, j, 0)][near], d[near]))
            tests += [ai[near], bi[np.where(near, j, 0)][near]]
        test = np.unique(np.concatenate(tests))
        acc_all = accessible(P, N, test)
        acc = dict(zip(test.tolist(), acc_all.tolist()))
        ids = I[test[acc_all]]
        cnt = np.bincount(ids, minlength=len(rb.names))
        for i in np.nonzero(cnt)[0]:
            a = cnt[i] / DENS
            if a > tr.expo.get(rb.names[i], (0,))[0]:
                tr.expo[rb.names[i]] = (a, "lift", L)
        for ai, bj, d in recs:
            # cell key from the reference position of rigid movers; deformers: one cell per part
            ref = P[ai] - np.where(np.isin(I[ai], rigid)[:, None], (0, 0, dz), 0)
            cell = np.floor(ref / CELL).astype(np.int64)
            cell[np.isin(I[ai], [rb.idx[n] for n in deform])] = 0
            k = (I[ai].astype(np.int64) << 42) ^ ((cell[:, 0] & 0x3FFF) << 28) ^ ((cell[:, 1] & 0x3FFF) << 14) ^ (cell[:, 2] & 0x3FFF)
            am = np.array([acc.get(i, False) for i in ai]); as_ = np.array([acc.get(i, False) for i in bj])
            sp = I[bj]
            order = np.lexsort((d, k))
            k, sp, d, am, as_ = k[order], sp[order], d[order], am[order], as_[order]
            brk = np.nonzero(np.r_[True, k[1:] != k[:-1]])[0]
            if not len(brk):
                continue
            ra = np.logical_or.reduceat(am, brk) & np.logical_or.reduceat(as_, brk)
            for b0, r in zip(brk, ra):
                hist.setdefault(int(k[b0]), []).append((L, float(d[b0]), int(sp[b0]), bool(r)))
        hist.setdefault("_poses", []).append(L)
        rb.log(f"    lift {L:.2f} m: {len(rigid)} parts ride the lift, deforming: {', '.join(deform) or 'none'}, {int(acc_all.sum()):,} accessible")


# ---------------------------------------------------------------------------------------------- evaluation + report
def _strip(n):
    return n[2:] if n[:2] in ("l_", "r_") else n


def _tree(rb, tr):
    """Kinematic tree from the experiments. For each moving body (joint, side): its parent link = the smallest body
    that strictly contains it, minus itself and minus sibling bodies (the other arm); the root's parent is everything
    that moves with nothing. The child link = the body minus the bodies nested in it."""
    ALL = set(rb.names)
    sets = tr.sets
    parent, child = {}, {}
    for k, M in sets.items():
        sups = [S for S in sets.values() if M < S]
        Sp = min(sups, key=len) if sups else ALL
        sib = set().union(*[S for S in sets.values() if S <= Sp and not (S & M)])
        parent[k] = Sp - M - sib
        child[k] = M - set().union(*[S for S in sets.values() if S < M])
    return parent, child


def evaluate(rb, tr):
    """Rows per location (joint, moving part, static part). Tier 1 (counted, R17): the moving body against its parent
    link (the joint's own nips: elbow fold, shoulder–torso, wrist–forearm, head–neck), and for the lift the carriage
    body against the mast, base, head and bellows. Tier 2 (listed): everything else, e.g. a hand reaching the base
    at an extreme yaw or a hanging arm on the base rim at the lowest lift; those are clamping zones handled by the
    planner's keep-outs and force limiting (D23 (5)), not by cover geometry."""
    parent, child = _tree(rb, tr)
    rows = {}
    for joint, hist in tr.gaps.items():
        poses = hist.get("_poses", [])
        for key, h in hist.items():
            if key == "_poses":
                continue
            mp = key >> 42
            if not any(r for *_, r in h):
                continue                                     # never reachable by the probe
            seen = {}
            for p_, g_, s_, _ in h:                          # lift: movers and deformers are separate passes
                if p_ not in seen or g_ < seen[p_][0]:
                    seen[p_] = (g_, s_)
            g = [seen.get(p_, (BOUND, -1))[0] for p_ in poses]
            mn, mx = min(g), max(g)
            sp = seen.get(poses[int(np.argmin(g))], (BOUND, -1))[1]
            if mn >= OPEN:
                continue
            shear = (mx - mn) < SHEAR_TOL and not (mx <= CLOSED or mn >= OPEN)
            ok = mx <= CLOSED or mn >= OPEN or shear
            a, b = rb.names[mp], rb.names[sp]
            bodies = [k for k in tr.sets if k[0] == joint and a in tr.sets[k]]
            if a in tr.deform:
                tier = 1
            elif bodies:
                k = bodies[0]
                tier = 1 if (b in parent[k] or b in tr.deform) and (joint != "lift" or a in child[k]) else 2
            else:
                tier = 2
            loc = (joint, _strip(a), _strip(b), tier)
            side = a[:1] if a[:2] in ("l_", "r_") else "-"
            cur = rows.get(loc)
            if cur is None or (not ok and cur["ok"]) or (ok == cur["ok"] and mn < cur["min"]):
                rows[loc] = dict(joint=joint, moving=loc[1], static=loc[2], tier=tier, min=mn, max=mx, ok=ok, shear=shear,
                                 at=poses[int(np.argmin(g))], sides=set((cur or {}).get("sides", set())) | {side})
            else:
                cur["sides"].add(side)
    gaps = sorted(rows.values(), key=lambda r: (r["tier"], r["ok"], r["joint"], r["min"]))
    expo = []
    for n, (a, j, p) in sorted(tr.expo.items(), key=lambda kv: -kv[1][0]):
        if COVER.search(n):
            continue
        for rx, kind, hard in EXPOSE:
            if rx.search(_strip(n)):
                if a >= MIN_PATCH:
                    expo.append(dict(part=n, kind=kind, area=a, joint=j, at=p, ok=not hard, hard=hard))
                break
    return gaps, expo


def _gap_table(rows):
    L = ["| Joint | Moving part | Static part | Sides | Min gap (mm) | Max gap (mm) | Min at | |", "|---|---|---|---|---|---|---|---|"]
    for r in rows:
        mx = "> 40" if r["max"] >= BOUND else f"{r['max']:.1f}"
        L.append(f"| {r['joint']} | {r['moving']} | {r['static']} | {'/'.join(sorted(r['sides']))} | {r['min']:.1f} | {mx} | {r['at']:.4g} | {'⚠️ shear' if r.get('shear') else ('✅' if r['ok'] else '❌')} |")
    return L if rows else L + ["| none | | | | | | | ✅ |"]


def report(rb, tr, joints_done, skipped, t_run, quick):
    gaps, expo = evaluate(rb, tr)
    t1 = [r for r in gaps if r["tier"] == 1]
    t2 = [r for r in gaps if r["tier"] == 2]
    gf = [r for r in t1 if not r["ok"]]
    ef = [r for r in expo if r["hard"]]
    L = ["# Pinch, shear and draw-in check (generated by `python -m rnd.pinch`)", "",
         __doc__.split("\n\n", 1)[1].strip(), "", "**Tiers.** " + " ".join(evaluate.__doc__[evaluate.__doc__.index("Tier 1"):].split()), "",
         f"**Result: {len(gf)} joint-nip gap fails, {len(ef)} exposed belt / pulley / motor parts. R17 needs 0 and 0.** "
         f"Also listed: {sum(1 for r in t2 if not r['ok'])} reach-clamping zones (tier 2) and "
         f"{sum(1 for r in expo if not r['hard'])} exposed shafts (warnings).", "",
         f"Swept: {', '.join(joints_done)}" + (f"; not swept: {', '.join(skipped)}" if skipped else "")
         + f". Reference pose {json.dumps(REF)} at lift {REF_LIFT} m. {'Quick (coarse) steps. ' if quick else ''}Run time {t_run / 60:.1f} min.", "",
         "## Exposed drive parts (accessible to the Ø12 probe at some step)", "",
         "| Part | Kind | Largest exposed patch (mm²) | Worst at | |", "|---|---|---|---|---|"]
    L += [f"| {r['part']} | {r['kind']} | {r['area']:.0f} | {r['joint']} {r['at']:.4g} | {'❌' if r['hard'] else '⚠️ shaft'} |" for r in expo]
    if not expo:
        L.append("| none | | | | ✅ |")
    L += ["", "## Compliant covers (TPU 95A boots; rule in item 4 above)", "",
          f"Wall ≥ {TPU_WALL_MIN} mm and every free span within ±{TPU_STRAIN:.0%} of its as-printed length over the joint's range. "
          "A boot that fails is taken out of the model; a design study (no part) shows what a boot there would need.", "",
          "| Cover | Part | Joint (range) | Wall (mm) | Worst span | Free (mm) | Min … max (mm) | Compression / extension | | Note |",
          "|---|---|---|---|---|---|---|---|---|---|"]
    for r in rb.compliant:
        L.append(f"| {r['name']} | {r['part'] or 'study'} | {r['joint']} ({r['range'][0]:g}…{r['range'][1]:g}°) | {r['wall']:.1f} | {r['span']} | "
                 f"{r['free']:.0f} | {r['min']:.0f} … {r['max']:.0f} | {r['comp']:.0%} / {r['ext']:.0%} | "
                 f"{'✅ closes' if r['ok'] else ('❌ taken out' if r['part'] else '❌ not buildable')} | {r['note']} |")
    if not rb.compliant:
        L.append("| none | | | | | | | | | |")
    if rb.unjudged:
        L += ["", f"Soft parts with no declared spans (kept as drawn, not judged): {', '.join(rb.unjudged)}."]
    L += ["", "## Tier 1: joint nips (reachable, < 25 mm at some step; counted)", "",
          "Min/max over the sweep for the worst 20 mm cell of the moving part against the nearest part of the parent link.", ""]
    L += _gap_table(t1)
    L += ["", "## Tier 2: clamping at reach (listed; planner keep-outs + force limit, D23 (5))", ""]
    L += _gap_table([r for r in t2 if not r["ok"]])
    (DOCS / "pinch.md").write_text("\n".join(L) + "\n")
    js = {"gap_fails": len(gf), "exposed_fails": len(ef), "fails": len(gf) + len(ef), "reach_zones": sum(1 for r in t2 if not r["ok"]),
          "joints": joints_done, "skipped": skipped, "quick": quick,
          "rule": {"closed_mm": CLOSED, "open_mm": OPEN, "probe_d_mm": 2 * PROBE_R},
          "gaps": [{**r, "sides": sorted(r["sides"])} for r in gaps], "exposed": expo,
          "compliant": rb.compliant, "compliant_rule": {"wall_min_mm": TPU_WALL_MIN, "strain": TPU_STRAIN},
          "dropped": rb.dropped, "soft_unjudged": rb.unjudged}
    (DOCS / "pinch.json").write_text(json.dumps(js, indent=1, default=float))
    return L, len(gf), len(ef)


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true", help="coarse steps (about 2× faster)")
    ap.add_argument("--joints", default="", help="comma list to sweep: pitch,elbow,wrist,roll,yaw,pan,tilt,lift")
    a = ap.parse_args(argv)
    wanted = {j for j in a.joints.split(",") if j}
    t0 = time.time()
    log = lambda s: print(s, flush=True)
    ranges = _ranges()
    step = {"pitch": 20, "elbow": 15, "wrist": 20, "roll": 15, "yaw": 30, "pan": 30, "tilt": 10}
    if a.quick:
        step = {k: v * 2 for k, v in step.items()}
    from .params import BASELINE
    u = BASELINE.upper
    lifts = [u.lift_min, REF_LIFT, u.lift_max] if a.quick else [u.lift_min, (u.lift_min + REF_LIFT) / 2, REF_LIFT, (REF_LIFT + u.lift_max) / 2, u.lift_max]
    with _Memo():
        rb = Robot(log)
        tr = Tracker(rb)
        rot, skipped = rotary_joints(rb, ranges, None)              # the whole tree, even when sweeping a subset
        rot += swept_joints(rb, ranges, None)
        for key, bodies, _ in rot:
            for side, mem, *_ in bodies:
                tr.sets[(key, side)] = set(mem)
        memb = lift_membership(rb, tr, lifts)
        done = []
        for key, bodies, (lo, hi) in rot:
            if not bodies:
                skipped.append(key)
                continue
            if wanted and key not in wanted:
                continue
            log(f"  sweep {key} {lo}…{hi}")
            run_joint(rb, tr, key, bodies, _steps(lo, hi, step[key]), a.quick)
            done.append(key)
        if not wanted or "lift" in wanted:
            log("  sweep lift")
            run_lift(rb, tr, lifts, memb)
            done.append("lift")
    L, ng, ne = report(rb, tr, done, skipped, time.time() - t0, a.quick)
    print("\n".join(L[L.index("## Exposed drive parts (accessible to the Ø12 probe at some step)"):]))
    print(f"\npinch: {ng} joint-nip gap fails, {ne} exposed drive parts")
    return ng + ne


if __name__ == "__main__":
    sys.exit(0 if main() == 0 else 1)
