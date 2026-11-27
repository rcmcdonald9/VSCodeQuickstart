"""R5 door stow pose (both arms).  python -m rnd.stow [--quick]  →  docs/rnd/stow.md + stow.json

R5 asks the robot to "fit 70 cm doors with arms folded". Folded, the arms span 0.70–0.78 m (rnd/tip.py note). This
searches the twin (rnd/twin.py: CAD collision hulls, rnd/parts/collision.py) for a **door stow pose**: both arms and
the lift, collision-free, inside the supervisor's target bounds in **both** modes (joint limits less the 3° margin;
shoulder roll ≥ +5° + 3° with people near, D25, sw/safety.py), that makes the whole robot (arms + cover allowance,
torso, base, head, mast) narrow enough for the door.

Width is measured **across the travel direction**: the base is holonomic (4 omni wheels at 0/90/180/270°), so the
robot need not face the door. W(ψ) = extent of the plan view along the normal of a travel heading ψ in the robot frame
(ψ = 0: driving forward). Thresholds (R4, ≤ 15–20 mm) must be crossed at ψ = 45° (D2b, rnd/threshold.py: straight
at a wheel fails at 15 mm), so the governing case is **W(45°)**; W(90°) (sideways) is for doors with no threshold.
The stow found for 45° is chiral: its mirror image (left ↔ right) is the stow for ψ = 135°.

Checks in the pose: cross-body gaps (arm vs torso / base / head / mast / the other arm, and non-adjacent links of the
same arm: the two-joint combinations rnd/pinch.py does not sweep) ≥ 25 mm after the cover allowance (R17 finger
gap), on the move into the stow from the default park pose too; hands ≥ 50 mm above the floor (thresholds); ZMP
reserve (rnd/tip.py reserve(), asymmetric pose) for the threshold crossing (casters released, 1.0 m/s² braking) and
with a person near (casters locked, 0.1 m/s² + 30 N push at 1 m); static (R12 ≥ 50 mm).
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"

# ---- the door (R5 gives "70 cm doors" and no clear width or source; RND_PLAN §2)
# Read as a standard interior door of the 70 cm class: DIN 18101 nominal 750 (leaf 735 × 1985) has ≈ 688 mm between the
# frame jambs (sanier.de, "Türen richtig planen"; baunetzwissen.de "Türblattgrößen nach DIN 18101"). Clear width the way
# ADA 2010 §404.2.3 measures it (face of the door open 90° to the stop) loses the leaf thickness (≈ 40 mm, ESTIMATE):
# ≈ 650 mm. US 28" (711 mm) leaves give ≈ 25.5–26" (650–660 mm) the same way. The literal 700 mm is reported too.
DOOR_LEAF = 0.735
DOOR_FRAME_CLEAR = 0.688
CLEAR = 0.650                    # R5 clear width used (ESTIMATE, see above)
CLEAR_LITERAL = 0.700
MARGIN = 0.040                   # each side: approach error of the base planner (lidar ± a few cm) + sway (ESTIMATE)
W_MAX = CLEAR - 2 * MARGIN       # 0.570 m
DOOR_H = 1.97                    # DIN 18101 leaf 1985 → ≈ 1.97 m clear under the head jamb (ESTIMATE)

# ---- R17 / pinch
GAP_OPEN = 0.025                 # finger-crush gap (ISO 13854 finger 25 mm, commonly cited; R17)
FLOOR_MIN = 0.050                # hands and arm parts ≥ 50 mm above the floor (15–20 mm thresholds, R4)

# ---- cover allowance on the twin's hulls (they model the CAD bodies, not the D23 covers): rigid covers are WALL 2.2 mm
# at ≈ 3 mm off their part (rnd/parts/covers.py, yaw hood convention) → 5.2 mm; the wrist boot stands 15 mm past the wrist
# cartridge on its outboard side (soft_shell y span WRIST_Y1 − WRIST_Y0 + 22, centred +4) → 15 mm on that hull. The
# inline roll's motor has no cover drawn yet (R17 note in covers.py): the 5.2 mm stands in for its dome (ESTIMATE).
PAD = 0.0052
PAD_GEOM = {"fore_wrist": 0.015}

ARM_J = ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")
MIRROR = {"pitch": 1, "roll": 1, "yaw": -1, "elbow": 1, "wpitch": 1, "wroll": -1}   # left = mirror of right (rnd/tip.py)
GATE = "cross"                   # pair class gated at GAP_OPEN in the search (None: all pairs, incl. roll body × torso)
SV_MARGIN = 3.0                  # deg, sw/safety.py Supervisor margin_deg


SH_Y0 = 0.200                    # the shoulder_y the cached collision hulls were exported for (rnd/out/collision_*)


def design_xml(d, build=None):
    """twin.build_xml(d), with the torso narrowed to follow a smaller shoulder_y: the torso-side pitch-module hulls are
    exported once for the baseline (rnd/parts/collision.py, shared rnd/out cache, not re-exported here), so they are shifted
    in by the same amount, and the torso box (390 mm shell) narrows with them."""
    from . import twin
    x = (build or twin.build_xml)(d)
    dy = SH_Y0 - d.arm.shoulder_y
    if abs(dy) < 1e-9:
        return x
    for s, sg in (("l", -1), ("r", 1)):
        x = x.replace(f'mesh="torso_pitchmod_{s}"', f'mesh="torso_pitchmod_{s}" pos="0 {sg * dy} 0"')
    return x.replace('size="0.115 0.195 0.105"', f'size="0.115 {0.195 - dy} 0.105"')


def r1_outer(d, ys=(-0.40, -0.325)):
    """R1 coverage (rnd/twin.ik_study, 2 random seeds) on the outer y columns only, where moving the shoulders in costs reach."""
    from . import twin
    orig = twin.build_xml
    twin.build_xml = lambda dd, geometry=None: design_xml(dd, orig) if geometry is None else orig(dd, geometry)
    try:
        r = twin.ik_study(d, {"x": twin.support_polygon("xdrive", d.base.wheel_radius_pos, 0)}, ys=np.array(ys), seeds=2)
    finally:
        twin.build_xml = orig
    return {k: v["coverage"] for k, v in r.items() if not v.get("info")}


class Twin:
    """The twin model with both arms posable independently. q (deg, m) = [lift, right arm 6, left arm 6], the left arm in
    the right arm's convention (equal values = mirror-symmetric pose)."""

    def __init__(self, d=None):
        import mujoco
        from .params import BASELINE
        self.mj = mujoco
        self.design = d or BASELINE
        m = self.m = mujoco.MjModel.from_xml_string(design_xml(self.design))
        self.d = mujoco.MjData(m)
        bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
        for s in "rl":
            m.body_mass[bid(f"payload_{s}")] = 1e-4
        self.jn = ["lift"] + [f"{j}_{s}" for s in "rl" for j in ARM_J]
        jid = [mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n) for n in self.jn]
        self.adr = np.array([m.jnt_qposadr[i] for i in jid])
        self.sign = np.array([1.0] + [1.0] * 6 + [MIRROR[j] for j in ARM_J])
        # bounds (deg / m): the twin's joint ranges ∩ sw/description/joints.json, less the supervisor margin
        J = {j["name"]: j for j in json.loads((Path(__file__).resolve().parent.parent / "sw" / "description" / "joints.json").read_text())}
        lo, hi, pn_lo = [], [], []
        for n, i in zip(self.jn, jid):
            r = m.jnt_range[i]
            if n == "lift":
                lo.append(r[0]); hi.append(r[1]); pn_lo.append(r[0]); continue
            j = n[:-2]
            a, b = math.degrees(r[0]), math.degrees(r[1])
            if j in J:
                a, b = max(a, math.degrees(J[j]["lower"])), min(b, math.degrees(J[j]["upper"]))
            pn = J.get(j, {}).get("people_near")
            if n.endswith("_l") and MIRROR[j] < 0:
                a, b = -b, -a
            lo.append(a + SV_MARGIN); hi.append(b - SV_MARGIN)
            pn_lo.append(max(a, math.degrees(pn[0])) + SV_MARGIN if pn else a + SV_MARGIN)
        self.lo, self.hi, self.pn_lo = np.array(lo), np.array(hi), np.array(pn_lo)
        # geometry: collision-relevant geoms of the robot (CAD hulls, torso / head / base / mast primitives); the twin's
        # joint spheres are mass placeholders inside the hulls and the hand box / bone capsules are group 3 (hulls instead)
        T = mujoco.mjtGeom
        robot_root = bid("robot")
        self.arm_side = {}
        for s in "rl":
            sh = bid(f"sh_{s}")
            for b in range(m.nbody):
                p = b
                while p not in (0, sh):
                    p = m.body_parentid[p]
                if p == sh:
                    self.arm_side[b] = s
        pay = {bid("payload_r"), bid("payload_l")}
        self.geoms, self.pad, self.gname = [], [], {}
        for g in range(m.ngeom):
            b = m.geom_bodyid[g]
            if b == 0 or b in pay or m.geom_group[g] == 3 or m.geom_type[g] == T.mjGEOM_SPHERE or m.geom_rgba[g][3] == 0:
                continue
            nm = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_MESH, m.geom_dataid[g]) if m.geom_type[g] == T.mjGEOM_MESH else None
            if m.geom_type[g] == T.mjGEOM_BOX and m.geom_size[g][0] < 0.015:      # the head's screen bezel plate: inside the head box ± 1 cm
                pass
            self.geoms.append(g)
            key = (nm or "")[:-2]
            self.pad.append((PAD_GEOM.get(key, PAD)) if b in self.arm_side else 0.0)
            self.gname[g] = nm or f"{mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b)}:{['plane','hfield','sphere','capsule','ellipsoid','cylinder','box','mesh'][m.geom_type[g]]}{g}"
        self.pad = np.array(self.pad)
        self._local = [self._local_pts(g) for g in self.geoms]
        # gap pairs: every arm geom against every geom of a body that is neither its own body nor its parent / child
        par = m.body_parentid
        self.pairs = []
        for i, g1 in enumerate(self.geoms):
            b1 = m.geom_bodyid[g1]
            if b1 not in self.arm_side:
                continue
            for k, g2 in enumerate(self.geoms):
                b2 = m.geom_bodyid[g2]
                if b2 == b1 or par[b1] == b2 or par[b2] == b1 or (b2 in self.arm_side and k <= i and self.arm_side[b2] == self.arm_side[b1]):
                    continue
                if b2 in self.arm_side and self.arm_side[b2] != self.arm_side[b1] and k < i:
                    continue
                if b2 not in self.arm_side and b2 != robot_root and b2 != bid("torso"):
                    continue
                # roll body × torso (torso shell / torso-side pitch module): a pitch + roll pair that rnd/pinch.py sweeps
                # (its roll rows "roll_* × torso_shell"); kept apart so it can be reported against pinch.py, not gated twice
                cls = "shoulder" if (mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_BODY, b1).startswith("roll_") and b2 == bid("torso")) else "cross"
                self.pairs.append((g1, g2, self.pad[i] + self.pad[k], cls))
        self.robot = robot_root

    def _local_pts(self, g):
        m, T = self.m, self.mj.mjtGeom
        t, s = m.geom_type[g], m.geom_size[g]
        if t == T.mjGEOM_MESH:
            i = m.geom_dataid[g]
            return m.mesh_vert[m.mesh_vertadr[i]: m.mesh_vertadr[i] + m.mesh_vertnum[i]].astype(float)
        if t == T.mjGEOM_BOX:
            return np.array([[a * s[0], b * s[1], c * s[2]] for a in (-1, 1) for b in (-1, 1) for c in (-1, 1)])
        if t == T.mjGEOM_CYLINDER:
            ang = np.linspace(0, 2 * math.pi, 72, endpoint=False)
            return np.array([[s[0] * math.cos(a), s[0] * math.sin(a), h * s[1]] for a in ang for h in (-1, 1)])
        if t == T.mjGEOM_CAPSULE:
            return np.array([[0, 0, -s[1]], [0, 0, s[1]]])      # + radius as pad (not used: capsules are group 3)
        return np.zeros((1, 3))

    # ---------------------------------------------------------------- pose
    def set(self, q):
        q = np.asarray(q, float)
        v = q * self.sign
        v[1:] = np.radians(v[1:])
        self.d.qpos[self.adr] = v
        self.mj.mj_kinematics(self.m, self.d)

    def points(self):
        """World points per geom and their pads."""
        out = []
        for g, P in zip(self.geoms, self._local):
            out.append(self.d.geom_xpos[g] + P @ self.d.geom_xmat[g].reshape(3, 3).T)
        return out

    def widths(self, psis):
        pts = self.points()
        res = []
        for psi in np.atleast_1d(psis):
            n = np.array([-math.sin(math.radians(psi)), math.cos(math.radians(psi))])
            hi = max(float((P[:, :2] @ n).max()) + p for P, p in zip(pts, self.pad))
            lo = min(float((P[:, :2] @ n).min()) - p for P, p in zip(pts, self.pad))
            res.append(hi - lo)
        return np.array(res)

    def extremes(self, psi):
        """(geom name, side value) of the two parts that set W(psi)."""
        pts = self.points()
        n = np.array([-math.sin(math.radians(psi)), math.cos(math.radians(psi))])
        hi = [(float((P[:, :2] @ n).max()) + p, self.gname[g]) for g, P, p in zip(self.geoms, pts, self.pad)]
        lo = [(float((P[:, :2] @ n).min()) - p, self.gname[g]) for g, P, p in zip(self.geoms, pts, self.pad)]
        return max(hi), min(lo)

    def arm_zmin(self):
        pts = self.points()
        return min(float(P[:, 2].min()) - p for g, P, p in zip(self.geoms, pts, self.pad) if self.m.geom_bodyid[g] in self.arm_side)

    def zmax(self):
        return max(float(P[:, 2].max()) + p for P, p in zip(self.points(), self.pad))

    def gaps(self, distmax=0.06, cls=None):
        """Smallest gaps after the cover allowance: [(gap m, geom a, geom b, class)], sorted. cls: only that pair class."""
        out = []
        ft = np.zeros(6)
        for g1, g2, pad, c in self.pairs:
            if cls and c != cls:
                continue
            dist = self.mj.mj_geomDistance(self.m, self.d, g1, g2, distmax, ft)
            if dist < distmax:
                out.append((float(dist) - pad, self.gname[g1], self.gname[g2], c))
        return sorted(out)

    def min_gap(self, distmax=0.06, cls="gate"):
        g = self.gaps(distmax, GATE if cls == "gate" else cls)
        return g[0][0] if g else distmax

    def com(self):
        m, d = self.m, self.d
        w = m.body_mass[1:]
        M = float(w.sum())
        return M, (w[:, None] * d.xipos[1:]).sum(axis=0) / M


# -------------------------------------------------------------------------------------------- search
def cost(T, q, psi, both_modes=True):
    lo = T.pn_lo if both_modes else T.lo
    q = np.clip(q, lo, T.hi)
    T.set(q)
    w = T.widths([psi])[0]
    g = T.min_gap()
    zf = T.arm_zmin()
    pen = 20.0 * max(0.0, GAP_OPEN - g) + 20.0 * max(0.0, FLOOR_MIN - zf)
    M, c = T.com()
    return w + pen + 0.02 * float(np.linalg.norm(c[:2])) + 0.002 * q[0], (w, g, zf)


SEEDS = {   # [lift, right pitch roll yaw elbow wpitch wroll, left ...] deg, right-arm convention for both
    "hanging": [0.80, 0, 10, 0, -5, 0, 0, 0, 10, 0, -5, 0, 0],
    "tuck (twin.TUCK)": [0.80, 0, 10, 0, -140, 60, 0, 0, 10, 0, -140, 60, 0],
    "left fwd, right back": [0.80, 40, 10, 0, -60, 0, 0, -90, 10, 0, -90, 0, 0],
    "left fwd folded in, right back": [0.80, 50, 10, 60, -30, 0, 0, -90, 10, -90, -110, 0, 0],
    "both fwd folded in": [0.80, -90, 10, 90, -110, 0, 0, -90, 10, -90, -110, 0, 0],
    "right fwd, left back": [0.80, -90, 10, 0, -90, 0, 0, 40, 10, 0, -60, 0, 0],
    "both back": [0.80, 45, 10, 0, -40, 0, 0, 45, 10, 0, -40, 0, 0],
}


def search(T, psi, both_modes=True, quick=False, seeds=None):
    from scipy.optimize import minimize
    lo = T.pn_lo if both_modes else T.lo
    best = None
    for name, s in (seeds or SEEDS).items():
        x0 = np.clip(np.array(s, float), lo, T.hi)
        f = lambda x: cost(T, x, psi, both_modes)[0]
        r = minimize(f, x0, method="Powell", bounds=list(zip(lo, T.hi)),
                     options=dict(maxfev=600 if quick else 2500, xtol=0.2, ftol=1e-5))
        x = np.clip(r.x, lo, T.hi)
        c, (w, g, zf) = cost(T, x, psi, both_modes)
        ok = g >= GAP_OPEN - 1e-4 and zf >= FLOOR_MIN - 1e-4
        if best is None or (ok, -w) > (best["ok"], -best["w"]):
            best = dict(seed=name, q=x, w=w, gap=g, zf=zf, ok=ok)
    return best


def approach(T, q0, q1, n=40):
    """Joint-space straight line from q0 to q1: smallest cross-body gap and floor clearance on the way."""
    worst_g, worst_z, at = 9.0, 9.0, 0.0
    for t in np.linspace(0, 1, n + 1):
        T.set((1 - t) * np.asarray(q0) + t * np.asarray(q1))
        g, z = T.min_gap(), T.arm_zmin()
        if g < worst_g:
            worst_g, at = g, t
        worst_z = min(worst_z, z)
    return worst_g, worst_z, at


def staged(T, q0, q1, order):
    """Move joint groups one after the other (order: lists of indices into q); worst gap over all stages."""
    cur = np.array(q0, float)
    worst = (9.0, 9.0)
    for grp in order:
        nxt = cur.copy()
        nxt[grp] = np.asarray(q1)[grp]
        g, z, _ = approach(T, cur, nxt, 20)
        worst = (min(worst[0], g), min(worst[1], z))
        cur = nxt
    return worst


def tip_cases(T, q):
    """ZMP reserve (rnd/tip.py reserve(), worst heading) with the robot in the stow pose, no load."""
    from . import tip
    T.set(q)
    M, c = T.com()
    d = T.design
    cas = tip.casters(d, "D18b")
    rows = []
    for name, cs, a, push in (("static, wheels only (R12: ≥ 50 mm)", None, 0.0, False),
                              ("threshold crossing (D2b), casters released, braking 1.0 m/s²", None, 1.0, False),
                              ("person near: casters locked, braking 0.1 m/s² + 30 N push at 1 m", cas, tip.A_PERSON, True)):
        D = c[2] * a / tip.G + (tip.F_PUSH * tip.H_PUSH / (M * tip.G) if push else 0.0)
        r, at = tip.reserve(d, cs, c[:2], c[2], D)
        rows.append(dict(case=name, reserve=float(r), at=float(at), need=0.05 if "static" in name else tip.NEED))
    return dict(M=M, com=[float(v) for v in c], rows=rows)


def pinch_rows(q):
    """rnd/pinch.py single-joint sweep failures (docs/rnd/pinch.json) whose worst angle is near the stow's angle."""
    f = DOCS / "pinch.json"
    if not f.exists():
        return None
    P = json.loads(f.read_text())
    out = []
    ang = {"pitch": (q[1], q[7]), "roll": (q[2], q[8]), "yaw": (q[3], -q[9]), "elbow": (q[4], q[10]), "wrist": (q[5], q[11])}
    for gsrc, rows in (("all", P.get("gaps", [])), ("people-near", P.get("people_near", {}).get("gaps", []))):
        for r in rows:
            if r.get("ok") or r["joint"] not in ang or r.get("tier") == 2:
                continue
            near = min(abs(a - r["at"]) for a in ang[r["joint"]])
            if near <= 15.0:
                out.append(dict(src=gsrc, joint=r["joint"], moving=r["moving"], static=r["static"], min=r["min"], at=r["at"], stow=near))
    return out


def shoulder_floor(T):
    """The pose-independent part of W(0°): the pitch body (roll motor, roll link, spring ring) only turns about Y, so its
    |y| never changes; with the cover allowance it is the floor on the forward width."""
    m = T.m
    ys = []
    for g, P, p in zip(T.geoms, T.points(), T.pad):
        b = m.geom_bodyid[g]
        if T.mj.mj_id2name(m, T.mj.mjtObj.mjOBJ_BODY, b).startswith("sh_"):
            ys.append(float(np.abs(P[:, 1]).max()) + p)
    return 2 * max(ys)


def fmt_q(q):
    r = dict(zip(ARM_J, q[1:7])); l = dict(zip(ARM_J, q[7:13]))
    return r, l


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--quick", action="store_true")
    a = ap.parse_args(argv)
    from .params import BASELINE
    from .twin import TUCK, TUCKS
    T = Twin(BASELINE)
    psis = np.arange(0.0, 180.0, 1.0)

    # 1. today's park poses (twin.TUCKS, mirror-symmetric) at mid lift
    base_rows = []
    for i, tk in enumerate(TUCKS):
        v = [math.degrees(tk[j]) for j in ARM_J]
        q = np.array([0.70] + v + v)
        T.set(q)
        W = T.widths(psis)
        base_rows.append(dict(name="twin.TUCK" if i == 0 else f"twin.TUCKS[{i}]", q=q.tolist(), w0=float(W[0]), w45=float(W[45]), w90=float(W[90]),
                              wmin=float(W.min()), at=float(psis[W.argmin()]), gap=T.min_gap()))
    floor0 = shoulder_floor(T)

    # 2. the search: ψ = 45° (threshold-legal, governing), 90° (sideways, no threshold), 0° (forward, for the record)
    res = {}
    for psi in (45.0, 90.0, 0.0):
        b = search(T, psi, True, a.quick)
        T.set(b["q"])
        W = T.widths(psis)
        (hv, hn), (lv, ln) = T.extremes(psi)
        res[psi] = dict(b, q=b["q"].tolist(), W=W.tolist(), extremes=[[hn, hv], [ln, lv]], zmax=T.zmax(), gaps=T.gaps()[:6])
    # normal-mode only (roll down to −12°) at 45°, to show what the people-near bound costs
    bn = search(T, 45.0, False, True, {k: SEEDS[k] for k in ("left fwd, right back", "left fwd folded in, right back")})
    res["45n"] = dict(bn, q=bn["q"].tolist())

    S = res[45.0]
    q = np.array(S["q"])
    tk = np.array(base_rows[0]["q"])
    ap_lin = approach(T, tk, q)
    order = [[0], [1, 7], [2, 8], [3, 9], [4, 10], [5, 6, 11, 12]]     # lift, pitch, roll, yaw, elbow, wrist
    order2 = [[0], [4, 10], [3, 9], [1, 7], [2, 8], [5, 6, 11, 12]]
    st = min(staged(T, tk, q, order), staged(T, tk, q, order2), key=lambda w: -w[0])
    tipr = tip_cases(T, q)
    pin = pinch_rows(q)
    mirror = np.concatenate([[q[0]], q[7:13], q[1:7]])
    T.set(mirror)
    w135 = float(T.widths([135.0])[0])

    S["W_band"] = {str(k): float(S["W"][int(k)]) for k in (35, 40, 45, 50, 55)}
    T.set(q)
    shoulder_gaps = dict(stow=[list(x) for x in T.gaps(cls="shoulder")[:1]])
    T.set(tk)
    shoulder_gaps["tuck"] = [list(x) for x in T.gaps(cls="shoulder")[:1]]

    # 3. geometry: the forward floor, the shoulder_y it would take, and a narrower shoulder re-searched at 45° + its R1 cost
    import dataclasses
    need_dy = (floor0 - W_MAX) / 2
    geo = [dict(shoulder_y=BASELINE.arm.shoulder_y, floor_w0=floor0, w45=S["w"], ok=S["ok"], r1=r1_outer(BASELINE))]
    for sy in (0.175,):
        dd = dataclasses.replace(BASELINE, arm=dataclasses.replace(BASELINE.arm, shoulder_y=sy))
        T2 = Twin(dd)
        T2.set(np.array(SEEDS["hanging"]))
        b2 = search(T2, 45.0, True, True, {k: SEEDS[k] for k in ("tuck (twin.TUCK)", "left fwd, right back")})
        geo.append(dict(shoulder_y=sy, floor_w0=shoulder_floor(T2), w45=b2["w"], ok=b2["ok"], r1=r1_outer(dd)))
    ok45 = S["ok"] and S["w"] <= W_MAX
    out = dict(clear=CLEAR, clear_literal=CLEAR_LITERAL, frame_clear=DOOR_FRAME_CLEAR, leaf=DOOR_LEAF, margin=MARGIN, w_max=W_MAX,
               door_h=DOOR_H, pad=PAD, gap_open=GAP_OPEN, floor_min=FLOOR_MIN, shoulder_y=BASELINE.arm.shoulder_y,
               roll_offset_y=BASELINE.arm.roll_offset_y,
               baseline=base_rows, shoulder_floor_w0=floor0, need_dy_fwd=need_dy, geometry=geo,
               stow={str(k): {kk: vv for kk, vv in v.items() if kk != "W"} for k, v in res.items()},
               W_curve={str(k): v["W"] for k, v in res.items() if "W" in v},
               approach_linear=dict(gap=ap_lin[0], zmin=ap_lin[1], at=ap_lin[2]), approach_staged=dict(gap=st[0], zmin=st[1]),
               shoulder_gaps=shoulder_gaps, tip=tipr, pinch_near=pin, mirror_q=mirror.tolist(), w135_mirror=w135, ok=bool(ok45))
    (DOCS / "stow.json").write_text(json.dumps(out, indent=1, default=float))
    (DOCS / "stow.md").write_text(report(out))
    print(f"stow: W(45°) {S['w'] * 1000:.0f} mm (≤ {W_MAX * 1000:.0f}), gap {S['gap'] * 1000:.0f} mm, ok={ok45}")
    return out


def scorecard_rows(path=None):
    """Rows for rnd/run_all.py's scorecard: [(label, value, target, pass)]."""
    f = Path(path or DOCS / "stow.json")
    lbl = ("R5 door stow (rnd/stow.py): whole-robot width across a 45° threshold crossing (D2b), both arms stowed, "
           "collision-free, cross-body gaps ≥ 25 mm, valid with people near (roll ≥ +8°)")
    if not f.exists():
        return [(lbl, "not run (`python -m rnd.stow`)", "≤ 570 mm, tip ≥ 30 mm", False)]
    o = json.loads(f.read_text())
    s = o["stow"]["45.0"]
    rows = [r for r in o["tip"]["rows"] if "static" not in r["case"]]
    tip_ok = all(r["reserve"] >= r["need"] for r in o["tip"]["rows"])
    return [(lbl, f"{s['w'] * 1000:.0f} mm (clear {o['clear'] * 1000:.0f} − 2 × {o['margin'] * 1000:.0f}), tip ≥ "
             f"{min(r['reserve'] for r in rows) * 1000:.0f} mm", f"≤ {o['w_max'] * 1000:.0f} mm, tip ≥ 30 mm", bool(o["ok"] and tip_ok))]


def _deg(v):
    return f"{v:+.0f}°"


def report(o):
    S, S90, S0, Sn = o["stow"]["45.0"], o["stow"]["90.0"], o["stow"]["0.0"], o["stow"]["45n"]
    q = S["q"]
    mm = lambda v: f"{v * 1000:.0f}"
    ok = lambda b: "✅" if b else "❌"
    tip_min = min(r["reserve"] for r in o["tip"]["rows"] if "static" not in r["case"])
    L = ["# Door stow pose, R5 (generated by `python -m rnd.stow`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Result: {'a door stow exists' if o['ok'] else 'no stow fits'}.** Driving through at the D2b threshold heading (45° to "
         f"the wheels) the stowed robot is **{mm(S['w'])} mm** wide (≤ {mm(o['w_max'])} mm = {mm(o['clear'])} mm clear − 2 × "
         f"{mm(o['margin'])} mm: {mm((o['clear'] - S['w']) / 2)} mm a side in a {mm(o['clear'])} mm opening, "
         f"{mm((o['clear_literal'] - S['w']) / 2)} mm in a literal 700 mm one), collision-free with every cross-body gap ≥ "
         f"{mm(S['gap'])} mm after the cover allowance, arm parts ≥ {mm(S['zf'])} mm off the floor, inside the supervisor bounds in both "
         f"modes, tip reserve ≥ {mm(tip_min)} mm. Sideways (90°, doors without a threshold) the arms fold inside the base: "
         f"**{mm(S90['w'])} mm** (the Ø500 shell). **Facing the door (0°) no pose fits:** the shoulder roll motors alone span "
         f"{mm(o['shoulder_floor_w0'])} mm whatever the joints do (best found {mm(S0['w'])} mm).", ""]
    L += ["## Door", "", "| | mm | source |", "|---|---|---|",
          "| R5 text | 700 | \"fits 70 cm doors with arms folded\" (RND_PLAN §2): no clear width, no source |",
          f"| door leaf (70 cm class) | {mm(o['leaf'])} | DIN 18101 nominal 750 (leaf 735 × 1985); UK 686, US 28\" 711 |",
          f"| between the frame jambs | {mm(o['frame_clear'])} | DIN 18101 tables (sanier.de \"Türen richtig planen\", baunetzwissen.de) |",
          f"| **clear width used** | **{mm(o['clear'])}** | jambs less the open leaf (≈ 40 mm), measured as ADA 2010 §404.2.3 does (door open 90°, face of the door to the stop); ESTIMATE |",
          f"| margin a side | {mm(o['margin'])} | base planner centring + sway (ESTIMATE) |",
          f"| **width allowed** | **{mm(o['w_max'])}** | |",
          f"| head-jamb height | {o['door_h'] * 1000:.0f} | DIN leaf 1985 (ESTIMATE); robot top in the stow {mm(S['zmax'])} {ok(S['zmax'] < o['door_h'])} |", ""]
    L += ["## Today's park poses (mirror-symmetric, lift 0.70 m)", "",
          "Width across the travel direction, whole robot with the cover allowance.", "",
          "| Pose | W(0°) forward | W(45°) threshold | W(90°) sideways | narrowest (heading) | min cross-body gap |", "|---|---|---|---|---|---|"]
    for r in o["baseline"]:
        L.append(f"| {r['name']} | {mm(r['w0'])} | {mm(r['w45'])} | {mm(r['w90'])} | {mm(r['wmin'])} ({r['at']:.0f}°) | {mm(r['gap'])} |")
    L += ["", f"Forward floor: the pitch body (the GIM8108-36 roll motor, Ø96 on the roll axis {mm(o['roll_offset_y'])} mm outboard of "
          f"the shoulder at y ±{mm(o['shoulder_y'])}; its spring ring and roll link) only turns about Y, so its |y| is fixed: "
          f"{mm(o['shoulder_floor_w0'])} mm with the cover allowance. The tip-over track's 0.70–0.78 m is this, measured facing forward.", ""]
    L += ["## The door stow (ψ = 45°, valid in both supervisor modes)", "",
          f"Search: Powell from {len(SEEDS)} seeds over the lift + 12 arm joints inside the supervisor target bounds (limits less 3°, "
          f"roll ≥ +5° + 3°); cost = W(45°) + penalties for gaps < 25 mm and arm parts < 50 mm off the floor (+ a small pull toward a "
          f"centred COM and a low lift). Seed kept: *{S['seed']}*.", "",
          "| Joint | Right arm | Left arm (twin sign) |", "|---|---|---|"]
    for i, j in enumerate(ARM_J):
        L.append(f"| {j} | {_deg(q[1 + i])} | {_deg(q[7 + i] * MIRROR[j])} |")
    L += [f"| lift (shoulder height) | {q[0]:.2f} m | |", "",
          f"In words: the **right arm swings back** (pitch {_deg(q[1])}, elbow {_deg(q[4])}); the **left arm points forward** (pitch "
          f"{_deg(q[7])}, elbow {_deg(q[10])}). Along the crossing diagonal the right shoulder leads and the left trails, so each arm "
          "goes to where the diagonal is short: ahead on the left, behind on the right.", "",
          "- W(ψ) round the crossing heading: " + ", ".join(f"{k}° {mm(v)}" for k, v in S["W_band"].items()) + " mm.",
          f"- Set by: {S['extremes'][0][0]} (+{mm(S['extremes'][0][1])} mm) and {S['extremes'][1][0]} ({mm(S['extremes'][1][1])} mm) "
          "along the normal to the travel direction.",
          f"- Crossing at 135° instead: the mirror pose (left ↔ right), {mm(o['w135_mirror'])} mm.",
          "- Smallest cross-body gaps (after the cover allowance): " + "; ".join(f"{g[1]} × {g[2]} {mm(g[0])} mm" for g in S["gaps"][:3]) + ".",
          f"- Normal mode only (roll down to −12°): {mm(Sn['w'])} mm (the people-near bound costs {mm(Sn['w'] - S['w'])} mm here).", ""]
    L += ["## Sideways stow (ψ = 90°, doors without a threshold)", "",
          f"{mm(S90['w'])} mm (seed *{S90['seed']}*), set by {S90['extremes'][0][0]} / {S90['extremes'][1][0]}. Joints (right / left, twin "
          "sign): " + ", ".join(f"{j} {_deg(S90['q'][1 + i])} / {_deg(S90['q'][7 + i] * MIRROR[j])}" for i, j in enumerate(ARM_J)) +
          f", lift {S90['q'][0]:.2f} m. Straight at a wheel, so only where there is no threshold (rnd/threshold.py: 0° fails at 15 mm).", ""]
    t = o["tip"]
    al, ast = o["approach_linear"], o["approach_staged"]
    L += ["## Checks in the stow", "", "| Check | Value | Need | |", "|---|---|---|---|"]
    for r in t["rows"]:
        L.append(f"| tip (rnd/tip.py reserve): {r['case']} | {mm(r['reserve'])} mm (worst heading {r['at']:.0f}°) | ≥ {mm(r['need'])} mm | {ok(r['reserve'] >= r['need'])} |")
    L += [f"| COM | x {mm(t['com'][0])}, y {mm(t['com'][1])}, h {mm(t['com'][2])} mm ({t['M']:.1f} kg) | | |",
          f"| cross-body gaps in the pose | ≥ {mm(S['gap'])} mm | ≥ 25 mm (R17) | {ok(S['gap'] >= o['gap_open'] - 1e-4)} |",
          f"| arm parts above the floor | ≥ {mm(S['zf'])} mm | ≥ 50 mm | {ok(S['zf'] >= o['floor_min'] - 1e-4)} |",
          f"| into the stow from twin.TUCK, all joints together | gap ≥ {mm(al['gap'])} mm, ≥ {mm(al['zmin'])} mm off the floor | ≥ 25 / 50 mm | {ok(al['gap'] >= o['gap_open'] - 1e-4 and al['zmin'] >= o['floor_min'] - 1e-4)} |",
          f"| into the stow, one joint group at a time (better of two orders) | gap ≥ {mm(ast['gap'])} mm, ≥ {mm(ast['zmin'])} mm | ≥ 25 / 50 mm | {ok(ast['gap'] >= o['gap_open'] - 1e-4 and ast['zmin'] >= o['floor_min'] - 1e-4)} |"]
    sg = o["shoulder_gaps"]
    if sg["stow"]:
        g, gt = sg["stow"][0], (sg["tuck"][0] if sg["tuck"] else None)
        L += ["", f"**Not gated here: the roll output × torso wedge** ({g[1]} × {g[2]}: {mm(g[0])} mm in the stow"
              + (f", {mm(gt[0])} mm in twin.TUCK" if gt else "") + "). It is a pitch + roll pair that rnd/pinch.py already sweeps "
              "(rows roll_cover_yaw / roll_yaw_band × torso_shell; 17 mm at +15° with people near), it sits in the 8–25 mm band in every "
              "arms-down pose, and only a cover change closes it (a torso-side skirt that keeps it ≤ 8 mm, or a hood that keeps ≥ 25 mm). "
              "The stow is no worse than the park pose."]
    if o["pinch_near"]:
        L += ["", "rnd/pinch.py single-joint rows (docs/rnd/pinch.json, tier 1) that fail within 15° of a stow angle. They fail over "
              "the sweep whichever pose is used; listed so the pose owner knows which ones it sits near:", "",
              "| Sweep | Joint | Moving × static | min gap | worst at | stow within |", "|---|---|---|---|---|---|"]
        seen = set()
        for r in o["pinch_near"]:
            k = (r["joint"], r["moving"], r["static"])
            if k not in seen:
                seen.add(k)
                L.append(f"| {r['src']} | {r['joint']} | {r['moving']} × {r['static']} | {r['min']:.1f} mm | {r['at']:.0f}° | {r['stow']:.0f}° |")
    g0 = o["geometry"][0]
    L += ["", "## Facing the door (0°): the floor and the geometry it would take", "",
          f"W(0°) ≥ {mm(o['shoulder_floor_w0'])} mm in any pose. To fit {mm(o['w_max'])} mm facing the door the roll motors' outboard "
          f"faces must come in {mm(o['need_dy_fwd'])} mm a side: shoulder_y {o['shoulder_y']:.3f} → {o['shoulder_y'] - o['need_dy_fwd']:.3f} m, "
          f"or roll_offset_y {mm(o['roll_offset_y'])} → {mm(o['roll_offset_y'] - o['need_dy_fwd'])} mm (the Ø96 GIM8108-36 sits on the roll "
          "axis, so that means a smaller roll motor or the belt roll back), or a mix. The torso-side pitch modules (135 mm deep in y, "
          "rnd/parts/collision.py) end at y ±60 mm against the 60 mm mast (±30 mm): the shoulders can come in ≈ 25 mm before the "
          "modules meet the mast, which is not enough on its own.", "",
          "| shoulder_y | W(0°) floor | W(45°) stow | R1 coverage, outer columns y −0.40 / −0.325 m (twin IK, 2 seeds) |", "|---|---|---|---|"]
    for g in o["geometry"]:
        r1 = "; ".join(f"{k.split(' (')[0]} {v:.0%}" for k, v in g["r1"].items())
        L.append(f"| {g['shoulder_y']:.3f}{' (today)' if g is g0 else f' (torso shell 390 → {390 - 2 * (o['shoulder_y'] - g['shoulder_y']) * 1000:.0f} mm)'} "
                 f"| {mm(g['floor_w0'])} | {mm(g['w45'])}{'' if g['ok'] else ' (gaps ❌)'} | {r1} |")
    L += ["", "R16 (rnd/dexterity.py) is computed on the bench arm in its own frame (sw/bench_env.py has no shoulder_y), so it does not "
          "change. The narrower torso is hull-level only (pitch-module hulls shifted, torso box narrowed; ESTIMATE).", "",
          "Not recommended: the 45° crossing fits with the shipped geometry, and 25 mm off the torso costs tray width and still "
          "does not fit facing the door.", ""]
    L += ["## Planner / supervisor (\"door mode\")", "",
          "For the software owner. A door is a map feature with a clear width c and a threshold flag.", "",
          "1. `planner.door_mode(door)`: crossing heading ψ = 45° or 135° (the mirror stow; whichever needs less base rotation) when "
          "the door has a threshold, ψ = 90° (sideways stow) otherwise; refuse the door and report it if c − W(ψ) < 2 × 40 mm "
          "(W from docs/rnd/stow.json).",
          "2. Stop ≥ 0.5 m before the door; lift first, then the arms into the stow (paths checked above); pan the head to the travel "
          "direction; centre on the door axis (lateral error ≤ (c − W) / 2); cross at ≤ 0.12 m/s (SW-T4, D2b); back to the park pose.",
          "3. Supervisor: the stow puts the hands outside the hand workspace box (ws_lo / ws_hi: x ≥ 0.10 m, z ≥ 0.76 m), so the fk "
          "workspace check would reject it. Proposed `Supervisor.set_door_stow(q_stow | None)`: while set, arm targets must lie within "
          "±3° of the stow path (anything else rejected) in place of the workspace box; base_accel_limit() unchanged; refused when "
          "payload > LOAD_KG (no door crossing with a load in this pose). The stow is inside the people-near bounds, so "
          "set_people_near() never has to walk it out.", ""]
    L += ["## Scorecard row (for rnd/run_all.py)", "", "```python", "from . import stow",
          "rows += stow.scorecard_rows()   # [(label, value, target, pass)] from docs/rnd/stow.json", "```", "",
          "Assumptions (ESTIMATE): the clear width; 40 mm a side; the 5.2 mm cover allowance on every arm hull (15 mm on the wrist-boot "
          "hull) and none on the torso box / base shell (they are the shells); gaps are hull distances (mj_geomDistance); the twin's "
          "joint spheres are left out (mass placeholders inside the hulls)."]
    return "\n".join(L) + "\n"


if __name__ == "__main__":
    main()
