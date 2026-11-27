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
SV_MARGIN = 3.0                  # deg, sw/safety.py Supervisor margin_deg


class Twin:
    """The twin model with both arms posable independently. q (deg, m) = [lift, right arm 6, left arm 6], the left arm in
    the right arm's convention (equal values = mirror-symmetric pose)."""

    def __init__(self, d=None):
        import mujoco
        from .params import BASELINE
        from .twin import build_xml
        self.mj = mujoco
        self.design = d or BASELINE
        m = self.m = mujoco.MjModel.from_xml_string(build_xml(self.design))
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
                self.pairs.append((g1, g2, self.pad[i] + self.pad[k]))
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

    def gaps(self, distmax=0.06):
        """Smallest cross-body gaps after the cover allowance: [(gap m, geom a, geom b)], sorted."""
        out = []
        ft = np.zeros(6)
        for g1, g2, pad in self.pairs:
            dist = self.mj.mj_geomDistance(self.m, self.d, g1, g2, distmax, ft)
            if dist < distmax:
                out.append((float(dist) - pad, self.gname[g1], self.gname[g2]))
        return sorted(out)

    def min_gap(self, distmax=0.06):
        g = self.gaps(distmax)
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

    # 3. geometry: forward width floor and what would close it
    ok45 = S["ok"] and S["w"] <= W_MAX
    out = dict(clear=CLEAR, clear_literal=CLEAR_LITERAL, margin=MARGIN, w_max=W_MAX, door_h=DOOR_H, pad=PAD,
               baseline=base_rows, shoulder_floor_w0=floor0, stow={str(k): v for k, v in res.items()},
               approach_linear=dict(gap=ap_lin[0], zmin=ap_lin[1], at=ap_lin[2]), approach_staged=dict(gap=st[0], zmin=st[1]),
               tip=tipr, pinch_near=pin, mirror_q=mirror.tolist(), w135_mirror=w135, ok=bool(ok45))
    for k in ("W",):
        for v in out["stow"].values():
            v.pop(k, None) if False else None
    (DOCS / "stow.json").write_text(json.dumps(out, indent=1, default=float))
    (DOCS / "stow.md").write_text(report(out, T))
    print(f"stow: W(45°) {S['w'] * 1000:.0f} mm (≤ {W_MAX * 1000:.0f}), gap {S['gap'] * 1000:.0f} mm, ok={ok45}")
    return out


def report(o, T):
    return "TODO"


if __name__ == "__main__":
    main()
