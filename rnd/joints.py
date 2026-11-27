"""Structural joints of the arm that carry the whole arm (Fable review 2026-11-27, C1 and M1).  python -m rnd.joints

C1: the pitch hub → pitch tube and the yaw flange → yaw tube joints were 6 × M3 into heat-set inserts in pull-out. Both
are now **tie-rod joints**: steel or aluminium threads only, the printed tube and hub clamped in compression between
metal parts (rnd/parts/shoulder.py HUB_TIE, rnd/parts/roll.py YAW_TIE). M1: the inline GIM8108-36 output boss, its rear
screws and its hard stops.

Loads at each joint interface come from the twin (rnd/twin.py, CAD masses) as the full wrench of everything outboard of
the interface: **pull-off** N (along the joint axis, outboard), **bending** M (the prying moment, perpendicular to the
axis), **torsion** T (about the axis, carried by the double-D / hex form fit, not by the tie).
- **Basket** (R2d): the rnd/basket.py targets with the D20 motors and 4.54 kg per hand; every collision-free IK solution
  is kept (an upper bound on the planner's pick), × 1.5 dynamic.
- **Hug** (R2c, rnd/payload_study.py case B): upper arm hanging, forearm horizontal, 4.54 kg 0.30 m out along the
  forearm, yaw 0/±45°, × 1.5 dynamic.
- **Bump**: 140 N at the grasp point (R17 transient contact limit) in ±X/±Y/±Z, arm hanging / forward / abducted /
  elbow 90°, plus the arm's weight. A fault/abuse load: checked against ultimate, the joint may gap.
- **Stall**: each of pitch, roll and elbow at its peak (D20 set, the worse one) driving the hand into a rigid block (the
  block force = stall torque / lever, in the joint's direction of motion), same poses. Fault load, as the bump.

Joint model (clamp()): a rigid metal end plate (crank/clamp ring, aluminium flange) on the printed contact annulus,
which is a compression-only elastic foundation (E 1.8 GPa over the printed stack length), with the bolts/tie as springs
holding the preload. It gives the moment at which the interface starts to gap, and the bolt force and peak printed
contact stress under each case. Criteria: service (basket, hug) no gap and contact ≤ 12 MPa; fault (bump, stall) bolt
≤ proof load, thread strip ≥ 1.5 × the bolt force, contact ≤ 40 MPa (PETG compressive yield ≈ 55–60 MPa, ESTIMATE;
compression closes the layers, so the across-layer 12 MPa does not apply). Preload loss by creep × 0.75 (as the other
printed joints, ESTIMATE until the coupon).
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
G = 9.81
E_PRINT = 1800.0                          # MPa (rnd/fea.py)
E_STEEL, E_AL = 200e3, 71e3
DYN = 1.5
F_BUMP = 140.0                            # N, R17 transient contact limit at the hand
SIGMA_SERVICE, SIGMA_FAULT = 12.0, 40.0   # MPa printed contact (sustained / fault), ESTIMATE


# ------------------------------------------------------------------------------------------------ loads from the twin
def _model(d):
    import mujoco
    from .params import act
    from .twin import Reacher
    R = Reacher(d)
    m = R.m
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    sub = lambda root: {b for b in range(m.nbody) if _is_desc(m, b, root)}
    ifs = {"pitch hub ↔ tube": dict(body=bid("sh_r"), sub=sub(bid("roll_r")), p=np.array([0, 0.0255, 0]), axis=np.array([0, 1.0, 0]), out=-1.0, skip=0.0),
           "yaw flange ↔ tube": dict(body=bid("upper_r"), sub=sub(bid("upper_r")), p=np.array([0, 0, 0.0036]), axis=np.array([0, 0, 1.0]), out=-1.0,
                                     skip=act(d.arm.joints["upper_yaw"][0])["mass"] + d.arm.yaw_stage_mass)}   # the yaw stage + motor (a point mass at the upper body origin in the twin) sit above the flange, on the roll output
    return R, ifs


def _is_desc(m, b, root):
    while b > 0:
        if b == root:
            return True
        b = m.body_parentid[b]
    return False


def wrench(R, ifs, q, ext=(), payload_on=True):
    """{interface: (N pull-off, M bending, T torsion)} for pose q under gravity + ext [(world point, world force)]."""
    import mujoco
    m, d = R.m, R.d
    d.qpos[R.adr] = q
    mujoco.mj_kinematics(m, d)
    pay = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload_r")
    out = {}
    for name, f in ifs.items():
        Rb = d.xmat[f["body"]].reshape(3, 3)
        p = d.xpos[f["body"]] + Rb @ f["p"]
        a = Rb @ f["axis"]
        F, M = np.zeros(3), np.zeros(3)
        pts = [(d.xipos[b], m.body_mass[b]) for b in f["sub"] if payload_on or b != pay] + [(d.xpos[f["body"]], -f["skip"])]
        for x, mass in pts:
            fg = np.array([0, 0, -G * mass])
            F += fg
            M += np.cross(x - p, fg)
        for x, fe in ext:
            F += fe
            M += np.cross(np.asarray(x) - p, fe)
        T = float(M @ a)
        out[name] = (float(F @ (f["out"] * a)), float(np.linalg.norm(M - T * a)), abs(T))
    return out


POSES = {"hanging": (0, 0, 0, 0), "forward": (-90, 0, 0, 0), "abducted 90°": (0, 90, 0, 0), "elbow 90°": (0, 0, 0, -90)}


def _q(pitch, roll, yaw, elbow, lift=0.7):
    r = math.radians
    return np.array([lift, r(pitch), r(roll), r(yaw), r(elbow), 0.0, 0.0])


def cases():
    """Worst (N, M, T) per interface and load case (basket and hug already × 1.5)."""
    import dataclasses
    import mujoco
    from .arm_sizing import capacity
    from .basket import POSES as BPOSES, HALF_SPAN, PER_HAND, design
    db = design()                                            # D20 motors, 4.54 kg per hand
    R, ifs = _model(db)
    res = {n: {} for n in ifs}

    def keep(case, w, factor=1.0, note=""):
        """Keep the largest bending (with its pose), and independently the largest pull-off and torsion of the case: the
        clamp is checked with max N and max M together (conservative)."""
        for n, (N, M, T) in w.items():
            cur = res[n].setdefault(case, dict(N=0.0, M=0.0, T=0.0, note=""))
            if M * factor > cur["M"]:
                cur.update(M=round(M * factor, 2), note=note)
            cur["N"] = max(cur["N"], round(N * factor, 1))
            cur["T"] = max(cur["T"], round(T * factor, 2))
    # basket: every collision-free solution of the passing targets
    for name, z, xs, rule in BPOSES:
        for x in xs:
            for q in R.solve(np.array([x, -HALF_SPAN, z]), [0, 0, -1.0], seeds=6):
                keep("basket × 1.5", wrench(R, ifs, q), DYN, f"{name}, x {x:.2f}")
    # hug: payload body off, 4.54 kg on the forearm 0.30 m from the elbow
    fore = mujoco.mj_name2id(R.m, mujoco.mjtObj.mjOBJ_BODY, "fore_r")
    for yaw in (-45, 0, 45):
        for el in (-90, 90):
            q = _q(0, 0, yaw, el)
            R.d.qpos[R.adr] = q
            mujoco.mj_kinematics(R.m, R.d)
            x = R.d.xpos[fore] + R.d.xmat[fore].reshape(3, 3) @ np.array([0, 0, -0.30])
            keep("hug × 1.5", wrench(R, ifs, q, [(x, np.array([0, 0, -G * PER_HAND]))], payload_on=False), DYN, f"yaw {yaw}°, elbow {el}°")
    # bump and stall (no payload)
    d0 = dataclasses.replace(db, arm=dataclasses.replace(db.arm, payload=0.0))
    R0, ifs0 = _model(d0)
    site = R0.site
    axes = {"pitch": ("sh_r", np.array([0, 1.0, 0])), "roll": ("roll_r", np.array([-1.0, 0, 0])), "elbow": ("fore_r", np.array([0, 1.0, 0]))}
    full = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "elbow": "elbow"}
    stall = {j: capacity(d0, full[j])[3] for j in axes}
    for pn, pose in POSES.items():
        q = _q(*pose)
        R0.d.qpos[R0.adr] = q
        mujoco.mj_kinematics(R0.m, R0.d)
        h = R0.d.site_xpos[site].copy()
        for k in range(3):
            for s in (-1, 1):
                fe = np.zeros(3)
                fe[k] = s * F_BUMP
                keep("bump 140 N", wrench(R0, ifs0, q, [(h, fe)]), 1.0, f"{pn}, {'+-'[s < 0]}{'XYZ'[k]}")
        for j, (bn, ax) in axes.items():
            b = mujoco.mj_name2id(R0.m, mujoco.mjtObj.mjOBJ_BODY, bn)
            R0.d.qpos[R0.adr] = q
            mujoco.mj_kinematics(R0.m, R0.d)
            a = R0.d.xmat[b].reshape(3, 3) @ ax
            r = h - R0.d.xpos[b]
            r_perp = r - (r @ a) * a
            lev = np.linalg.norm(r_perp)
            if lev < 0.05:
                continue
            u = np.cross(a, r_perp) / lev
            for s in (-1, 1):
                keep(f"stall (pitch {stall['pitch']:.0f} / roll {stall['roll']:.0f} / elbow {stall['elbow']:.0f} N·m)",
                     wrench(R0, ifs0, q, [(h, s * stall[j] / lev * u)]), 1.0, f"{j} stall, {pn}")
    return res


# ------------------------------------------------------------------------------------------------ clamped joint model
class Clamp:
    """Rigid end plate on a printed contact annulus r_in…r_out (mm) of stack length L_print (mm), held by bolts
    [(x, y, k N/mm)] each preloaded to `preload` N (× creep). solve(N, M): pull-off N (N) and moment M (N·mm, about x)
    on the plate → (contact pressure field MPa, bolt forces N, gap?). The bolt stiffness k = E·A_s/L_grip."""

    def __init__(self, r_in, r_out, L_print, bolts, preload, creep=0.75, n=50):
        rr = np.linspace(r_in, r_out, n)
        tt = np.linspace(0, 2 * math.pi, 4 * n, endpoint=False)
        R_, T_ = np.meshgrid((rr[:-1] + rr[1:]) / 2, tt)
        self.dA = (rr[1] - rr[0]) * R_ * (tt[1] - tt[0])
        self.Y = R_ * np.sin(T_)
        self.kf = E_PRINT / L_print
        self.Fp = preload * creep
        self.by = np.array([b[1] for b in bolts], float)
        self.kb = np.array([b[2] for b in bolts], float)
        self.u0 = len(bolts) * self.Fp / (self.kf * self.dA.sum())

    def _res(self, v, N, M):
        w, th = v
        p = self.kf * np.maximum(self.u0 - w - th * self.Y, 0.0)
        Fb = self.Fp + self.kb * (w + th * self.by)
        return np.array([N + (p * self.dA).sum() - Fb.sum(), (M + (p * self.dA * self.Y).sum() - (Fb * self.by).sum()) / 20.0]), p, Fb

    def solve(self, N, M):
        v = np.zeros(2)
        for _ in range(300):
            r0, p, Fb = self._res(v, N, M)
            if np.abs(r0).max() < 1e-4:
                break
            J = np.zeros((2, 2))
            for i, h in enumerate((1e-7, 1e-9)):
                dv = np.zeros(2)
                dv[i] = h
                J[:, i] = (self._res(v + dv, N, M)[0] - r0) / h
            step = np.linalg.solve(J, r0)
            v = v - step * (0.5 if np.abs(step[1]) > 1e-3 else 1.0)
        r0, p, Fb = self._res(v, N, M)
        assert np.abs(r0).max() < 1e-2, f"clamp solver did not converge {r0}"
        return p, Fb, bool((p <= 0).any())

    def m_gap(self, N=0.0):
        """Moment (N·m) at which the interface starts to open, with pull-off N."""
        lo, hi = 0.0, 5e5
        if self.solve(N, 0.0)[2]:
            return 0.0
        for _ in range(40):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if not self.solve(N, mid)[2] else (lo, mid)
        return lo / 1000

    def check(self, N, M):
        p, Fb, gap = self.solve(N, M * 1000)
        return dict(gap=gap, F_bolt=float(Fb.max()), sigma=float(p.max()))


def bolt_ring(n, r, ang0_deg, d_s, L_grip, E=E_STEEL):
    """n bolts on radius r (mm), tensile area d_s (mm²), grip L_grip (mm): [(x, y, k)]."""
    k = E * d_s / L_grip
    return [(r * math.cos(math.radians(ang0_deg + 360 * i / n)), r * math.sin(math.radians(ang0_deg + 360 * i / n)), k) for i in range(n)]


def thread_strip(d, pitch, L_e, tau):
    """Internal-thread strip strength (N): A = π·n·L_e·D_s,min·[1/(2n) + 0.57735·(D_s,min − E_n,max)] (FED-STD-H28 /
    Machinery's Handbook), D_s,min ≈ d − 0.16·P (6g), E_n,max ≈ d − 0.6495·P + 0.11·P (6H), × τ_u of the nut material."""
    n = 1 / pitch
    Ds = d - 0.16 * pitch
    En = d - 0.6495 * pitch + 0.11 * pitch
    return math.pi * n * L_e * Ds * (1 / (2 * n) + 0.57735 * (Ds - En)) * tau


# ------------------------------------------------------------------------------------------------ the joints
SERVICE = ("basket × 1.5", "hug × 1.5")


def joint_models():
    """The two C1 joints as clamp models (two printed-stack lengths, stiff / soft foundation; two pattern orientations)."""
    from .parts import roll, shoulder
    h, y = shoulder.HUB_TIE, roll.YAW_TIE
    lv = roll.yaw_tie_levels()
    L_hub = shoulder.Y_TUBE_END - shoulder.Y_TUBE_IN                         # printed tube length under the clamp
    L_yaw = lv["seat"] - (roll.FL_TOP - roll.FL_REC)
    out = {}
    for name, t, r_in, r_out, L, L_soft, grip, nut in (
            ("pitch hub ↔ tube", h, 12.0, 19.0, L_hub, L_hub + 19.0, shoulder.hub_tie_length() - 2.0 - h["nut_h"] / 2,
             dict(what=f"7075-T6 crank ({h['crank_t']:.0f} mm tapped)", L_e=h["crank_t"], tau=h["tau_crank"])),
            ("yaw flange ↔ tube", y, 14.1, 20.0, L_yaw, L_yaw, y["L"] - lv["engaged"] / 2,
             dict(what=f"6061-T6 flange ({lv['engaged']:.1f} mm tapped)", L_e=lv["engaged"], tau=y["tau_flange"]))):
        models = []
        for Lp in (0.5 * L, L_soft):
            for rot in (0.0, 180.0 / t["n"]):
                models.append(Clamp(r_in, r_out, Lp, bolt_ring(t["n"], t["r"], t["ang0"] + rot, t["As"], grip), t["preload"], t["creep"]))
        out[name] = dict(t=t, models=models, strip=thread_strip(t["d"], t["pitch"], nut["L_e"], nut["tau"]), nut=nut,
                         proof=t["proof"] * t["As"], r=(r_in, r_out), area=math.pi * (r_out ** 2 - r_in ** 2))
    return out


def _before(name, N, M):
    """Wave-2 joint: 6 × M3 into heat-set inserts; force per insert = M/(3·r) + N/6 (pivot at the centre, as the review)
    against ≈ 1.0 kN pull-out (ESTIMATE): capacity / demand."""
    r = 15.5 if name.startswith("pitch") else 16.5
    return round(1000.0 / max(M * 1000 / (3 * r) + N / 6, 1e-6), 2)


def joint_rows(loads):
    rows = []
    for name, J in joint_models().items():
        t = J["t"]
        F0 = t["n"] * t["preload"]
        rows.append(dict(joint=name, case="preload (assembly)", verdict=True,
                         text=f"{t['n']} × M{t['d']:.0f} {t['grade']} at {t['preload'] / 1e3:.1f} kN = {F0 / 1e3:.1f} kN → {F0 / J['area']:.1f} MPa on the printed "
                              f"contact annulus r {J['r'][0]:.1f}–{J['r'][1]:.1f} (× {t['creep']} after creep); M_gap at N = 0: "
                              f"{min(m.m_gap(0) for m in J['models']):.0f} N·m"))
        for case, L in loads[name].items():
            N, M = max(L["N"], 0.0), L["M"]
            chk = [m.check(N, M) for m in J["models"]]
            Fb = max(c["F_bolt"] for c in chk)
            sig = max(c["sigma"] for c in chk)
            gap = any(c["gap"] for c in chk)
            if case in SERVICE:
                mg = min(m.m_gap(N) for m in J["models"])
                ok = (not gap) and sig <= SIGMA_SERVICE
                margin = min(mg / max(M, 1e-6), SIGMA_SERVICE / sig)
                txt = f"no gap up to {mg:.0f} N·m; bolt {Fb / 1e3:.2f} kN; contact {sig:.1f} MPa (≤ {SIGMA_SERVICE:.0f})"
            else:
                ok = Fb <= J["proof"] and J["strip"] >= 1.5 * Fb and sig <= SIGMA_FAULT
                margin = min(J["proof"] / Fb, J["strip"] / (1.5 * Fb), SIGMA_FAULT / sig)
                txt = (f"{'gaps' if gap else 'no gap'}; bolt {Fb / 1e3:.2f} kN vs proof {J['proof'] / 1e3:.2f} kN; thread strip in the "
                       f"{J['nut']['what']} {J['strip'] / 1e3:.1f} kN ({J['strip'] / Fb:.1f}× the bolt, need 1.5×); contact {sig:.1f} MPa (≤ {SIGMA_FAULT:.0f})")
            rows.append(dict(joint=name, case=case, N=N, M=M, T=L["T"], pose=L["note"], margin=round(margin, 2), verdict=bool(ok), text=txt,
                             before=_before(name, N, M)))
    return rows


# ------------------------------------------------------------------------------------------------ M1: inline roll
def roll_rows():
    from .params import act
    from .parts import roll
    M = roll.MOT36
    stall, limit = act("GIM8108-36")["peak"], act("GIM8108-36")["rated"]
    r0 = M["out_pcd"] / 2000
    rows = []
    # output boss ↔ adapter: the dowels carry the torque; screw friction is a bonus
    dow_shear = math.pi * 2.0 ** 2 * 450.0                              # ISO 8734 Ø4: min double shear 11.3 kN → 5.65 kN single
    sh = M["dowel_n"] * dow_shear * r0
    sb = 240.0                                                          # bearing yield: cast A380 σ_y 160 × 1.5; 6061-T6 F_bry 345 MPa (MMPDS, e/D 1.5)
    br = M["dowel_n"] * M["dowel_d"] * M["dowel_l"] * sb * r0
    strip = thread_strip(4.0, 0.7, 3.0, 207.0)
    pre = 1500.0
    fr = M["out_n"] * pre * 0.15 * r0
    cap = min(sh, br)
    rows.append(dict(item="M1 output boss ↔ 7075 adapter (torque path)",
                     before="128 N·m with 26 N·m of friction on a 4 kN preload: 1.25× stall (review: ≈ 1.07× with ≤ 1 kN)",
                     after=f"dowels alone {cap:.0f} N·m = {cap / stall:.2f}× the {stall:.1f} N·m stall (+ {fr:.0f} N·m friction not counted)",
                     ok=cap >= 1.5 * stall, margin=cap / stall,
                     basis=(f"3 × Ø4 m6 hardened dowels on r {r0 * 1000:.2f}: single shear {dow_shear / 1e3:.2f} kN each (ISO 8734 min double shear 11.3 kN) → {sh:.0f} N·m; "
                            f"bearing in the motor's output, {M['dowel_l']:.0f} mm deep at {sb:.0f} MPa (ESTIMATE: cast A380 σ_y 160 MPa × 1.5; 6061-T6 F_bry 345 MPa, "
                            f"MMPDS, e/D 1.5; output material and dowel-hole depth to confirm with the seller) → {br:.0f} N·m (governs); the 7075 adapter side is 7 mm deep. "
                            f"Friction (bonus): 3 × M4 × 10 csk at an honest {pre / 1e3:.1f} kN (3.0 mm of M4 in Al strips at ≈ {strip / 1e3:.1f} kN, τ 207 MPa; ≈ 1.2 N·m), μ 0.15 → {fr:.0f} N·m")))
    rs = roll.REAR_SCREW
    worst = rs["L_tol"] + rs["grip_tol"] + rs["washer_tol"]
    rss = math.sqrt(rs["L_tol"] ** 2 + rs["grip_tol"] ** 2 + rs["washer_tol"] ** 2)
    rows.append(dict(item="M1 rear M4 engagement in the 3.5 mm housing threads (8 ×)", before="M4 × 18 under a 3.5 mm counterbore: 2.5 ± 0.55 mm",
                     after=f"{rs['engage']:.1f} mm nominal (± {worst:.2f} worst / ± {rss:.2f} RSS from stock parts) → shim picked at assembly: 3.0 ± 0.2",
                     ok=abs(rs["engage"] - 3.0) < 0.05, margin=None,
                     basis=(f"{rs['size']} (length js15 ± {rs['L_tol']}, ISO 4759-1) + ISO 7089 M4 washer {rs['washer']} (± {rs['washer_tol']}) + one DIN 988 shim "
                            f"{rs['shim']} (pick 0 / 0.3 / 0.5 after measuring the protrusion with a depth gauge, motor off) in a Ø{rs['cb_d']} × {rs['cb_depth']:.1f} counterbore; "
                            f"printed grip {rs['grip'] - rs['washer'] - rs['shim']:.1f} ± {rs['grip_tol']} (ESTIMATE). Preload {rs['preload'] / 1e3:.1f} kN (≈ 0.45 N·m; ≈ 10 MPa "
                            f"under the washer): friction 8 × μ 0.3 × 0.75 at r 42 = {8 * rs['preload'] * 0.75 * 0.3 * 0.042:.0f} N·m vs the {limit:.1f} N·m drive limit, "
                            f"then the screws bear in 19 mm of print; strip of 3.0 mm of M4 in Al ≈ {strip / 1e3:.1f} kN")))
    sp = roll.STOP_PIN
    F35, Fst = limit / (roll.STOP_R / 1000), stall / (roll.STOP_R / 1000)
    a = sp["proud"] / 2
    seat = lambda F: F / (sp["d"] * sp["seat"]) + 6 * F * a / (sp["d"] * sp["seat"] ** 2)
    pad = lambda F: F / (sp["strip"][0] * sp["strip"][1])
    sig_pin = 32 * Fst * a / (math.pi * sp["d"] ** 3)
    old = F35 * (roll.STOP_R / 44.0) / (8.0 * 6.0)
    rows.append(dict(item="M1 roll hard stops (both ends)",
                     before=f"printed Ø8 lugs at r 44: ≈ {old:.0f} MPa at the 35 N·m limit vs 12 MPa ({SIGMA_SERVICE / old:.2f}×)",
                     after=(f"steel Ø{sp['d']:.0f} pins at r {roll.STOP_R:.0f}: seat {seat(F35):.1f} MPa at the limit ({SIGMA_SERVICE / seat(F35):.2f}×), "
                            f"{seat(Fst):.0f} MPa at stall ({SIGMA_FAULT / seat(Fst):.2f}× of {SIGMA_FAULT:.0f})"),
                     ok=seat(F35) <= SIGMA_SERVICE and seat(Fst) <= SIGMA_FAULT and pad(Fst) <= SIGMA_FAULT, margin=min(SIGMA_SERVICE / seat(F35), SIGMA_FAULT / seat(Fst)),
                     basis=(f"F = τ / r: {F35:.0f} N at {limit:.1f} N·m, {Fst:.0f} N at stall. ISO 8734 Ø{sp['d']:.0f} × 28 hardened dowel pressed {sp['seat']:.1f} mm into the floor + cup "
                            f"ring, contact {a:.0f} mm off the face: seat pressure F/(d·L) + 6·F·a/(d·L²); pin bending at stall {sig_pin:.0f} MPa (hardened); 20 × 8 × 1.5 stainless "
                            f"wear strip bonded flush in each roll_arm_rear edge: {pad(F35):.1f} / {pad(Fst):.1f} MPa on the print (limit / stall)")))
    return rows


def report(loads=None):
    loads = loads or cases()
    jr, rr = joint_rows(loads), roll_rows()
    from .parts import roll, shoulder
    h, y = shoulder.HUB_TIE, roll.YAW_TIE
    lv = roll.yaw_tie_levels()
    L = ["# Arm structural joints: C1 tie-rod joints + M1 inline roll (generated by `python -m rnd.joints`)", "",
         __doc__.split("\n\n", 1)[1].strip(), "", "## Hardware", "",
         f"- **Pitch hub ↔ pitch tube** (rnd/parts/shoulder.py HUB_TIE): {h['n']} × M{h['d']:.0f} class {h['grade']} studs (DIN 976-1, cut to {shoulder.hub_tie_length():.1f} mm) "
         f"on r {h['r']} at {h['ang0']:.0f}° + k·90°, Loctite 270 into the 7075-T6 balancer crank (tapped through {h['crank_t']:.0f} mm, stud end flush with its inboard face), "
         f"Ø{h['hole']} holes through the tube wall (now r {h['tube_r_in']:.0f}–20) and the hub, M4 nuts (ISO 4032 class 8, Loctite 243) on a 6061 clamp ring "
         f"r {h['ring'][0]:.0f}–{h['ring'][1]:.0f} × {h['ring'][2]:.0f} in a {h['recess']:.1f} mm recess of the detach face; {h['preload'] / 1e3:.1f} kN each (≈ 1.3 N·m, ESTIMATE). "
         "Assembly: tube into the inner wall → crank + studs from the inboard side → belt → outer wall → hub over the double-D and the studs → ring, nuts.",
         f"- **Yaw flange ↔ yaw tube** (rnd/parts/roll.py YAW_TIE): the flange is now 6061-T6 (machined, ESTIMATE $25; its hex spigot stays the torque path). "
         f"{y['n']} × M{y['d']:.0f} × {y['L']:.0f} ISO 4762 class {y['grade']} go down from the tube's top end, heads on a steel clamp ring r {y['ring'][0]}–{y['ring'][1]} × {y['ring'][2]:.0f} "
         f"in a Ø{2 * y['cb_r']:.0f} × {y['cb_depth']:.1f} counterbore (heads {y['top_gap']} mm under the top, 0.5 mm under the top plate, under the yaw AS5048A board), through Ø{y['hole']} holes in the "
         f"wall (bore Ø16 for the ribbon) into M3 threads tapped at the hex corners (r {y['r']:.0f}) of the flange's spigot + plate ({lv['engaged']:.1f} mm engaged, tips "
         f"{lv['tip_to_face']:.1f} mm inside its underside, Loctite 243); {y['preload'] / 1e3:.1f} kN each (≈ 0.5 N·m). Tightened from above on the bench, before the yaw block goes onto "
         "the roll output.",
         "- Not drawn (rnd/parts/arm.py, arm track): the upper bone ↔ yaw flange screws. With the flange in aluminium they should be M4 into tapped holes in the flange, "
         "heads on washers on the bone flange, not inserts.", "",
         "## C1: capacity per load case", "",
         "Before = the wave-2 joint (6 × M3 into heat-set inserts, ≈ 1.0 kN pull-out ESTIMATE, force per insert = M/(3r) + N/6, as the review). Service rows "
         "(basket, hug) need no gap and ≤ 12 MPa; fault rows (bump, stall) need bolt ≤ proof, thread strip ≥ 1.5 × bolt, contact ≤ 40 MPa.", "",
         "| Joint | Case | Pull-off (N) | Bending (N·m) | Worst pose | Before: inserts | After | Margin | |", "|---|---|---|---|---|---|---|---|---|"]
    L += [f"| {r['joint']} | {r['case']} | {r['N']:.0f} | {r['M']:.1f} | {r['pose']} | {r['before']:.2f}× | {r['text']} | {r['margin']:.2f}× | {'✅' if r['verdict'] else '❌'} |"
          for r in jr if not r["case"].startswith("preload")]
    L += [""] + [f"- {r['joint']}, preload: {r['text']}" for r in jr if r["case"].startswith("preload")]
    L += ["", "## M1: inline GIM8108-36", "", "| Item | Before | After | |", "|---|---|---|---|"]
    L += [f"| {r['item']} | {r['before']} | {r['after']} | {'✅' if r['ok'] else '❌'} |" for r in rr]
    L += [""] + [f"- **{r['item']}:** {r['basis']}" for r in rr]
    L += ["", "Coupons to add (docs/COUPONS.md): the tie-rod joint (printed tube + hub under the stud preload, 1000 h at 40 °C, then the moment to gap), "
          "the 6061 threads of the yaw flange, the stop-pin seat at 2.4 kN."]
    ok = all(r["verdict"] for r in jr) and all(r["ok"] for r in rr)
    return "\n".join(L), ok, jr, rr


def main():
    md, ok, jr, rr = report()
    (DOCS / "joints.md").write_text(md + "\n")
    (DOCS / "joints.json").write_text(json.dumps(dict(ok=ok, joints=jr, roll=rr), default=str, indent=1))
    print(md)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
