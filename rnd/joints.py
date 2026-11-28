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
compression closes the layers, so the across-layer 12 MPa does not apply).

Preload over life (Neon review 2026-11-27, finding 3 / assumption 1): the flat "× 0.75 for creep" is replaced by a
creep model (life_preload(): effective modulus, creep coefficient φ bounded from printed-PETG data, + embedding + a
cold day). With it, the wave-3 joints (print as the clamped member) keep ≈ 30 % of their preload, and the yaw tie
gaps under the basket wrench. Disc springs in series (the review's fix) keep more preload but soften the bolts, whose
stiffness carried the bending (disc_alternative()): not enough. So each C1 bolt now runs in steel crush sleeves
(SleeveClamp: bolt ↔ sleeve metal stack, the print sandwiched with a light squeeze): the preload only loses its
embedding, the no-gap moment is the metal stack's, and the service rows are checked at that life preload with the
print's squeeze crept away; the fault rows at the fresh and the life preload. M1 rows (inline
GIM8108-36): output dowels with load sharing, the metal rear clamp (plate + sleeves), the two-bearing roll support
(61806 + the motor's output bearing: reactions per load case from the twin, misalignment preload), the hard stops.
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
                                     skip=act(d.arm.joints["upper_yaw"][0])["mass"] + d.arm.yaw_stage_mass),   # the yaw stage + motor (a point mass at the upper body origin in the twin) sit above the flange, on the roll output
           # M1: the roll output on its two bearings (origin on the roll axis at x = 0, the yaw axis); the twin's roll-body
           # sphere (roll motor + static roll stage) is static in the CAD, so it is taken off
           "roll bearings": dict(body=bid("roll_r"), sub=sub(bid("roll_r")), p=np.zeros(3), axis=np.array([1.0, 0, 0]), out=-1.0,
                                 skip=act(d.arm.joints["shoulder_roll"][0])["mass"] + d.arm.roll_stage_mass, roll=True)}
    return R, ifs


def _is_desc(m, b, root):
    while b > 0:
        if b == root:
            return True
        b = m.body_parentid[b]
    return False


def wrench(R, ifs, q, ext=(), payload_on=True):
    """{interface: (N pull-off, M bending, T torsion)} for pose q under gravity + ext [(world point, world force)]. For the
    "roll bearings" interface the tuple is (R1 61806, R2 motor output bearing, F radial, M tilting about the motor bearing,
    T roll torque) instead (bearing_loads())."""
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
        if f.get("roll"):
            out[name] = bearing_loads(F, M, a) + (abs(T),)
        else:
            out[name] = (float(F @ (f["out"] * a)), float(np.linalg.norm(M - T * a)), abs(T))
    return out


def roll_spans():
    """(x1, x2) m: the 61806 (W1) and the motor's output bearing (W2) centres on the roll axis (rnd/parts/roll.py)."""
    from .parts import roll
    return (roll.W1[0] + roll.W1[1]) / 2000, (roll.W2[0] + roll.W2[1]) / 2000


def bearing_loads(F, M, a):
    """Two radial supports on the axis a at x1 (61806) and x2 (motor), the outboard wrench F, M about x = 0:
    R1 + R2 = −F⊥ and x1·R1 + x2·R2 = a × M⊥ → R2 = (a × M⊥ + x1·F⊥)/(x2 − x1). Without the 61806 the motor bearing
    carries F⊥ and the tilting moment M⊥ − x2·a × F about its own centre. Returns (|R1|, |R2|, |F⊥|, |M_x2⊥|)."""
    x1, x2 = roll_spans()
    Fp = F - (F @ a) * a
    Mp = M - (M @ a) * a
    R2 = (np.cross(a, Mp) + x1 * Fp) / (x2 - x1)
    R1 = -Fp - R2
    Mx2 = M - x2 * np.cross(a, F)
    Mx2 = Mx2 - (Mx2 @ a) * a
    return float(np.linalg.norm(R1)), float(np.linalg.norm(R2)), float(np.linalg.norm(Fp)), float(np.linalg.norm(Mx2))


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
        for n, v in w.items():
            if len(v) == 5:                                          # roll bearings: keep each maximum
                cur = res[n].setdefault(case, dict(R1=0.0, R2=0.0, F=0.0, Mm=0.0, T=0.0, note=""))
                if v[1] * factor > cur["R2"]:
                    cur["note"] = note
                for k, x in zip(("R1", "R2", "F", "Mm", "T"), v):
                    cur[k] = max(cur[k], round(x * factor, 1))
                continue
            N, M, T = v
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


# ------------------------------------------------------------------------------------------------ threads
# ISO 965-1/-2 limits for tolerance class 6g (bolt) / 6H (nut), by pitch (d range 2.8–5.6 mm): fundamental deviation es,
# major-diameter tolerance Td, pitch-diameter tolerances Td2 / TD2, minor-diameter tolerance TD1 (mm).
THREAD_6G6H = {0.5: dict(es=0.020, Td=0.106, Td2=0.075, TD1=0.140, TD2=0.100),
               0.7: dict(es=0.022, Td=0.140, Td2=0.085, TD1=0.180, TD2=0.112),
               0.8: dict(es=0.024, Td=0.150, Td2=0.095, TD1=0.200, TD2=0.125)}


def thread_strip(d, pitch, L_e, tau, tau_ext=None):
    """Thread-strip strength (N), FED-STD-H28/2 (Machinery's Handbook, "Length of engagement") at the 6g/6H limits:
    internal (nut material, τ): A_n = π·n·L_e·d_min·[1/(2n) + 0.57735·(d_min − D2_max)];
    external (bolt, τ_ext):     A_s = π·n·L_e·D1_max·[1/(2n) + 0.57735·(d2_min − D1_max)]; the weaker of the two.
    L_e = the *effective* engaged length: the caller takes off the thread chamfers (ESTIMATE 0.5 P per chamfered end)."""
    t = THREAD_6G6H[pitch]
    n = 1 / pitch
    d2, D1 = d - 0.649519 * pitch, d - 1.082532 * pitch
    d_min, D2_max = d - t["es"] - t["Td"], d2 + t["TD2"]
    D1_max, d2_min = D1 + t["TD1"], d2 - t["es"] - t["Td2"]
    F_int = math.pi * n * L_e * d_min * (1 / (2 * n) + 0.57735 * (d_min - D2_max)) * tau
    if tau_ext is None:
        return F_int
    F_ext = math.pi * n * L_e * D1_max * (1 / (2 * n) + 0.57735 * (d2_min - D1_max)) * tau_ext
    return min(F_int, F_ext)


TAU = {"7075-T6": 331.0, "6061-T6": 207.0, "Ti-6Al-4V": 550.0, "cast Al (housing)": 190.0, "10.9": 0.6 * 1040.0, "12.9": 0.6 * 1220.0}
TAU_BASIS = ("τ_u: 7075-T6 331, 6061-T6 207 MPa (ASM / MMPDS); Ti-6Al-4V 550 MPa (ASM, annealed); motor housing cast Al 190 MPa "
             "(ESTIMATE, A380-class 0.6 × 320); bolt 0.6 × R_m (ISO 898-1: 10.9 1040, 12.9 1220 MPa)")


# ------------------------------------------------------------------------------------------------ disc springs + creep
# DIN 6796 conical spring washers (for bolted joints): d1 × d2 × s, h max, and the load to flat (DIN 6796 table, as
# reproduced by fasten.it; dimensions as JC Fasteners / McMaster-Carr DIN 6796 listings). Sourcing: McMaster-Carr "DIN
# 6796 lock washers" (M3 / M4, spring steel), norelem 07000 "Conical spring washers DIN 6796", Bossard BN 794 —
# part numbers to confirm at order (any DIN 6796 spring steel washer of the size works; not the stainless A2 ones: lower F).
DIN6796 = {"M3": dict(d1=3.2, d2=7.0, s=0.6, h=0.85, F_flat=1320.0, m_g=0.13),
           "M4": dict(d1=4.3, d2=9.0, s=1.0, h=1.30, F_flat=3770.0, m_g=0.35)}


class Disc:
    """One DIN 2092 disc spring (no flats): F(s) = C·(s/t)·[(h0/t − s/t)(h0/t − s/2t) + 1], C = 4E/(1−ν²)·t⁴/(K1·De²).
    h0 is fitted so that F(h0) = the tabulated load to flat (h0 ≈ 0.24 mm for both DIN 6796 M3 and M4 at E 206 GPa)."""

    def __init__(self, size, E=206e3, nu=0.3):
        g = DIN6796[size]
        self.size, self.g = size, g
        De, Di, t = g["d2"], g["d1"], g["s"]
        dl = De / Di
        K1 = ((dl - 1) / dl) ** 2 / ((dl + 1) / (dl - 1) - 2 / math.log(dl)) / math.pi
        self.t, self.C = t, 4 * E / (1 - nu ** 2) * t ** 4 / (K1 * De ** 2)
        self.h0 = g["F_flat"] / (self.C / t)
        self.F_flat = g["F_flat"]

    def F(self, s):
        t, h = self.t, self.h0
        s = min(max(s, 0.0), h)
        return self.C * (s / t) * ((h / t - s / t) * (h / t - s / (2 * t)) + 1)

    def s(self, F):
        """Deflection (mm) at load F (flat = h0 beyond F_flat)."""
        if F >= self.F_flat:
            return self.h0
        lo, hi = 0.0, self.h0
        for _ in range(60):
            mid = (lo + hi) / 2
            lo, hi = (mid, hi) if self.F(mid) < F else (lo, mid)
        return (lo + hi) / 2


# Creep of the printed stack (assumption 1 of the Neon review). Effective-modulus method: under the sustained clamp the
# PETG's strain grows to σ/E·(1 + φ); the bolt + spring stack and the print are in series, so the life preload solves
#   δ_b(F) + δ_bell(F) + δ_p(F)·(1 + φ) + embedding + thermal = δ_b(F0) + δ_bell(F0) + δ_p(F0)   (assembly length).
# φ (creep coefficient at ≈ 16 % of the compressive yield, 40–50 °C, 2 × 10⁴ h) — ESTIMATE, bounded:
#   printed PETG in compression, 25 °C: +5.5 % strain after 180 min at 25 % of the compressive yield (Aerospace 2022,
#   9(3) 124, "Compressive behaviour of 3D-printed PETG composites") → φ(3 h, RT) ≈ 0.05; printed PETG tension creep ≈ 5 %
#   of the elastic strain in 5 h at 21 °C (PMC12349189, 2025). Findley power law t^n, n 0.15–0.25 for an amorphous
#   polymer below Tg (ESTIMATE): × 4–11 to 2 × 10⁴ h; 40–50 °C service (Tg ≈ 80 °C) = 1–2 decades of time–temperature
#   shift (ESTIMATE; Romeijn et al., UTS, printed PETG creep master curves valid below 60 °C): × 1.6–3. → φ ≈ 0.3–1.6.
#   Design value φ = 2.0 (= the review's upper 1 % creep strain at 9 MPa / 1.8 GPa); nominal 1.0.
CREEP = dict(phi=2.0, phi_nom=1.0, alpha_p=68e-6, alpha_s=11.5e-6, dT_cold=10.0, embed_print=0.010, embed_metal=0.002)
CREEP_BASIS = ("φ = 2.0 design / 1.0 nominal (ESTIMATE, bounded from printed-PETG creep data: Aerospace 2022 9(3) 124, PMC12349189; "
               "Findley t^n + 1–2 decades of 40–50 °C shift); PETG α 68e-6/K (datasheet range 60–70e-6, ESTIMATE) vs steel 11.5e-6, "
               "assembled at 20 °C, coldest service 10 °C; embedding 10 µm per printed interface (ESTIMATE: FDM layer peaks; "
               "VDI 2230 gives ≈ 2–3 µm per machined metal interface, used for the metal joints)")


def life_preload(F0, c_b, c_p, bell=None, n_bell=0, L_p=0.0, n_print_if=0, n_metal_if=0, phi=None):
    """Preload (N) left after creep (φ), embedding and the cold-day thermal shrink. c_b, c_p: bolt and printed-stack
    compliance (mm/N, per bolt), bell: a Disc (n_bell in series), L_p printed length (mm)."""
    phi = CREEP["phi"] if phi is None else phi
    db = lambda F: F * c_b + (n_bell * bell.s(F) if bell else 0.0)
    loss = n_print_if * CREEP["embed_print"] + n_metal_if * CREEP["embed_metal"] + (CREEP["alpha_p"] - CREEP["alpha_s"]) * CREEP["dT_cold"] * L_p
    target = db(F0) + F0 * c_p - loss
    lo, hi = 0.0, F0
    for _ in range(60):
        mid = (lo + hi) / 2
        lo, hi = (mid, hi) if db(mid) + mid * c_p * (1 + phi) < target else (lo, mid)
    return lo


# ------------------------------------------------------------------------------------------------ clamped joint model
class Clamp:
    """Rigid end plate on a printed contact annulus r_in…r_out (mm) of stack length L_print (mm), held by bolts
    [(x, y, k N/mm)] each preloaded to Fp N. solve(N, M): pull-off N (N) and moment M (N·mm, about x) on the plate →
    (contact pressure field MPa, bolt forces N, gap?). The bolt stiffness k = E·A_s/L_grip, in series with a disc-spring
    stack until it is flat (bilinear: k_soft up to F_knee, k beyond)."""

    def __init__(self, r_in, r_out, L_print, bolts, Fp, n=50, k_soft=None, F_knee=None):
        rr = np.linspace(r_in, r_out, n)
        tt = np.linspace(0, 2 * math.pi, 4 * n, endpoint=False)
        R_, T_ = np.meshgrid((rr[:-1] + rr[1:]) / 2, tt)
        self.dA = (rr[1] - rr[0]) * R_ * (tt[1] - tt[0])
        self.Y = R_ * np.sin(T_)
        self.kf = E_PRINT / L_print
        self.Fp = Fp
        self.by = np.array([b[1] for b in bolts], float)
        self.kb = np.array([b[2] for b in bolts], float)
        self.ks = self.kb if k_soft is None else np.full_like(self.kb, k_soft)
        self.knee = np.inf if F_knee is None else max(F_knee, Fp)
        self.u0 = len(bolts) * self.Fp / (self.kf * self.dA.sum())

    def _bolt(self, d):
        F = self.Fp + self.ks * d
        over = F > self.knee
        return np.where(over, self.knee + self.kb * (d - (self.knee - self.Fp) / self.ks), F)

    def _res(self, v, N, M):
        w, th = v
        p = self.kf * np.maximum(self.u0 - w - th * self.Y, 0.0)
        Fb = self._bolt(w + th * self.by)
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


class SleeveClamp(Clamp):
    """Neon finding 3 fix: each bolt runs in a steel crush sleeve (compression limiter) that spans the printed stack, so
    the bolt preload is reacted metal to metal (bolt ↔ sleeve) and the print is only sandwiched with a light squeeze.
    Rigid end plate; per bolt a tension spring k_b (preload Fp) and a compression-only sleeve spring k_s at the same
    point; the printed annulus is a compression-only foundation pre-squeezed by u0p (mm; 0 once creep has relaxed it).
    'gap' = the most-unloaded sleeve leaves its seat (the metal interface opens)."""

    def __init__(self, r_in, r_out, L_print, bolts, Fp, k_s, u0p=0.0, n=50):
        super().__init__(r_in, r_out, L_print, bolts, Fp, n=n)
        self.k_s = k_s
        self.u0 = u0p
        P = self.kf * u0p * self.dA.sum()
        self.us0 = (Fp - P / len(bolts)) / k_s                       # sleeve pre-compression

    def _res(self, v, N, M):
        w, th = v
        p = self.kf * np.maximum(self.u0 - w - th * self.Y, 0.0)
        S = self.k_s * np.maximum(self.us0 - w - th * self.by, 0.0)
        Fb = self.Fp + self.kb * (w + th * self.by)
        self._S = S
        return np.array([N + (p * self.dA).sum() + S.sum() - Fb.sum(),
                         (M + (p * self.dA * self.Y).sum() + (S * self.by).sum() - (Fb * self.by).sum()) / 20.0]), p, Fb

    def solve(self, N, M):
        p, Fb, _ = super().solve(N, M)
        return p, Fb, bool((self._S <= 0).any())

    def check(self, N, M):
        p, Fb, gap = self.solve(N, M * 1000)
        return dict(gap=gap, F_bolt=float(Fb.max()), sigma=float(p.max()), S=float(self._S.max()))


# ------------------------------------------------------------------------------------------------ the joints
SERVICE = ("basket × 1.5", "hug × 1.5")

# D26 cost-down (wave 11): the crush sleeves from stock tube, cut and faced to length by the builder, where the yield
# clears the sleeve stress; turned 42CrMo4+QT only where no stock tube fits the CAD hole. Per sleeve set: (OD, ID) mm,
# material, R_p (MPa, the guaranteed minimum), stock. The CAD holes (shoulder.HUB_TIE / roll.YAW_TIE sleeve_hole,
# roll.REAR_SCREW, roll.OUT_CLAMP) are unchanged: every stock OD is ≤ the CAD sleeve OD.
SLEEVE_STOCK = {
    "pitch": dict(sleeve=(7.0, 4.3), mat="42CrMo4+QT, turned", Rp=750.0,
                  stock=("turned: the two sleeves butt at the hub ↔ tube face, so they must be located by the Ø7.1 hole; 1/4\" × .035\" 4130 "
                         "(F_ty 517) floats 0.4 mm in it (butt overlap down to ≈ 0.3 of its 0.89 mm wall) and its softer section takes the stall "
                         "row 1.18 → 1.08× (thread strip); it fits if the CAD hole goes Ø7.1 → Ø6.45")),
    "yaw": dict(sleeve=(4.6, 3.2), mat="42CrMo4+QT, turned", Rp=750.0,
                stock="turned: no stock tube ≤ Ø4.7 with an ID ≥ 3.2 clears the yaw stall stress (4.5 × 0.6 E355+SR, R_eH 440: ≈ 1.07×)"),
    "rear": dict(sleeve=(6.0, 4.4), mat="E235+C precision tube (EN 10305-1; R_eH taken as the E235+N 235 MPa floor)", Rp=235.0,
                 stock="6 × 0.8 seamless E235+C precision tube"),
    "clevis": dict(sleeve=(8.0, 5.6), mat="E235+C precision tube (EN 10305-1; R_eH taken as 235 MPa)", Rp=235.0,
                   stock="8 × 1.2 seamless E235+C precision tube (press fit in the print: the bearing path)"),
}


def joint_models():
    """The two C1 joints: geometry, compliances, the life preload, and clamp-model factories (two printed-stack lengths
    for the print's share; two bolt-pattern orientations). make(Fp, u0p) → the sleeve-clamp models; make_bare(Fp) → the
    wave-3 joint (print as the clamped member, no sleeves) for the before/after comparison."""
    from .parts import roll, shoulder
    h = dict(shoulder.HUB_TIE, sleeve=SLEEVE_STOCK["pitch"]["sleeve"], Rp=SLEEVE_STOCK["pitch"]["Rp"])
    y = dict(roll.YAW_TIE, sleeve=SLEEVE_STOCK["yaw"]["sleeve"], Rp=SLEEVE_STOCK["yaw"]["Rp"])
    lv = roll.yaw_tie_levels()
    L_hub = shoulder.Y_TUBE_END - shoulder.Y_TUBE_IN                         # printed tube length under the clamp
    L_hub2 = shoulder.hub_print_under_ring()                                 # the hub's print between the tube end and the ring
    L_yaw = lv["seat"] - (roll.FL_TOP - roll.FL_REC)
    holes = lambda t, d: t["n"] * math.pi * (d / 2) ** 2
    out = {}
    for name, t, r_in, r_out, L, L_soft, grip, nut, stack, L_sl, n_if in (
            ("pitch hub ↔ tube", h, 12.0, 19.0, L_hub, L_hub + 19.0, shoulder.hub_tie_length() - 2.0 - h["nut_h"] / 2,
             dict(what=f"{h['crank_mat']} crank ({h['crank_t']:.0f} mm tapped through, {h['crank_t'] - 1.5 * h['pitch']:.2f} effective)",
                  L_e=h["crank_t"] - 1.5 * h["pitch"], tau=TAU[h["crank_mat"]], tau_ext=TAU[h["grade"]]),
             [(L_hub, math.pi * (20.0 ** 2 - h["tube_r_in"] ** 2)), (L_hub2, math.pi * (h["ring"][1] ** 2 - h["ring"][0] ** 2))],
             L_hub + L_hub2, 3),
            ("yaw flange ↔ tube", y, 14.1, 20.0, L_yaw, L_yaw, y["L"] - lv["engaged"] / 2,
             dict(what=f"{y.get('flange_mat', '6061-T6')} flange ({lv['engaged']:.1f} mm tapped, {lv['engaged'] - y['pitch']:.1f} effective)", L_e=lv["engaged"] - y["pitch"],
                  tau=y["tau_flange"], tau_ext=TAU[y["grade"]]),
             [(roll.YF["hex_h"], math.pi * (20.0 ** 2 - 14.1 ** 2)), (L_yaw - roll.YF["hex_h"], math.pi * (20.0 ** 2 - 8.0 ** 2))],
             L_yaw - roll.YF["hex_h"], 2)):
        sl = t["sleeve"]
        A_s = math.pi / 4 * (sl[0] ** 2 - sl[1] ** 2)
        k_s = E_STEEL * A_s / L_sl
        c_b, c_s = grip / (E_STEEL * t["As"]), 1 / k_s
        c_p_bare = sum(Li / (E_PRINT * (Ai - holes(t, t["hole"])) / t["n"]) for Li, Ai in stack)
        Lp = sum(Li for Li, _ in stack)
        F0 = t["preload"]
        # sleeved: metal stack, embedding only (head/nut, sleeve ends ×2–3, threads; VDI 2230 ≈ 2 µm each, ESTIMATE)
        F_life = life_preload(F0, c_b + c_s, 0.0, n_metal_if=n_if + 2, phi=0.0)
        # wave 3 as designed (the print is the clamped member): creep φ, embedding, the cold day
        F0_bare = t.get("preload_bare", F0)
        F_bare = life_preload(F0_bare, c_b, c_p_bare, L_p=Lp, n_print_if=n_if)
        F_bare_nom = life_preload(F0_bare, c_b, c_p_bare, L_p=Lp, n_print_if=n_if, phi=CREEP["phi_nom"])
        u0p = t["squeeze"]                                                    # the print's squeeze, fresh (mm over the stack)

        def make(Fp, u0p_=0.0, t=t, r_in=r_in, r_out=r_out, L=L, L_soft=L_soft, grip=grip, k_s=k_s):
            return [SleeveClamp(r_in, r_out, Lp_, bolt_ring(t["n"], t["r"], t["ang0"] + rot, t["As"], grip), Fp, k_s, u0p_)
                    for Lp_ in (0.5 * L, L_soft) for rot in (0.0, 180.0 / t["n"])]

        def make_bare(Fp, k_soft=None, knee=None, t=t, r_in=r_in, r_out=r_out, L=L, L_soft=L_soft, grip=grip):
            return [Clamp(r_in, r_out, Lp_, bolt_ring(t["n"], t["r"], t["ang0"] + rot, t["As"], grip), Fp, k_soft=k_soft, F_knee=knee)
                    for Lp_ in (0.5 * L, L_soft) for rot in (0.0, 180.0 / t["n"])]
        out[name] = dict(t=t, make=make, make_bare=make_bare, models=make(F_life), strip=thread_strip(t["d"], t["pitch"], nut["L_e"], nut["tau"], nut["tau_ext"]),
                         nut=nut, proof=t["proof"] * t["As"], r=(r_in, r_out), area=math.pi * (r_out ** 2 - r_in ** 2), A_s=A_s, k_s=k_s, L_sl=L_sl,
                         c_b=c_b, c_s=c_s, c_p=c_p_bare, L_print=Lp, F0=F0, F0_bare=F0_bare, F_life=F_life, F_bare=F_bare, F_bare_nom=F_bare_nom,
                         creep=F_life / F0, u0p=u0p, P_print=E_PRINT / Lp * u0p * (stack[0][1] - holes(t, sl[0] + 0.1)))
    return out


def required_preload(J, N, M, bare=False):
    """Smallest preload (N per bolt) at which the joint does not gap under (N, M N·m) in any of its clamp models (the
    sleeved joint with the print's squeeze relaxed; bare = the wave-3 print-clamped joint)."""
    lo, hi = 0.0, 4.0 * J["F0"]
    for _ in range(30):
        mid = (lo + hi) / 2
        ok = all(not m.check(N, M)["gap"] for m in (J["make_bare"](mid) if bare else J["make"](mid)))
        lo, hi = (lo, mid) if ok else (mid, hi)
    return hi


def disc_alternative(J, N, M, n_disc=3):
    """What the review's DIN 6796 stack would do: n discs in series under each nut/head on the wave-3 joint (print as the
    clamped member). The stack keeps more preload but softens the bolts, and the stiff bolts are what carried the bending:
    → (life preload, preload needed for no gap) N."""
    t = J["t"]
    disc = Disc(t["bell"])
    F0 = J["F0_bare"]
    F_life = life_preload(F0, J["c_b"], J["c_p"], disc, n_disc, J["L_print"], 3 if t["n"] == 4 else 2)
    k_soft = 1 / (J["c_b"] + n_disc * (disc.s(F_life * 1.02) - disc.s(F_life)) / (0.02 * F_life))
    mk = lambda Fp: J["make_bare"](Fp, k_soft, disc.F_flat)
    lo, hi = 0.0, 4.0 * F0
    for _ in range(30):
        mid = (lo + hi) / 2
        ok = all(not m.check(N, M)["gap"] for m in mk(mid))
        lo, hi = (lo, mid) if ok else (mid, hi)
    return F_life, hi, n_disc * disc.s(F0)


def _before(name, N, M):
    """Wave-2 joint: 6 × M3 into heat-set inserts; force per insert = M/(3·r) + N/6 (pivot at the centre, as the review)
    against ≈ 1.0 kN pull-out (ESTIMATE): capacity / demand."""
    r = 15.5 if name.startswith("pitch") else 16.5
    return round(1000.0 / max(M * 1000 / (3 * r) + N / 6, 1e-6), 2)


SLEEVE_RP = 750.0      # MPa, turned 42CrMo4+QT (R_p0.2 ≥ 750, EN 10083-3); each joint's own sleeve yield is SLEEVE_STOCK[...]["Rp"]


def joint_rows(loads):
    rows = []
    for name, J in joint_models().items():
        t = J["t"]
        F0 = t["n"] * t["preload"]
        svc = [(L["N"], L["M"]) for c, L in loads[name].items() if c in SERVICE]
        Nw, Mw = max(svc, key=lambda v: v[1])
        F_req = required_preload(J, max(Nw, 0.0), Mw)
        F_req_bare = required_preload(J, max(Nw, 0.0), Mw, bare=True)
        dF_life, dF_req, d_trav = disc_alternative(J, max(Nw, 0.0), Mw)
        bare_txt = (f"wave 3 as designed (print as the clamped member, {J['F0_bare'] / 1e3:.1f} kN): bolt {J['c_b'] * 1e6:.0f} vs print {J['c_p'] * 1e6:.0f} µm/kN per bolt "
                    f"({J['L_print']:.1f} mm of PETG: {J['F0_bare'] * J['c_p'] * 1000:.0f} µm of elastic squeeze, so the springs are bolt + print, not the bolt alone), "
                    f"life preload {J['F_bare'] / 1e3:.2f} kN ({100 * J['F_bare'] / J['F0_bare']:.0f} %; φ {CREEP['phi_nom']:.0f}: {J['F_bare_nom'] / 1e3:.2f} kN) vs "
                    f"{F_req_bare / 1e3:.2f} kN needed → {J['F_bare'] / F_req_bare:.2f}×")
        disc_txt = (f"the review's disc springs (3 × DIN 6796-{t['bell']} in series, {d_trav:.2f} mm travel): life {dF_life / 1e3:.2f} kN but the softer bolts need "
                    f"{dF_req / 1e3:.2f} kN → {dF_life / dF_req:.2f}× (the stiff bolts carried the bending)")
        rows.append(dict(joint=name, case="preload (assembly → life)", verdict=bool(J["F_life"] >= F_req), margin=round(J["F_life"] / F_req, 2),
                         F0=t["preload"], F_life=round(J["F_life"]), F_req=round(F_req), F_bare=round(J["F_bare"]), F_req_bare=round(F_req_bare),
                         disc=dict(F_life=round(dF_life), F_req=round(dF_req)),
                         text=(f"{t['n']} × M{t['d']:.0f} {t['grade']} at {t['preload'] / 1e3:.2f} kN in Ø{t['sleeve'][0]}/{t['sleeve'][1]} steel crush sleeves "
                               f"({J['L_sl']:.1f} mm, {J['k_s'] / 1e3:.0f} kN/mm; the print sandwiched with a {t['squeeze']:.2f} mm squeeze ≈ "
                               f"{E_PRINT * t['squeeze'] / J['L_print']:.1f} MPa that may creep away): metal stack, embedding only → life preload "
                               f"**{J['F_life'] / 1e3:.2f} kN ({100 * J['creep']:.0f} %)** vs {F_req / 1e3:.2f} kN for no gap under the worst service wrench "
                               f"({Mw:.1f} N·m, {Nw:.0f} N) → **{J['F_life'] / F_req:.2f}×**. Before: {bare_txt}. Not chosen: {disc_txt}.")))
        fresh = J["make"](t["preload"], J["u0p"])
        life = J["models"]
        for case, L in loads[name].items():
            N, M = max(L["N"], 0.0), L["M"]
            chk = [m.check(N, M) for m in fresh + life]
            Fb = max(c["F_bolt"] for c in chk)
            sig = max(c["sigma"] for c in chk)
            S = max(c["S"] for c in chk)
            s_sl = S / J["A_s"]
            if case in SERVICE:
                mg = min(m.m_gap(N) for m in life)
                ok = mg >= M and sig <= SIGMA_SERVICE
                margin = min(mg / max(M, 1e-6), SIGMA_SERVICE / max(sig, 1e-6))
                txt = f"no gap up to {mg:.0f} N·m at the life preload; bolt {Fb / 1e3:.2f} kN; print contact {sig:.1f} MPa (≤ {SIGMA_SERVICE:.0f})"
            else:
                gap = any(c["gap"] for c in chk)
                ok = Fb <= J["proof"] and J["strip"] >= 1.5 * Fb and sig <= SIGMA_FAULT and s_sl <= t["Rp"]
                margin = min(J["proof"] / Fb, J["strip"] / (1.5 * Fb), SIGMA_FAULT / max(sig, 1e-6), t["Rp"] / s_sl)
                txt = (f"{'gaps' if gap else 'no gap'}; bolt {Fb / 1e3:.2f} kN vs proof {J['proof'] / 1e3:.2f} kN; thread strip (the weaker of the "
                       f"{J['nut']['what']} and the {t['grade']} {'stud' if name.startswith('pitch') else 'screw'}) {J['strip'] / 1e3:.1f} kN ({J['strip'] / Fb:.2f}× the bolt, need 1.5×); "
                       f"sleeve {s_sl:.0f} MPa (≤ {t['Rp']:.0f}); print contact {sig:.1f} MPa (≤ {SIGMA_FAULT:.0f})")
            rows.append(dict(joint=name, case=case, N=N, M=M, T=L["T"], pose=L["note"], margin=round(margin, 2), verdict=bool(ok), text=txt,
                             before=_before(name, N, M)))
    return rows


# ------------------------------------------------------------------------------------------------ M1: inline roll
# ------------------------------------------------------------------------------------------------ C1 faces (Astra #3 m9)
# A sleeve below its own yield says nothing about the softer face it stands on. Per interface: contact pressure = force /
# the annulus that bears it, against the face material's 0.2 % proof stress R_p (guaranteed minimum): service ≤ R_p (no
# plastic set, so the preload loses only the VDI 2230 asperity embedding already counted), fault ≤ 1.1 R_p (onset of local
# yield under a flat annular punch with the surrounding metal as constraint, ESTIMATE). Forces: bolt force on the head /
# nut face, sleeve force S on the sleeve's two end faces (joint_rows' clamp models, worst of fresh and life preload).
FACE_MAT = {"6061-T6": (240.0, "EN 485-2 EN AW-6061 T6 plate R_p0.2 ≥ 240"), "7075-T6": (460.0, "EN 485-2 EN AW-7075 T651 R_p0.2 ≥ 460"),
            "S235": (235.0, "EN 10025-2 R_eH ≥ 235"), "C45+QT": (490.0, "EN 10083-2 +QT d ≤ 16 R_e ≥ 490"),
            "42CrMo4+QT": (750.0, "EN 10083-3 +QT R_p0.2 ≥ 750"), "ISO 7092 200 HV": (450.0, "≈ 200 HV steel washer, R_p ≈ 450 (ESTIMATE)")}
FAULT_FACE = 1.1
# Fixes (2026-11-28): both clamp rings → C45+QT (2 mm, laser cut from QT strip; were 6061-T6 at the pitch, S235 at the
# yaw); the yaw flange 6061-T6 → 7075-T6 (its spigot top is the sleeves' foot; a hardened washer under each sleeve does not
# fit: Ø6 at r 11 breaks into the Ø16 ribbon bore). Geometry unchanged.
C1_FACES = {
    "pitch hub ↔ tube": [("M4 nut (d_w 5.9) on the clamp ring", "bolt", (5.9, 4.5), "C45+QT", "6061-T6"),
                         ("sleeve end on the clamp ring", "sleeve", (7.0, 4.5), "C45+QT", "6061-T6"),
                         ("sleeve end on the 7075-T6 crank", "sleeve", (7.0, 4.3), "7075-T6", "7075-T6"),
                         ("sleeve ↔ sleeve butt (hub ↔ tube face)", "sleeve", (7.0, 4.3), "42CrMo4+QT", "42CrMo4+QT")],
    "yaw flange ↔ tube": [("M3 head (d_w 5.07) on the clamp ring", "bolt", (5.07, 3.4), "C45+QT", "S235"),
                          ("sleeve end on the clamp ring", "sleeve", (4.6, 3.4), "C45+QT", "S235"),
                          ("sleeve foot on the flange's spigot top", "sleeve", (4.6, 3.2), "7075-T6", "6061-T6")],
}


def ring_squeeze(name):
    """Clamp ring between two studs as a beam on the printed stack (an elastic foundation pre-squeezed by the design squeeze):
    how much of the squeeze it imposes between the studs, and its bending stress, for the ring's material before / after."""
    from .parts import roll, shoulder
    t = shoulder.HUB_TIE if name.startswith("pitch") else roll.YAW_TIE
    r0, r1, th = t["ring"]
    L_p = joint_models()[name]["L_print"]
    span = 2 * math.pi * t["r"] / t["n"]
    w = r1 - r0
    I = w * th ** 3 / 12
    k = E_PRINT / L_p * w                                          # N/mm per mm of ring length
    out = {}
    for lab, E in (("6061-T6", 69e3), ("steel", 205e3)):
        c = span ** 4 / (384 * E * I)
        dlt = c * k / (1 + c * k) * t["squeeze"]
        q = k * (t["squeeze"] - dlt)
        out[lab] = dict(share=1 - dlt / t["squeeze"], sigma=q * span ** 2 / 12 / (w * th ** 2 / 6))
    return out


def face_rows(loads):
    """Per C1 joint and interface: contact pressure at the worst service and fault rows, before / after the fixes."""
    rows = []
    JM = joint_models()
    for name, J in JM.items():
        t = J["t"]
        fresh, life = J["make"](t["preload"], J["u0p"]), J["models"]
        Fsv = Ssv = Fft = Sft = 0.0
        for case, L in loads[name].items():
            N, M = max(L["N"], 0.0), L["M"]
            chk = [m.check(N, M) for m in fresh + life]
            Fb, S = max(c["F_bolt"] for c in chk), max(c["S"] for c in chk)
            if case in SERVICE:
                Fsv, Ssv = max(Fsv, Fb), max(Ssv, S)
            else:
                Fft, Sft = max(Fft, Fb), max(Sft, S)
        for lab, kind, (do, di), mat, before in C1_FACES[name]:
            A = math.pi / 4 * (do ** 2 - di ** 2)
            Fs, Ff = (Fsv, Fft) if kind == "bolt" else (Ssv, Sft)
            ps, pf = Fs / A, Ff / A
            r = dict(joint=name, face=lab, A=A, p_service=ps, p_fault=pf)
            for tag, m in (("after", mat), ("before", before)):
                if m is None:
                    continue
                Rp = FACE_MAT[m][0]
                r[tag] = dict(mat=m, Rp=Rp, sf=min(Rp / ps, FAULT_FACE * Rp / pf), ok=ps <= Rp and pf <= FAULT_FACE * Rp)
            rows.append(r)
    return rows


def face_report(loads):
    from .parts import roll, shoulder
    L = ["", "## C1: contact faces, clamp rings, sleeve lengths (Astra #3 major 9)", "",
         f"Contact pressure on each face that a bolt head, nut or sleeve end stands on, at the worst service and fault rows above, "
         f"against the face's R_p (service ≤ R_p: no plastic set; fault ≤ {FAULT_FACE} R_p: onset of local indentation, ESTIMATE). "
         "Sources: " + "; ".join(f"{k} {v[1]}" for k, v in FACE_MAT.items()) + ".", "",
         "| Joint | Face | Area | p service / fault | Before | After | |", "|---|---|---|---|---|---|---|"]
    for r in face_rows(loads):
        b, a = r.get("before"), r.get("after")
        fb = f"{b['mat']}: {b['sf']:.2f}× {'✅' if b['ok'] else '❌'}" if b else "—"
        fa = f"{a['mat']}: {a['sf']:.2f}×" if a else "(removed)"
        L.append(f"| {r['joint']} | {r['face']} | {r['A']:.1f} mm² | {r['p_service']:.0f} / {r['p_fault']:.0f} MPa | {fb} | {fa} | "
                 f"{('✅' if a['ok'] else '❌') if a else ''} |")
    for name in ("pitch hub ↔ tube", "yaw flange ↔ tube"):
        rs = ring_squeeze(name)
        t = shoulder.HUB_TIE if name.startswith("pitch") else roll.YAW_TIE
        L.append(f"- {name}: clamp ring r {t['ring'][0]}–{t['ring'][1]} × {t['ring'][2]:.0f} between studs on the print (beam on an elastic "
                 f"foundation): it imposes {rs['6061-T6']['share']:.0%} of the {t['squeeze']} mm squeeze between studs in 6061 "
                 f"({rs['6061-T6']['sigma']:.0f} MPa), {rs['steel']['share']:.0%} in steel ({rs['steel']['sigma']:.0f} MPa). The sleeves and the nut "
                 "or head are coaxial, so the clamp load goes through the ring's thickness, not its span; only the print's light squeeze "
                 "bends it, and the service rows already take that squeeze as crept away.")
    JM = joint_models()
    L += [f"- Squeeze, one dimension: the pitch's two sleeves per stud (tube + hub) are faced so that TOGETHER they are "
          f"{shoulder.HUB_TIE['squeeze']} ± 0.02 mm shorter than the measured tube + hub print stack (the tube sleeve to the tube "
          "print − 0.025, then the hub sleeve to the remainder, micrometer); before, each was specified 0.05 short, i.e. 0.10 in "
          f"total against the model's 0.05. The yaw has one sleeve per screw: {roll.YAW_TIE['squeeze']} ± 0.02 under the print, the "
          "same micrometer check.",
          "- Unequal sleeves and preload scatter: each stud's nut closes its own bolt ↔ sleeve loop, so a ± 0.02 mm sleeve-length "
          "spread changes only the local print squeeze, not the sleeve preload. The real scatter is the tightening: a click "
          "wrench gives ± 20 % (VDI 2230 α_A ≈ 1.6, ESTIMATE). With every bolt at −20 % the service preload margins become "
          + "; ".join(f"{n} {J['F_life'] * 0.8 / required_preload(J, 0.0, max(L_['M'] for c, L_ in loads[n].items() if c in SERVICE)):.2f}×"
                      for n, J in JM.items())
          + " (no gap); a single low bolt is between that and nominal.",
          "- Not drawn yet (arm track): the upper bone ↔ yaw flange screws, the detach clamp, the elbow core / clamp and the wrist "
          "output bearing in the same wrench chain — those rows stay open (review major 9)."]
    return L


def huth_k(t1, E1, t2, E2, d, E3=E_STEEL, n=1, a=2 / 3, b=3.0):
    """Shear stiffness (N/mm) of a pin/bolt joining two plates, Huth (1986, ASTM STP 927; b = 3.0 bolted metallic)."""
    C = ((t1 + t2) / (2 * d)) ** a * (b / n) * (1 / (t1 * E1) + 1 / (n * t2 * E2) + 1 / (2 * t1 * E3) + 1 / (2 * n * t2 * E3))
    return 1 / C


# the motor's output-boss dowels (Neon finding 6a). sb: bearing allowable in the unconfirmed boss material (cast A380-class
# σ_y ≈ 160 MPa, no bearing factor: the 240 MPa of wave 3 applied MMPDS's wrought 6061-T6 e/D ≥ 1.5 bearing yield to a
# casting nobody has confirmed; reverted to roll_inline.md's 150). Load sharing: the first dowel to touch carries
# k·δ before the other two engage. δ: H7 hole / m6 dowel spread 0.008 mm (ISO 286: Ø4 H7 0/+12 µm, m6 +4/+12 µm) once the
# dowels are bonded (Loctite 638) into Ø4.1 holes of the adapter while seated in the motor's holes, so the adapter's own
# hole positions drop out; dry dowels in jig-bored adapter holes: + ±0.02 true position on each side (ESTIMATE) → 0.04.
DOWEL = dict(sb=150.0, d_fit=0.008, d_dry=0.040, mu=0.15, pre=2000.0)
SKF_61806 = dict(C=4.49e3, C0=2.9e3)      # SKF catalogue, 61806 (30 × 42 × 7)
S0_SERVICE, S0_FAULT = 1.5, 1.0           # static safety C0/P0: slowly oscillating (SKF guideline 1–2, as rnd/balancer.py), fault 1.0


def roll_rows(loads=None):
    from .params import act
    from .parts import roll
    M = roll.MOT36
    stall, limit = act("GIM8108-36")["peak"], act("GIM8108-36")["rated"]
    r0 = M["out_pcd"] / 2000
    rows = []
    lb = loads["roll bearings"] if loads else {}
    T_bump = max([v["T"] for c, v in lb.items() if c.startswith("bump")] or [0.0])
    # (a) output boss ↔ 7075 adapter: the dowels carry the torque, the 3 × M4 csk friction (metal clamp, no print) helps
    dow_shear = math.pi * 2.0 ** 2 * 450.0                              # ISO 8734 Ø4: min double shear 11.3 kN → 5.65 kN single
    k = huth_k(M["dowel_l"], 70e3, roll.ADAPT["t"], 71e3, M["dowel_d"])
    Fa = M["dowel_d"] * M["dowel_l"] * DOWEL["sb"]                     # bearing allowable per dowel
    n = M["dowel_n"]
    T_bear = lambda dl: n * (Fa - (n - 1) / n * k * dl) * r0         # most-loaded dowel = F_avg + (n−1)/n·k·δ reaches Fa
    strip = thread_strip(4.0, 0.7, M["out_tap"] - 1.0 - 0.35, TAU["cast Al (housing)"], TAU["10.9"])   # M4 × 10 csk: 3.0 into the 4 mm tapped, − chamfer
    fr = M["out_n"] * DOWEL["pre"] * DOWEL["mu"] * r0
    cap, cap_dry, cap_even = T_bear(DOWEL["d_fit"]) + fr, T_bear(DOWEL["d_dry"]) + fr, n * Fa * r0 + fr
    rows.append(dict(item="M1 output boss ↔ 7075 adapter (torque path)",
                     before="dowels alone 164 N·m = 1.60× stall at a 240 MPa bearing allowable, equal sharing assumed",
                     after=(f"{cap:.0f} N·m = {cap / stall:.2f}× the {stall:.1f} N·m stall, {cap / limit:.1f}× the {limit:.1f} N·m drive limit (dowels bonded into the "
                            f"adapter; dry: {cap_dry:.0f} N·m = {cap_dry / stall:.2f}×)"),
                     ok=cap >= 1.0 * stall and n * dow_shear * r0 >= 1.5 * stall, margin=cap / stall,
                     basis=(f"bearing allowable {DOWEL['sb']:.0f} MPa (reverted from 240: cast A380-class σ_y ≈ 160 MPa with no bearing factor, ESTIMATE until the seller "
                            f"names the boss material; a steel or wrought 6061-T6 boss would justify ≈ 240). {n} × Ø4 × {M['dowel_l']:.0f} mm on r {r0 * 1000:.2f}: "
                            f"{Fa / 1e3:.2f} kN each; dowel stiffness {k / 1e3:.0f} kN/mm (Huth, Ø4 steel in 4 mm Al + 7 mm 7075); the first dowel takes k·δ before the "
                            f"others touch: δ {DOWEL['d_fit'] * 1000:.0f} µm (H7/m6 spread, ISO 286) when the dowels are bonded (Loctite 638) into Ø4.1 adapter holes "
                            f"while seated in the motor (the adapter's hole positions drop out) → {T_bear(DOWEL['d_fit']):.0f} N·m; dry in jig-bored holes δ ≈ "
                            f"{DOWEL['d_dry'] * 1000:.0f} µm (± 0.02 true position each side, ESTIMATE) → {T_bear(DOWEL['d_dry']):.0f} N·m (the review's ≈ 55 N·m); equal "
                            f"sharing {n * Fa * r0:.0f} N·m. + 3 × M4 × 10 csk at {DOWEL['pre'] / 1e3:.1f} kN, μ {DOWEL['mu']}, metal on metal (no creep): {fr:.0f} N·m "
                            f"(strip of {M['out_tap'] - 1.35:.2f} mm effective M4 in the cast boss ≈ {strip / 1e3:.1f} kN). Dowel shear {n * dow_shear * r0:.0f} N·m (ISO 8734). "
                            f"Criterion: stall is a fault, checked at a yield-level bearing allowable at ≥ 1.0× (as bolts ≤ proof); "
                            f"the 7075 adapter side is {roll.ADAPT['t']:.0f} mm deep. The 140 N bump puts up to {T_bump:.0f} N·m on the roll output "
                            f"({T_bump / cap:.2f}× this capacity): below the stall only if the actuator backdrives at its current limit (open: backdrive torque, seller / bench)")))
    # (b) rear mount: metal clamp stack (plate + sleeves + motor), the print is only sandwiched
    rs = dict(roll.REAR_SCREW, sleeve_od=SLEEVE_STOCK["rear"]["sleeve"][0], sleeve_id=SLEEVE_STOCK["rear"]["sleeve"][1])
    c_b = rs["grip"] / (E_STEEL * 8.78)
    c_s = (rs["grip"] - rs["plate_t"]) / (E_STEEL * math.pi / 4 * (rs["sleeve_od"] ** 2 - rs["sleeve_id"] ** 2))
    c_pl = rs["plate_t"] / (E_AL * math.pi / 4 * (9.0 ** 2 - 4.5 ** 2))
    F_life = life_preload(rs["preload"], c_b + c_s + c_pl, 0.0, n_metal_if=5, phi=0.0)     # head/plate, plate/sleeve, sleeve/motor, thread ×2 (VDI 2230)
    T_fr = rs["n"] * F_life * rs["mu"] * M["rear_pcd"] / 2000
    L_eng = rs["engage"]
    worst = rs["L_tol"] + rs["plate_tol"] + rs["sleeve_tol"]
    strip_r = thread_strip(4.0, 0.7, L_eng - worst - 0.35, TAU["cast Al (housing)"], TAU["10.9"])
    x1, x2 = roll_spans()
    x_face = roll.X_MOT[0] / 1000
    R2_max = max([v["R2"] for v in lb.values()] or [300.0])
    M_face = R2_max * (x2 - x_face)
    phi_l = (c_s + c_pl) / (c_b + c_s + c_pl)                                          # load factor of a sleeved metal joint (VDI 2230, load at the interface)
    M_open = F_life * rs["n"] * M["rear_pcd"] / 2000 / 2 / (1 - phi_l)
    F_bolt = rs["preload"] + (phi_l if M_face < M_open else 1.0) * M_face / (rs["n"] * M["rear_pcd"] / 2000 / 2)
    sl_bear = stall / (rs["n"] * M["rear_pcd"] / 2000) / (rs["sleeve_od"] * (rs["grip"] - rs["plate_t"]))
    # D26 stock-tube sleeves: their own yield (rear: the worst bolt force; clevis M5: preload + the largest output moment
    # of any case shared over the 8 on PCD ADAPT, as a bolt ring)
    A_rs = math.pi / 4 * (rs["sleeve_od"] ** 2 - rs["sleeve_id"] ** 2)
    s_rs = F_bolt / A_rs
    oc, ck = roll.OUT_CLAMP, SLEEVE_STOCK["clevis"]
    M_out = max([v["Mm"] for v in lb.values()] or [100.0])
    F_ck = oc["preload"] + M_out * 1000 / (roll.ADAPT["n"] * roll.ADAPT["pcd"] / 2 / 2)
    s_ck = F_ck / (math.pi / 4 * (ck["sleeve"][0] ** 2 - ck["sleeve"][1] ** 2))
    sl_ok = s_rs <= SLEEVE_STOCK["rear"]["Rp"] and s_ck <= ck["Rp"]
    rows.append(dict(item="M1 rear mount: 8 × M4 into the 3.5 mm housing threads",
                     before="8 × M4 × 18 + washer + picked shim through 13.9 mm of PETG at 0.5 kN (4 µm of bolt stretch; friction 38 N·m at × 0.75 creep)",
                     after=(f"metal clamp: {rs['size']} through a {rs['plate_t']:.0f} mm 7075 rear plate and Ø{rs['sleeve_od']:.0f}/{rs['sleeve_id']} steel sleeves onto the "
                            f"motor at {rs['preload'] / 1e3:.1f} kN → {F_life / 1e3:.2f} kN after embedding (no print in the stack); friction {T_fr:.0f} N·m = "
                            f"{T_fr / limit:.2f}× the drive limit; engagement {L_eng:.1f} ± {worst:.2f} (never bottoms in 3.5)"),
                     ok=T_fr >= 1.5 * limit and strip_r >= 1.5 * F_bolt and L_eng + worst < rs["tap"] and L_eng - worst >= 2.5 and sl_ok, margin=T_fr / limit,
                     basis=(f"stack per screw: bolt {c_b * 1e6:.0f} + sleeve {c_s * 1e6:.0f} + plate {c_pl * 1e6:.0f} µm/kN, 5 metal interfaces × "
                            f"{CREEP['embed_metal'] * 1000:.0f} µm embedding (VDI 2230, ESTIMATE) → {100 * F_life / rs['preload']:.0f} % left, no creep term (the printed floor + cup "
                            f"ring are sandwiched between the plate and the motor face, sleeves {rs['sleeve_fit']:+.2f} mm vs the print: located, not clamped). Friction μ "
                            f"{rs['mu']} steel/Al (ESTIMATE). Engagement {rs['L']:.0f} − {rs['grip']:.0f} = {L_eng:.1f} mm, worst ± {worst:.2f} (screw length js15 ± {rs['L_tol']}, ISO 4759-1; "
                            f"plate ± {rs['plate_tol']}, sleeve ± {rs['sleeve_tol']}): {L_eng - worst:.2f}–{L_eng + worst:.2f} in {rs['tap']} mm, so no shim picking; strip at the "
                            f"shortest ({L_eng - worst - 0.35:.2f} mm effective) {strip_r / 1e3:.1f} kN vs the worst bolt {F_bolt / 1e3:.2f} kN ({strip_r / F_bolt:.1f}×; motor "
                            f"bearing {R2_max:.0f} N × {1000 * (x2 - x_face):.0f} mm → {M_face:.0f} N·m on the bolt circle, under the {M_open:.0f} N·m that opens it, load factor {phi_l:.2f}). Stall reaction through the sleeves "
                            f"into the print: {sl_bear:.1f} MPa bearing (Ø{rs['sleeve_od']:.0f} × {rs['grip'] - rs['plate_t']:.0f} mm, 8 sleeves). Sleeve yield (D26 stock tube): "
                            f"rear {SLEEVE_STOCK['rear']['stock']} {s_rs:.0f} MPa at the worst bolt vs {SLEEVE_STOCK['rear']['Rp']:.0f} ({SLEEVE_STOCK['rear']['Rp'] / s_rs:.2f}×); "
                            f"clevis M5 {ck['stock']} {s_ck:.0f} MPa at {oc['preload'] / 1e3:.1f} kN + the {M_out:.0f} N·m worst output moment on PCD {roll.ADAPT['pcd']:.0f} "
                            f"vs {ck['Rp']:.0f} ({ck['Rp'] / s_ck:.2f}×)")))
    # (c) the two-bearing roll support: 61806 (W1) + the motor's output bearing (W2)
    B = roll.ROLL_SUPPORT
    F_mis = (B["e_worst"] * B["k_r"][1], B["e_rss"] * B["k_r"][0])
    sv = [v for c, v in lb.items() if c in SERVICE]
    ft = [v for c, v in lb.items() if c not in SERVICE]
    g = lambda vs, kk: max([v[kk] for v in vs] or [0.0])
    R1s, R1f, R2s, R2f = g(sv, "R1"), g(ft, "R1"), g(sv, "R2"), g(ft, "R2")
    Fs, Ff, Ms, Mf = g(sv, "F"), g(ft, "F"), g(sv, "Mm"), g(ft, "Mm")
    s0s, s0f = SKF_61806["C0"] / (R1s + F_mis[0]), SKF_61806["C0"] / (R1f + F_mis[0])
    fric = 0.5 * 0.0015 * F_mis[0] * BRG_BORE_M
    rows.append(dict(item="M1 roll support: 61806 (rear) + the motor's output bearing",
                     before="61806 on a printed stub, located axially, 68 mm from the motor bearing: hyperstatic (review: binds or idles)",
                     after=(f"61806 floats axially (± {B['float']:.1f} mm on the stub) and is bonded into an oversize seat in situ; the misalignment preload is bounded by "
                            f"the printed support's radial stiffness: ≤ {F_mis[0]:.0f} N (worst) / {F_mis[1]:.0f} N (RSS). 61806 s0 {s0s:.1f} service / {s0f:.1f} fault "
                            f"(need {S0_SERVICE} / {S0_FAULT}); the motor bearing needs ≥ {R2s + F_mis[0]:.0f} N radial in service, {R2f + F_mis[0]:.0f} N at a fault"),
                     ok=s0s >= S0_SERVICE and s0f >= S0_FAULT, margin=min(s0s / S0_SERVICE, s0f / S0_FAULT),
                     basis=(f"reactions from the twin wrench of the roll output (the static roll stage taken off) on supports {1000 * x1:.1f} / {1000 * x2:.1f} mm "
                            f"({1000 * (x2 - x1):.0f} mm apart): service (basket, hug × 1.5) 61806 {R1s:.0f} N, motor {R2s:.0f} N; fault (bump, stall) {R1f:.0f} / {R2f:.0f} N. "
                            f"Misalignment: the stub and the cup's driver pilot are one print (Ø{2 * roll.CUP_R:.1f} pilot on the Ø75 driver, {roll.CUP_R - 37.5:.1f} radial) → static "
                            f"offset ≤ {B['e_worst']:.2f} worst / {B['e_rss']:.2f} RSS mm (pilot fit + print coaxiality 0.1 + motor 0.05, ESTIMATE); the rotating seat's own error "
                            f"(the clevis/arm print loop) is taken out by bonding the outer race (Loctite 660, {roll.SEAT_GAP:.1f} mm radial gap) with the arm bolted at mid-travel. "
                            f"Radial stiffness of the printed rear support {B['k_r'][0] / 1e3:.1f}–{B['k_r'][1] / 1e3:.1f} kN/mm (fea.md: stub 288 N → 0.13 mm, rear arm "
                            f"525 N → 0.09 mm, in series; × 2 upper bound, ESTIMATE). The extra 61806 friction at {F_mis[0]:.0f} N is {fric * 1000:.0f} mN·m (SKF μ 0.0015): "
                            f"it does not bind or blind the collision observer. **Deleting the 61806** (the review's option) needs the motor's output bearing to carry "
                            f"{Fs:.0f} N radial + {Ms:.1f} N·m tilting in service and {Ff:.0f} N + {Mf:.0f} N·m at a fault (about its own centre): only a crossed-roller "
                            f"or a preloaded pair with M0 ≥ {Mf:.0f} N·m qualifies. Seller question still open (output bearing type, C0, M0) — until then the 61806 stays")))
    # (d) hard stops (R17, 2026-11-28): two 7075 dogs on the output adapter against two 7075 blocks on the front stop ring
    # (roll.FRONT_STOP), inside the static front collar. W13's steel bushes on the rear plate are gone (rnd/pinch.py).
    f = roll.FRONT_STOP
    ang = roll.front_stop_angles()
    xb0 = roll.X_MOT[0] + M["body_h"] + f["base_t"]                    # ring top (blocks' root)
    x_lo, xt = xb0 + f["dog_lo"], roll.X_ADAPT[0] - f["top_gap"]       # dog tip, block top: the contact band
    rc = (f["dog"][0] + f["dog"][1]) / 2                                # contact radius (mm)
    F35, Fst = limit / (rc / 1000), stall / (rc / 1000)                 # total, both dogs
    share = 1.0                                                         # the more-loaded dog takes it all (one dog touches first; ESTIMATE, 0.5 nominal)
    b_d, h_d = f["dog"][1] - f["dog"][0], rc * math.radians(f["dog_ang"])
    a_d = roll.X_ADAPT[0] - (x_lo + xt) / 2                             # dog root (adapter back face) → contact centre
    rb = (f["blk"][0] + f["blk"][1]) / 2
    b_b, h_b = f["blk"][1] - f["blk"][0], rb * math.radians(ang["W"])
    a_b = (x_lo + xt) / 2 - xb0                                         # block root (ring top) → contact centre
    s_dog = lambda F: share * F * a_d / (b_d * h_d ** 2 / 6)
    s_blk = lambda F: share * F * a_b / (b_b * h_b ** 2 / 6)
    p_c = lambda F: share * F / (b_d * (xt - x_lo))
    rs8 = 8.78                                                          # M4 stress area, mm²
    T_fr = 8 * rs["preload"] * 0.7 * rs["mu"] * STOP_SCREW_R / 1000      # ring friction at 70 % of the rear-plate preload (embedding, ESTIMATE)
    tau_scr = lambda T: max(0.0, T - T_fr) * 1000 / (8 * STOP_SCREW_R) / rs8
    cand = [("dog", f["Rp"] / s_dog(Fst)), ("block", f["Rp"] / s_blk(Fst)), ("contact", f["Rp"] / p_c(Fst)), ("ring screws", 0.6 * 940.0 / max(tau_scr(stall), 1e-6))]
    mg = min(v for _, v in cand)
    rows.append(dict(item="M1 roll hard stops (both ends)",
                     before=("W13: steel stop bushes (Ø10 × 8 proud) on two of the 8 rear M4, against stainless strips in roll_arm_rear's edges: "
                             "1.81× at stall, but they stood 8 mm proud of the rear plate, the one non-round static feature in front of the yaw hood "
                             "(rnd/pinch.py R17: cover_yaw × roll_gim_housing 0.1 … 8.0 mm)"),
                     after=(f"two {f['mat']} dogs on the output adapter (r {f['dog'][0]:.0f}–{f['dog'][1]:.1f}, {f['dog_ang']:.0f}°) against two blocks on a 3 mm ring "
                            f"under the front collar ({ang['W']:.1f}°), 180° apart (a couple: no radial load on the motor bearing): at stall, one dog taking it all, "
                            f"dog {s_dog(Fst):.0f} MPa, block {s_blk(Fst):.0f} MPa, contact {p_c(Fst):.0f} MPa, ring screws {tau_scr(stall):.0f} MPa → {mg:.2f}× (worst of four)"),
                     ok=mg >= 1.5, margin=mg,
                     basis=(f"F = τ / r: {F35:.0f} N at {limit:.1f} N·m, {Fst:.0f} N at stall (r {rc:.2f}, both dogs; the check puts it all on one). Contact band "
                            f"x {x_lo:.1f}–{xt:.1f} ({xt - x_lo:.1f} mm) × {b_d:.1f} mm radial. Dog: cantilever from the adapter's back face, {a_d:.2f} mm to the "
                            f"contact centre, root {b_d:.1f} × {h_d:.1f} mm; block: from the ring top, {a_b:.2f} mm, root {b_b:.1f} × {h_b:.1f} mm; 7075-T6 σ_y "
                            f"{f['Rp']:.0f} MPa (as the rear plate), contact against σ_y. Stop angles: blocks at "
                            + ", ".join(f"{a0:.2f}…{a1:.2f}°" for a0, a1 in ang["block"]) + f"; contact at {roll.STOP_AT[0]}° / {roll.STOP_AT[1]}°. Load path: "
                            f"dog → block → ring → {f['csk']} (friction {T_fr:.1f} N·m at 70 % of 1.6 kN each, the rest in shear on the M4 stress area; the "
                            f"Ø60.6 bore on the motor's Ø60 step pilots it) → housing; the adapter ↔ output-boss joint (row (a)) carried the stop torque before too. "
                            f"The printed clevis / yaw block / rear arm no longer carry the motor's stall torque into the stop. 7075 on 7075 at a rare hard-stop "
                            f"contact: hard-anodise both (ESTIMATE; wear coupon)")))
    return rows


BRG_BORE_M = 0.030
STOP_SCREW_R = 42.0                         # mm: the front stop ring's M4 circle (motor PCD 84)


def report(loads=None):
    loads = loads or cases()
    jr, rr = joint_rows(loads), roll_rows(loads)
    from .parts import roll, shoulder
    h, y = shoulder.HUB_TIE, roll.YAW_TIE
    lv = roll.yaw_tie_levels()
    JM = joint_models()
    L = ["# Arm structural joints: C1 tie-rod joints + M1 inline roll (generated by `python -m rnd.joints`)", "",
         __doc__.split("\n\n", 1)[1].strip(), "", "## Hardware", "",
         f"- **Pitch hub ↔ pitch tube** (rnd/parts/shoulder.py HUB_TIE): {h['n']} × M{h['d']:.0f} class {h['grade']} studs (DIN 976-1, cut to {shoulder.hub_tie_length():.1f} mm) "
         f"on r {h['r']} at {h['ang0']:.0f}° + k·90°, Loctite 270 into the {h['crank_mat']} balancer crank (tapped through {h['crank_t']:.0f} mm, stud end flush with its inboard face), "
         f"each in two steel crush sleeves Ø{SLEEVE_STOCK['pitch']['sleeve'][0]} / Ø{SLEEVE_STOCK['pitch']['sleeve'][1]} ({SLEEVE_STOCK['pitch']['mat']}; one through the tube wall r {h['tube_r_in']:.0f}–20, one through "
         f"the hub, butting at the hub ↔ tube face, together {h['squeeze']} ± 0.02 mm shorter than the measured tube + hub print stack), M4 nuts (ISO 4032 class 8, Loctite 243) on a C45+QT clamp ring (was 6061-T6, Astra #3 m9) "
         f"r {h['ring'][0]:.0f}–{h['ring'][1]:.0f} × {h['ring'][2]:.0f} in a {h['recess']:.1f} mm recess of the detach face; {h['preload'] / 1e3:.1f} kN each (≈ 1.3 N·m, ESTIMATE). "
         "Assembly: tube into the inner wall → crank + studs from the inboard side → tube sleeves over the studs → belt → outer wall → hub over the double-D, hub sleeves → ring, nuts.",
         f"- **Yaw flange ↔ yaw tube** (rnd/parts/roll.py YAW_TIE): 7075-T6 flange (was 6061-T6: Astra #3 m9, the sleeve feet on its spigot top; its hex spigot stays the torque path). "
         f"{y['n']} × M{y['d']:.0f} × {y['L']:.0f} ISO 4762 class {y['grade']} go down from the tube's top end in steel crush sleeves Ø{y['sleeve'][0]} / Ø{y['sleeve'][1]} (ring → the "
         f"flange's spigot top, {y['squeeze']} mm shorter than the print), heads on a C45+QT clamp ring (was S235, Astra #3 m9) "
         f"r {y['ring'][0]}–{y['ring'][1]} × {y['ring'][2]:.0f} in a Ø{2 * y['cb_r']:.0f} × {y['cb_depth']:.1f} counterbore (heads {y['top_gap']} mm under the top), through Ø{y['hole']} holes in the "
         f"wall into M3 threads tapped at the hex corners (r {y['r']:.0f}) of the flange's spigot + plate ({lv['engaged']:.1f} mm engaged, Loctite 243); {y['preload'] / 1e3:.1f} kN each "
         "(≈ 0.5 N·m). Tightened from above on the bench, before the yaw block goes onto the roll output.",
         "- Crush sleeves (D26 cost-down: stock tube where its guaranteed yield clears the sleeve stress, cut and faced square by the builder in a "
         "drilled guide block to the measured print − 0.05 ± 0.02, checked with a micrometer): "
         + "; ".join(f"{k} Ø{v['sleeve'][0]} / Ø{v['sleeve'][1]} {v['mat']} (R_p ≥ {v['Rp']:.0f} MPa): {v['stock']}" for k, v in SLEEVE_STOCK.items())
         + ". Was turned 42CrMo4+QT on all four sets ($7 each, 56 per robot); an E235 tube would yield at the pitch / yaw bump and stall rows. The review's DIN 6796 disc springs (McMaster-Carr \"DIN 6796 lock "
         "washers\", norelem 07000, Bossard BN 794; load to flat M3 1.32 / M4 3.77 kN) were evaluated and not used: see the preload rows.",
         "- Not drawn (rnd/parts/arm.py, arm track): the upper bone ↔ yaw flange screws. With the flange in aluminium they should be M4 into tapped holes in the flange, "
         "heads on washers on the bone flange, not inserts.", "",
         "## C1: preload over life", "", CREEP_BASIS + ".", ""]
    L += [f"- **{r['joint']}:** {r['text']} {'✅' if r['verdict'] else '❌'}" for r in jr if r["case"].startswith("preload")]
    L += ["", "## C1: capacity per load case", "",
          "Before = the wave-2 joint (6 × M3 into heat-set inserts, ≈ 1.0 kN pull-out ESTIMATE, force per insert = M/(3r) + N/6, as the review). Service rows "
          "(basket, hug) need no gap at the life preload and print contact ≤ 12 MPa; fault rows (bump, stall) need bolt ≤ proof, thread strip ≥ 1.5 × bolt, "
          "sleeve ≤ its yield, print contact ≤ 40 MPa (worst of the fresh and the life preload).", "",
          "| Joint | Case | Pull-off (N) | Bending (N·m) | Worst pose | Before: inserts | After | Margin | |", "|---|---|---|---|---|---|---|---|---|"]
    L += [f"| {r['joint']} | {r['case']} | {r['N']:.0f} | {r['M']:.1f} | {r['pose']} | {r['before']:.2f}× | {r['text']} | {r['margin']:.2f}× | {'✅' if r['verdict'] else '❌'} |"
          for r in jr if not r["case"].startswith("preload")]
    lb = loads.get("roll bearings", {})
    L += face_report(loads)
    L += ["", "## M1: inline GIM8108-36", "", "| Item | Before | After | |", "|---|---|---|---|"]
    L += [f"| {r['item']} | {r['before']} | {r['after']} | {'✅' if r['ok'] else '❌'} |" for r in rr]
    L += [""] + [f"- **{r['item']}:** {r['basis']}" for r in rr]
    if lb:
        L += ["", "Roll support reactions per load case (N, N·m; worst pose by the motor-bearing load):", "",
              "| Case | 61806 | Motor bearing (with the 61806) | Motor alone: radial | Motor alone: tilting | Roll torque | Pose |", "|---|---|---|---|---|---|---|"]
        L += [f"| {c} | {v['R1']:.0f} | {v['R2']:.0f} | {v['F']:.0f} | {v['Mm']:.1f} | {v['T']:.1f} | {v['note']} |" for c, v in lb.items()]
    L += ["", "Coupons to add (docs/COUPONS.md): a printed tube clamped as wave 3 (no sleeves) at 8.8 MPa, 1000 h at 40 °C with a load washer (measures φ, "
          "the ESTIMATE behind the 'before' rows), the sleeved joint's moment to gap, the 7075 threads of the yaw flange, the front stop's dog "
          "on its block at 3 kN (anodised 7075 pair, 100 hits: wear and dent), the output dowels' slip torque on a seller motor."]
    ok = all(r["verdict"] for r in jr) and all(r["ok"] for r in rr) and all(r["after"]["ok"] for r in face_rows(loads) if "after" in r)
    return "\n".join(L), ok, jr, rr, lb


def main():
    md, ok, jr, rr, lb = report()
    (DOCS / "joints.md").write_text(md + "\n")
    (DOCS / "joints.json").write_text(json.dumps(dict(ok=ok, joints=jr, roll=rr, roll_bearings=lb), default=str, indent=1))
    print(md)
    raise SystemExit(0 if ok else 1)


if __name__ == "__main__":
    main()
