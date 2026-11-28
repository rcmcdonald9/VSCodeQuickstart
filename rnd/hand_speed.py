"""People-near hand-speed cap from the directional effective mass (Astra review #3, Part A *Collision*; R17 140 N).
python -m rnd.hand_speed  → docs/rnd/hand_speed.md + hand_speed.json; the cap is sw/safety.py V_HAND_PN / hand_speed_cap().

First impact of a free effective mass m at closing speed v on a body region of stiffness k (the fixed-target spring
model rnd/inline_study.py uses): F = v·√(k·m), so F ≤ F_max needs v ≤ F_max / √(k·m), i.e. m_max = (F_max / v)² / k.
For a hand moving at velocity v = J q̇ the worst contact normal n gives (Cauchy–Schwarz on Λ⁻¹ = J M⁻¹ Jᵀ)
max_n (nᵀv)² / (nᵀ Λ⁻¹ n) = vᵀ Λ v, so F_peak = √(k · q̇ᵀ Jᵀ Λ J q̇). λ_max(Λ) is no use as a bound: near a folded
elbow or a straight arm it runs to tens of kg (or ∞), along a direction the hand can only move in with large joint
rates. So the cap is found for the motions the supervisor actually allows: |J q̇| ≤ V (the Cartesian cap) **and**
|q̇_i| ≤ ω (its per-joint slew, PN_JOINT_V in people-near mode). Per pose, max q̇ᵀAq̇ (A = JᵀΛJ) over that set is
taken over 4,000 random joint-rate directions plus A's and (A, JᵀJ)'s eigenvectors, each scaled to the first limit it
meets; V is the largest speed whose worst pose stays ≤ F_max. Poses: the people-near mode's (roll ≥ +8°, the hand inside
PERSON_WS), with the reflected rotor inertias J_rotor·N² as joint armature (rnd/inline_study.py, ESTIMATE rotor inertias),
lift and base held. A payload held at the hand adds its mass (Λ → Λ + m_p·I for a point mass at the grasp site; ≤ that
for the redundant arm): the cap uses m_arm + the whole payload (two-handed loads: the other arm carries it too, so this
is conservative). Closing speed = hand speed relative to the base + the base speed (≤ PERSON_V = 0.1 m/s with a
person near), so the supervisor gives the arm v_cap − |v_base|.

Inputs: F_max 140 N (RND_PLAN R17), k 75 N/mm (hand/finger region, ISO/TS 15066 Annex A as quoted in inline_study.py,
ESTIMATE from memory of the table; a forehead at 150 N/mm needs the cap ÷ √2; the PERSON_WS box keeps the hands
≤ 0.60 m high, below an adult's head but not a small child's). Not covered: contacts on other link sites (elbow, forearm:
smaller lever speeds, larger effective masses), the base's own momentum, clamping (quasi-static) after the first peak.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
F_MAX = 140.0             # N, R17 transient at the hands
K_HAND = 75e3             # N/m (inline_study.K_BODY["hand"])
ARM_J = ("pitch", "roll", "yaw", "elbow", "wpitch", "wroll")
FULL = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}
PAYLOADS = (0.0, 1.0, 2.5, 4.535)
OMEGA = (1.5, 1.0, 0.75, 0.5)   # rad/s per-joint slew in people-near mode: sw/safety.py default v_max 1.5 and lower candidates
N_DIR = 4000


def v_cap(m, f=F_MAX, k=K_HAND):
    return f / math.sqrt(k * m)


class Lam:
    def __init__(self, d, rotors=True):
        import mujoco
        from .params import SPECS
        from .twin import build_xml
        self.mj = mujoco
        m = mujoco.MjModel.from_xml_string(build_xml(d))
        self.m, self.dat = m, mujoco.MjData(m)
        jid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, n)
        self.qa = {j: m.jnt_qposadr[jid(f"{j}_r")] for j in ARM_J}
        self.qa["lift"] = m.jnt_qposadr[jid("lift")]
        self.idx = [m.jnt_dofadr[jid(f"{j}_r")] for j in ARM_J]
        m.dof_armature[:] = 0.0
        if rotors:
            for j, full in FULL.items():
                n, r = d.arm.joints[full]
                m.dof_armature[m.jnt_dofadr[jid(f"{j}_r")]] = SPECS[n].get("rotor_inertia_kgm2", 3e-5) * (SPECS[n]["gear_ratio"] * r) ** 2
        self.site = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, "grasp_r")
        self.pay = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "payload_r")
        self.lift_rng = tuple(m.jnt_range[jid("lift")])

    def at(self, q, payload=0.0):
        """λ_max(Λ) (kg), its direction, and the hand position for the right arm at q (dict, rad)."""
        mj, m, dat = self.mj, self.m, self.dat
        m.body_mass[self.pay] = max(payload, 1e-4)
        dat.qpos[:] = 0
        for j, a in self.qa.items():
            dat.qpos[a] = q[j]
        mj.mj_forward(m, dat)
        M = np.zeros((m.nv, m.nv))
        if hasattr(dat, "qM"):
            mj.mj_fullM(m, M, dat.qM)
        else:
            mj.mj_fullM(m, dat, M)
        Jp = np.zeros((3, m.nv))
        mj.mj_jacSite(m, dat, Jp, None, self.site)
        Ja, Ma = Jp[:, self.idx], M[np.ix_(self.idx, self.idx)]
        Li = Ja @ np.linalg.solve(Ma, Ja.T)
        w, V = np.linalg.eigh(Li)
        self.last = (Ja, np.linalg.inv(Li))
        return float(1.0 / max(w[0], 1e-9)), V[:, 0], dat.site_xpos[self.site].copy()

    def worst_force(self, V, omega, dirs):
        """Peak first-impact force (N) at the last pose over joint rates with |J q̇| ≤ V and |q̇_i| ≤ omega."""
        Ja, Lam = self.last
        A = Ja.T @ Lam @ Ja
        B = Ja.T @ Ja
        extra = [np.linalg.eigh(A)[1].T]
        try:
            import scipy.linalg as sl
            extra.append(sl.eigh(A, B + 1e-9 * np.eye(len(A)))[1].T)
        except Exception:
            pass
        U = np.vstack([dirs] + extra)
        U /= np.linalg.norm(U, axis=1, keepdims=True)
        jv = np.linalg.norm(U @ Ja.T, axis=1)
        s = np.minimum(V / np.maximum(jv, 1e-12), omega / np.max(np.abs(U), axis=1))
        e = np.einsum("ij,jk,ik->i", U, A, U) * s ** 2
        return math.sqrt(K_HAND * float(e.max()))


def sample(L, n, rng, box, roll_lo=8.0):
    from .twin import JR
    lo, hi = map(np.array, box)
    out, tries = [], 0
    while len(out) < n and tries < 400 * n:
        tries += 1
        q = {"lift": rng.uniform(*L.lift_rng)}
        for j in ARM_J:
            a, b = (roll_lo, JR["roll"][1]) if j == "roll" else JR[j]
            if j == "pitch":
                a, b = -150.0, 30.0
            q[j] = math.radians(rng.uniform(a, b))
        _, _, p = L.at(q)
        if np.all(p >= lo) and np.all(p <= hi):
            out.append(q)
    return out


def study(n=600, seed=3):
    from sw import safety as S
    from .inline_study import POSES
    from .params import BASELINE, with_arm_motors
    rng = np.random.default_rng(seed)
    res = {}
    for key, d in (("B0", BASELINE), ("B1", with_arm_motors(BASELINE, "d20"))):
        L = Lam(d)
        qs = sample(L, n, rng, S.PERSON_WS)
        m0 = np.array([L.at(q)[0] for q in qs])
        add = []
        for p in PAYLOADS[1:]:
            mp = np.array([L.at(q, p)[0] for q in qs[:100]])
            add.append(float(np.max(mp - m0[:100] - p)))        # ≤ 0 when the payload adds at most its own mass
        k = int(np.argmax(m0))
        dirs = rng.normal(size=(N_DIR, len(ARM_J)))
        cap = {}
        for om in OMEGA:
            for p in PAYLOADS:
                lo_v, hi_v = 0.01, 1.5
                Fs = None
                for _ in range(18):                              # bisection on V: worst pose ≤ F_MAX
                    v = 0.5 * (lo_v + hi_v)
                    worst = 0.0
                    for q in qs[:200]:
                        L.at(q, p)
                        worst = max(worst, L.worst_force(v, om, dirs))
                        if worst > F_MAX:
                            break
                    lo_v, hi_v = (v, hi_v) if worst <= F_MAX else (lo_v, v)
                cap[f"{om:g}/{p:g}"] = lo_v
        ref = {}
        for pn, pose in POSES.items():
            q = {"lift": 0.5 * sum(L.lift_rng), **{j: math.radians(pose.get(j, 0.0)) for j in ARM_J}}
            ref[pn] = L.at(q)[0]
        res[key] = dict(n=len(qs), m_max=float(m0.max()), m_p99=float(np.percentile(m0, 99)), m_p50=float(np.median(m0)),
                        worst_pose={j: round(math.degrees(v), 1) for j, v in qs[k].items() if j != "lift"} | {"lift": round(qs[k]["lift"], 3)},
                        payload_excess=max(add), ref=ref, cap=cap)
    return res




OMEGA_PN = 0.75           # the per-joint slew chosen for people-near mode (sw/safety.py PN_JOINT_V)


def caps(res, omega=OMEGA_PN):
    """The supervisor's table at the chosen joint slew: (payload kg, closing-speed cap m/s), the lower of B0 / B1, rounded down to 5 mm/s."""
    return [(p, math.floor(min(r["cap"][f"{omega:g}/{p:g}"] for r in res.values()) * 200) / 200) for p in PAYLOADS]


def report(res):
    from sw import safety as S
    cp = caps(res)
    sw_tab = getattr(S, "V_HAND_PN", None)
    L = ["# People-near hand-speed cap (generated by `python -m rnd.hand_speed`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
         f"**Result: with the per-joint slew at {OMEGA_PN} rad/s in people-near mode, the closing speed that keeps the first impact "
         f"≤ {F_MAX:.0f} N at k {K_HAND / 1000:.0f} N/mm is {cp[0][1]:.3f} m/s empty, "
         + ", ".join(f"{v:.3f} m/s with {p:g} kg" for p, v in cp[1:])
         + f" (the lower of today's arm and D20). The arm gets that minus the base speed (≤ {S.PERSON_V} m/s). "
         f"sw/safety.py: PN_JOINT_V = {getattr(S, 'PN_JOINT_V', '?')} rad/s, V_HAND_PN = {sw_tab}.** "
         f"For comparison: at 0.5 m/s the bound allows {(F_MAX / 0.5) ** 2 / K_HAND:.2f} kg (Astra); the reach pose's worst direction is "
         + " / ".join(f"{k} {r['ref'][next(iter(r['ref']))]:.2f} kg → {v_cap(r['ref'][next(iter(r['ref']))]):.3f} m/s" for k, r in res.items()) + ".", "",
         "## Closing-speed cap V (m/s) by per-joint slew ω and payload (worst sampled people-near pose ≤ 140 N)", "",
         "| Arm | ω (rad/s) | " + " | ".join(f"{p:g} kg" for p in PAYLOADS) + " |", "|---|---|" + "---|" * len(PAYLOADS)]
    for k, r in res.items():
        for om in OMEGA:
            L.append(f"| {k} | {om:g}{' ← PN_JOINT_V' if om == OMEGA_PN else ''} | " + " | ".join(f"{r['cap'][f'{om:g}/{p:g}']:.3f}" for p in PAYLOADS) + " |")
    L += ["", "## Effective mass over the people-near poses (no rate limit: why λ_max alone is not the bound)", "",
          "| Arm | Poses (roll ≥ +8°, hand in PERSON_WS) | λ_max(Λ) median / p99 / max (kg) | Worst pose (deg; lift m) | |λ_max-only cap (m/s)| | Payload adds at most its mass (excess, kg) | Reference poses (inline_study.POSES, λ_max) |",
          "|---|---|---|---|---|---|---|"]
    for k, r in res.items():
        ref = "; ".join(f"{pn}: {v:.2f} kg" if v < 1e6 else f"{pn}: ∞ (singular)" for pn, v in r["ref"].items())
        L.append(f"| {k} | {r['n']} | {r['m_p50']:.2f} / {r['m_p99']:.2f} / {r['m_max']:.2f} | {r['worst_pose']} | {v_cap(r['m_max']):.3f} | "
                 f"{r['payload_excess']:+.3f} | {ref} |")
    L += ["", "**Reading.** At the 0.5 m/s the inline study used, 140 N allows only "
          f"{(F_MAX / 0.5) ** 2 / K_HAND:.2f} kg; the arm is 2–4 kg in the reach pose's worst direction (most of it the roll rotor, "
          "0.078 kg·m² reflected at 36:1; D20 adds 0.035 kg·m² at pitch and elbow), so near people the hand moves at about a third to a "
          "half of a brisk reach. The joint-rate limit matters as much as the Cartesian one: at the default 1.5 rad/s slew a folded-elbow "
          "pose can reach 140 N at hand speeds the Cartesian cap would allow (D20: 0.07 m/s), because the rotor energy is in the joint "
          "rates, not the hand speed. A payload adds its mass to every direction, so the cap falls with load. The detector trips after "
          "the first peak (14–18 ms vs a 9–10 ms rise), so this speed, not the trip, bounds the first impact; the trip and the compliant "
          "hold bound what follows. Measure: hand effective mass on the bench (two known added inertias, Astra's test 2) and the "
          "instrumented compliant target at progressively higher speed."]
    return "\n".join(L)


def main():
    res = study()
    (DOCS / "hand_speed.md").write_text(report(res) + "\n")
    (DOCS / "hand_speed.json").write_text(json.dumps(dict(res, omega_pn=OMEGA_PN, caps=caps(res), F_max=F_MAX, k=K_HAND), indent=1))
    print(report(res))


if __name__ == "__main__":
    main()
