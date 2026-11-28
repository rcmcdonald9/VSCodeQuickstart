"""Motor thermal check: continuous task duty and worst-case hold times (confidence campaign).  python -m rnd.thermal

Model (per motor, first order). Copper loss scales with torque², so the steady winding rise is
ΔT_ss = ΔT_rated,mount · (τ / τ_rated)². τ_rated is the maker's continuous torque, taken to give ΔT_RATED (60 K) with the
motor **open in still air** (ESTIMATE; the maker's test mounting is not published). Heating follows ΔT(t) = ΔT_ss · (1 − e^(−t/TAU)).

**Mounting (Fable review M7, 2026-11-27).** The arm motors do not sit in open air: each is inside a closed 3 mm ASA sleeve
with a printed rear cap and a plastic Oldham coupling, so there is no metal conduction path, and the R17 covers go over
that. The old model added a flat 10 K for this. Now a thermal network: winding → case (a fixed share F_WC of the open-air
rise) → case → ambient, where the open motor has R_open = 1 / (h_o · A_motor) and the sleeved motor
R_sleeve = 1 / (h_i · A_motor) + t / (k_ASA · A_sleeve) + 1 / (h_o · A_sleeve) (trapped air inside, the ASA wall, still air
outside). The ratio K = R_sleeve / R_open scales the case-to-ambient part of the rise:
ΔT_rated,mount = ΔT_RATED · (F_WC + (1 − F_WC) · K). Mitigation row: an **aluminium rear cap** bolted to the motor's rear face
(thermal pad) with a finned face open to the room, a parallel path R_cap = R_contact + 1 / (h_o · A_fins).

Duty cases:
1. **Task day.** The MuJoCo fetch cycle repeated back to back with no rest, in the Monte Carlo worst corner: arms ×1.25,
   joint friction 0.6 N·m, motors at 75 % torque constant (so the same torque needs more current: τ_rated × 0.75), 0.6 kg can.
2. **Worst static hold.** The worst workspace pose with the 1 kg payload, held indefinitely. The result is the time to
   reach the limit.
3. **D20 and B1y (Astra review #3 major 11, 2026-11-28).** The heavy-carry motor sets (params `d20`, `d20_yaw`) in the mount
   the BOM pays for (belt joints: closed sleeve + aluminium finned rear cap; the B1y inline DM-J4340 yaw: bare under the
   R17 cover, no cap): task day, worst 1 kg hold, the R2c hug at the spec 0.25 m and the R2d basket carry (2 min, from
   cold and from the task-day temperature), each with a **sensitivity band** over the network's estimated constants, and
   the number the bench has to measure to keep each pass.
"""
from __future__ import annotations

import json
import math
from pathlib import Path

import numpy as np

from .params import REAR_CAP

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
AMBIENT, T_LIMIT = 30.0, 100.0      # °C: warm room; winding/driver limit (ESTIMATE: GIM/SDC104 class over-temp cut-out ~100 °C)
DT_RATED = 60.0                     # K rise at rated torque, motor open in still air (ESTIMATE)
TAU = 600.0                         # s thermal time constant, 60–80 mm outrunner class (ESTIMATE; longer when sleeved: ignored)
WORST = dict(derate=0.75, mass=1.25, joint_friction=0.6, finger_mu=0.5, calib=np.zeros(6))
# Thermal network constants (all ESTIMATE; the M1 bench logs the drive temperature and a sleeve thermistor to replace them)
H_OUT, H_IN = 12.0, 5.0             # W/m²K: still air outside (convection + radiation); trapped air in a narrow closed gap
K_ASA, T_WALL = 0.17, 0.003         # W/mK, m: ASA sleeve wall
F_WC = 0.30                         # share of the open-air rise that is winding → case (stator to housing)
R_CONTACT = 0.5                     # K/W motor rear face → aluminium cap through a thermal pad, 3 screws
FIN_AREA = 3.0                      # finned cap face = 3 × its disc area open to the room
# D29 #5 (cost_down.md): the rear cap the BOM buys, params PIP_REAR_CAP. "machined": the CNC'd finned 6061 disc (Ø86–96 × 6
# with fins, FIN_AREA / R_CONTACT above). "stock": a stock Ø90 × 10 round LED heatsink (radial fins on one face, the flat LED
# face to the motor), faced flat and drilled to the cap pattern, on the same thermal pad. Its fin area is an ESTIMATE from
# the listing geometry (≈ 36 radial fins × 7 mm × 30 mm, 2 faces, + the base ≈ 3.5 × the disc area); its contact is
# unmeasured (an as-extruded face, faced by the builder), so nominal 1.0 K/W and a wider band; the fin area gets a band too.
CAPS = {"machined": dict(fin=FIN_AREA, r_contact=R_CONTACT, band=dict(r_contact=(0.25, 2.0))),
        "stock": dict(fin=3.5, r_contact=1.0, band=dict(r_contact=(0.5, 3.0), fin=(2.5, 4.5)))}
COVER_DT = 5.0                      # K: local air under the R17 covers above the room (ESTIMATE; the old model's flat 10 K covered sleeve + covers)
# Motor envelope (actuators.yaml / drawings); the sleeve is Ø+6.6 mm and 30 mm longer (cartridge_m: Ø86.6 for the GIM6010)
ENVELOPE = {"GIM6010-8": (0.080, 0.040), "GIM8108-8": (0.096, 0.045), "GIM8108-36": (0.096, 0.057), "DM-J4310-2EC_V1.1": (0.056, 0.046),
            "GIM6010-48": (0.080, 0.0675), "DM-J4340-2EC": (0.057, 0.056), "DM-J8009-2EC": (0.098, 0.0617)}   # the last three: actuators.yaml frames
# Inline motor under the R17 cover (no cartridge sleeve; the housing bolts to a printed seat, so no metal path): the cover
# shell is Ø+20 mm and 40 mm longer than the motor, 2.5 mm ASA (ESTIMATE, D23 covers not drawn yet)
COVER_GAP_D, COVER_GAP_L, T_COVER = 0.020, 0.040, 0.0025
# Sensitivity band (Astra #3 major 11): each estimated constant over a plausible range, all corners combined. Ambient stays
# the 30 °C warm room for the band; a 35 °C hot-room row is reported on top of the band's worst corner.
BAND = dict(h_o=(8.0, 16.0), h_i=(3.0, 8.0), r_contact=(0.25, 2.0), f_wc=(0.20, 0.40), cover_dt=(3.0, 10.0), tau=(400.0, 900.0))
AMBIENT_HOT = 35.0
ACT_OF = {"pitch": "shoulder_pitch", "roll": "shoulder_roll", "yaw": "upper_yaw", "elbow": "elbow", "wpitch": "wrist_pitch"}
CAN_LIMIT = {"PETG": 55.0, "ASA": 75.0}   # long-term wall limits (HDT 70 / 94 °C less creep margin)


def network(name="GIM6010-8", h_o=None, h_i=None, r_contact=None, f_wc=None, mount="sleeve", cap_kind=None, fin=None):
    """Open, mounted and mounted + aluminium cap: case-to-ambient resistance ratios to the open motor, and the share of
    the mounted rise that the sleeve wall sees. mount "sleeve": the closed ASA cartridge sleeve (belt joints); "cover": an
    inline motor bare under the R17 cover. R_open is anchored to the maker's rating (still air at the nominal H_OUT): a
    lower real h_o raises the mount's resistance, not the rating's."""
    ck = CAPS[cap_kind or REAR_CAP]
    h_o = H_OUT if h_o is None else h_o
    h_i = H_IN if h_i is None else h_i
    r_contact = ck["r_contact"] if r_contact is None else r_contact
    fin = ck["fin"] if fin is None else fin
    f_wc = F_WC if f_wc is None else f_wc
    D, L = ENVELOPE.get(name, ENVELOPE["GIM6010-8"])
    A_m = math.pi * D * L + 2 * math.pi * (D / 2) ** 2
    if mount == "cover":
        Ds, Ls, t = D + COVER_GAP_D, L + COVER_GAP_L, T_COVER
    else:
        Ds, Ls, t = D + 0.0066, L + 0.030, T_WALL
    A_s = math.pi * Ds * Ls + 2 * math.pi * (Ds / 2) ** 2
    R_open = 1 / (H_OUT * A_m)
    R_wall = t / (K_ASA * A_s) + 1 / (h_o * A_s)
    R_sl = 1 / (h_i * A_m) + R_wall
    R_cap = r_contact + 1 / (h_o * fin * math.pi * (Ds / 2) ** 2)
    R_sc = 1 / (1 / R_sl + 1 / R_cap)
    R_wc = f_wc / (1 - f_wc) * R_open
    return dict(K=R_sl / R_open, K_cap=R_sc / R_open, R_open=R_open, R_sleeve=R_sl, R_cap=R_cap,
                wall_frac=R_wall / (R_wc + R_sl))


def dt_rated(k, f_wc=None):
    f_wc = F_WC if f_wc is None else f_wc
    return DT_RATED * (f_wc + (1 - f_wc) * k)


def hold_time(dt_ss, ambient=None, cover_dt=None, tau=None, t_start=None):
    """Time (s) for the winding to reach T_LIMIT at a steady rise dt_ss over the local air, from cold (the local air) or
    from a start temperature t_start (°C)."""
    amb = (AMBIENT if ambient is None else ambient) + (COVER_DT if cover_dt is None else cover_dt)
    tau = TAU if tau is None else tau
    T0 = amb if t_start is None else max(t_start, amb)
    Tss = amb + dt_ss
    if T0 >= T_LIMIT:
        return 0.0
    return math.inf if Tss <= T_LIMIT else -tau * math.log((Tss - T_LIMIT) / (Tss - T0))


# ------------------------------------------------------------------------------------------- design mounts + the band
def mount_of(d, joint):
    """(mount, with_cap) the BOM pays for: belt joints sit in the closed sleeve with the aluminium finned rear cap
    (rnd/bom.py, m1_kit AL_CAP); an inline joint is bare under the cover; the wrist DM-J4310 is in its sleeve, no cap."""
    n, r = d.arm.joints[joint]
    if joint == "wrist_pitch":
        return "sleeve", False
    if r == 1.0 and joint != "shoulder_roll":
        return "cover", False
    if joint == "shoulder_roll":
        return "cover", False          # inline GIM8108-36 roll (D24): bare under the shoulder cap
    return "sleeve", True


def winding(name, x2, mount="sleeve", cap=True, duration=None, t_start=None, ambient=None, **kw):
    """Winding temperature (°C) at torque ratio² x2 for a mount: steady (duration None) or after `duration` s from
    t_start (default: cold). kw: h_o, h_i, r_contact, f_wc, cover_dt, tau (BAND keys)."""
    cover_dt = kw.pop("cover_dt", COVER_DT)
    tau = kw.pop("tau", TAU)
    f_wc = kw.get("f_wc")
    nw = network(name, mount=mount, **kw)
    amb = (AMBIENT if ambient is None else ambient) + cover_dt
    Tss = amb + dt_rated(nw["K_cap" if cap else "K"], f_wc) * x2
    if duration is None:
        return Tss
    T0 = amb if t_start is None else t_start
    return Tss + (T0 - Tss) * math.exp(-duration / tau)


def band(name, x2, mount="sleeve", cap=True, duration=None, t_start=None, cap_kind=None):
    """Over every corner of BAND: dict(nom, lo, hi (30 °C room), hot (worst corner, 35 °C room), t_nom / t_lo (time to
    T_LIMIT from cold, steady duties), k_nom / k_hi (mount rise at rated, K)). t_start may be a callable of the corner (the
    task-day temperature in that corner) for a carry started hot."""
    import itertools
    ck = cap_kind or REAR_CAP
    B = {**BAND, **(CAPS[ck]["band"] if cap else {})}                  # D29 #5: the stock heatsink widens r_contact and adds a fin-area band
    keys = [k for k in B if not (k in ("r_contact", "fin") and not cap)]
    split = lambda kw: ({k: v for k, v in kw.items() if k not in ("cover_dt", "tau")}, kw.get("cover_dt", COVER_DT), kw.get("tau", TAU))

    def one(kw, ambient=AMBIENT, shift=0.0):
        kw = {**kw, "cap_kind": ck}
        ts = t_start(kw) if callable(t_start) else t_start
        ts = None if ts is None else ts + shift
        T = winding(name, x2, mount, cap, duration, ts, ambient=ambient, **dict(kw))
        nk, cdt, tau = split(kw)
        rise = dt_rated(network(name, mount=mount, **nk)["K_cap" if cap else "K"], nk.get("f_wc"))
        t = hold_time(rise * x2, ambient=ambient, cover_dt=cdt, tau=tau) if duration is None else None
        return T, t, rise
    T0, t0, k0 = one({})
    out = dict(nom=T0, t_nom=t0, k_nom=k0, lo=math.inf, hi=-math.inf, hot=-math.inf, t_lo=math.inf, k_hi=0.0)
    for c in itertools.product(*(B[k] for k in keys)):
        kw = dict(zip(keys, c))
        T, t, k = one(kw)
        Th, _, _ = one(kw, AMBIENT_HOT, AMBIENT_HOT - AMBIENT)
        out.update(lo=min(out["lo"], T), hi=max(out["hi"], T), hot=max(out["hot"], Th), k_hi=max(out["k_hi"], k))
        if t is not None:
            out["t_lo"] = min(out["t_lo"], t)
    if duration is not None:
        out["t_nom"] = out["t_lo"] = None
    return out


def continuous(rated, rise, ambient=AMBIENT, cover_dt=COVER_DT):
    """Joint torque that settles at T_LIMIT in a mount whose winding rise at rated torque is `rise` (K)."""
    return rated * math.sqrt((T_LIMIT - ambient - cover_dt) / rise)


def allowed_rise(x2, ambient=AMBIENT, cover_dt=COVER_DT):
    """The winding rise at rated torque (K, motor in its final mount, at equilibrium) that keeps a duty of torque ratio²
    x2 at or under T_LIMIT: what the bench measures (winding resistance, or the drive's estimate, against the room)."""
    return (T_LIMIT - ambient - cover_dt) / max(x2, 1e-9)


def r_contact_max(name, x2, mount="sleeve", ambient=AMBIENT, **kw):
    """Largest cap contact resistance (K/W) that keeps the steady duty x2 at T_LIMIT (nominal other constants), or None if
    even a perfect contact does not, or inf if the mount without the cap passes."""
    lo, hi = 0.0, 50.0
    f = lambda r: winding(name, x2, mount, True, ambient=ambient, r_contact=r, **kw) - T_LIMIT
    if f(lo) > 0:
        return None
    if f(hi) <= 0:
        return math.inf
    for _ in range(60):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if f(mid) <= 0 else (lo, mid)
    return lo


def design_duties(d, label, basket_rows=None, options=True, basket70=None, cap_kinds=(REAR_CAP,)):
    """Task day (MuJoCo fetch cycle, Monte Carlo worst corner), worst 1 kg hold, the R2c hug at 0.25 m and the R2d basket
    carry for design d, per big joint, in the BOM's mount (and, options=True, an inline joint with an aluminium finned
    ring / cap through its cover as the mitigation row). Returns rows (dict)."""
    from . import sim
    from .payload_study import LB20, capacity, torques
    from .spring import workspace_margins
    Lm = sim.limits(d)
    r = sim.run(d, video=False, can_mass=0.6, perturb=WORST)
    task = {}
    for n, f in r["force"].items():
        j = n[2:].replace("_r", "")
        if j in ACT_OF:
            task[ACT_OF[j]] = (math.sqrt(np.mean(np.square(f))), Lm[n][0] * WORST["derate"])
    wm = workspace_margins(d)
    hug_t = torques(d, 0.0, math.pi / 2, LB20 / 2, 0.25)
    rows = []
    for j in ("shoulder_pitch", "shoulder_roll", "upper_yaw", "elbow"):
        n = d.arm.joints[j][0]
        mount0, cap0 = mount_of(d, j)
        mounts = [(mount0, cap0)] + ([(mount0, True)] if options and mount0 == "cover" and j != "shoulder_roll" else [])
        for mount, cap in mounts:
            rms, rated_d = task[j]
            x2_task = (rms / rated_d) ** 2
            task_T = lambda kw, x2=x2_task, m_=mount, c_=cap: winding(n, x2, m_, c_, **(kw or {}))
            rated = capacity(d, j)[2]
            cases = [("task day (non-stop fetch, worst corner)", x2_task, None, None, rms, rated_d)]
            cases.append(("worst 1 kg workspace hold", (wm[j]["max"] / wm[j]["rated"]) ** 2, None, None, wm[j]["max"], wm[j]["rated"]))
            if j in hug_t:
                cases.append(("R2c hug 0.25 m, held", (hug_t[j] / rated) ** 2, None, None, hug_t[j], rated))
            for lab_b, rows_b in (("R2d basket carry 2 min, from the task-day temperature", basket_rows),
                                  ("R2d basket carry 2 min at a 70/30 share, heavier hand, from the task-day temperature", basket70)):
                if not rows_b:
                    continue
                br = [b for b in rows_b if b.get("t") and b["rule"] == "carry" and b["x"] <= 0.45 and "washer" not in b["pose"] and "0.80" not in b["pose"]]
                key = {"shoulder_pitch": "pitch", "shoulder_roll": "roll", "upper_yaw": "yaw", "elbow": "elbow"}[j]
                if br:
                    tb = max(b["t"][key] for b in br)
                    cases.append((lab_b, (tb / rated) ** 2, 120.0, task_T, tb, rated))
            for name, x2, dur, ts, tq, rt in cases:
                for ck in (cap_kinds if cap else (REAR_CAP,)):
                    b = band(n, x2, mount, cap, dur, ts, cap_kind=ck)
                    ml = mount + ((" + Al cap" if ck == "machined" else " + stock heatsink") if cap else "") + ("" if (mount, cap) == (mount0, cap0) else " (option)")
                    rows.append(dict(design=label, joint=j, act=n, mount=ml, case=name, torque=tq, rated=rt, x=math.sqrt(x2), cap_kind=ck if cap else None, **b,
                                     cont_nom=continuous(rt, b["k_nom"]), cont_lo=continuous(rt, b["k_hi"]),
                                     rise_allowed=allowed_rise(x2) if dur is None else None,
                                     rc_max=r_contact_max(n, x2, mount, cap_kind=ck) if (cap and dur is None) else None))
    return rows


def cap_compare(rows):
    """D29 #5: the capped rows with the machined cap vs the stock heatsink (same duty, same band corners otherwise).
    Pass = no row that is within the limit with the machined cap (nominal, and at the band's 30 °C worst) leaves it with the
    stock heatsink, and the task day stays within the limit at the band's worst. Returns (pairs, verdict dict)."""
    key = lambda r: (r["design"], r["joint"], r["mount"].replace(" + stock heatsink", " + Al cap"), r["case"])
    m = {key(r): r for r in rows if r.get("cap_kind") == "machined"}
    s = {key(r): r for r in rows if r.get("cap_kind") == "stock"}
    pairs = [(m[k], s[k]) for k in m if k in s]
    new_nom = [(a, b) for a, b in pairs if a["nom"] <= T_LIMIT < b["nom"]]
    new_hi = [(a, b) for a, b in pairs if a["hi"] <= T_LIMIT < b["hi"]]
    task = [b for _, b in pairs if b["case"].startswith("task day")]
    ok = not new_nom and not new_hi and all(b["hi"] <= T_LIMIT for b in task)
    return pairs, dict(ok=ok, n=len(pairs), new_nom=len(new_nom), new_hi=len(new_hi), task_hi_max=max((b["hi"] for b in task), default=None),
                       d_nom_max=max((b["nom"] - a["nom"] for a, b in pairs), default=0.0), d_hi_max=max((b["hi"] - a["hi"] for a, b in pairs), default=0.0),
                       worst=max(pairs, key=lambda p: p[1]["hi"] - p[0]["hi"]) if pairs else None)


def design_section(rows):
    fmt = lambda t: "—" if t is None else ("unlimited" if math.isinf(t) else f"{t / 60:.0f} min")
    ok = lambda T: "✅" if T <= T_LIMIT else "❌"

    def rc(r):
        if "cap" not in r["mount"] or r["rise_allowed"] is None:
            return "—"
        v = r["rc_max"]
        return "no cap passes" if v is None else ("any (no cap needed)" if math.isinf(v) else f"≤ {v:.2f} K/W")

    def need(r):
        if r["rise_allowed"] is None:
            return "—"
        return "not binding (> 250 K)" if r["rise_allowed"] > 250 else f"≤ {r['rise_allowed']:.0f} K (model {r['k_nom'] * r['x'] ** 2 / max(r['x'] ** 2, 1e-9):.0f} K, band ≤ {r['k_hi']:.0f} K)"
    def cont(r):
        if r["rise_allowed"] is None:
            return "— (2-min duty)"
        if r["torque"] < 0.05 * r["rated"]:
            return f"{r['cont_nom']:.1f} / {r['cont_lo']:.1f}"
        return f"{r['cont_nom']:.1f} / {r['cont_lo']:.1f} ({r['cont_nom'] / r['torque']:.2f}× / {r['cont_lo'] / r['torque']:.2f}× the duty)"
    L = ["| Design | Joint (actuator, mount) | Duty | τ / rated (N·m) | **Winding, nominal** | Band at 30 °C (lo–hi) | Band worst, 35 °C room | "
         "Time to 100 °C from cold: nominal / band worst | Continuous torque in this mount: nominal / band worst (N·m) | "
         "Bench must show: winding rise at rated torque in this mount | Cap contact resistance needed |",
         "|---|---|---|---|---|---|---|---|---|---|---|"]
    for r in rows:
        L.append(f"| {r['design']} | {r['joint']} ({r['act']}, {r['mount']}) | {r['case']} | {r['torque']:.1f} / {r['rated']:.1f} | "
                 f"**{r['nom']:.0f} °C** {ok(r['nom'])} | {r['lo']:.0f}–{r['hi']:.0f} °C {ok(r['hi'])} | {r['hot']:.0f} °C {ok(r['hot'])} | "
                 f"{fmt(r['t_nom'])} / {fmt(r['t_lo'])} | {cont(r)} | "
                 f"{need(r)} | {rc(r)} |")
    return L


def designs():
    """(label, design, basket rows) for B0 (default), D20 (B1) and B1y: the basket rows come from docs/rnd/basket.json
    (rnd/basket.py writes the D20 and B1y equal-share runs) when it exists."""
    from .params import BASELINE, with_arm_motors
    bj = json.loads((DOCS / "basket.json").read_text()) if (DOCS / "basket.json").exists() else {}
    var = bj.get("variants", {})
    sr = bj.get("share_rows", {})
    return [("B0", BASELINE, None, None), ("B1 (D20)", with_arm_motors(BASELINE, "d20"), var.get("B1") or bj.get("rows"), sr.get("B1", {}).get("0.70")),
            ("B1y", with_arm_motors(BASELINE, "d20_yaw"), var.get("B1y"), sr.get("B1y", {}).get("0.70"))]


def design_text(rows):
    """Reading of section 3 + the bench list."""
    bad = [r for r in rows if r["hi"] > T_LIMIT]
    get = lambda des, j, case, opt=False: next((r for r in rows if r["design"] == des and r["joint"] == j and r["case"].startswith(case)
                                                and (("option" in r["mount"]) == opt)), None)
    P = []
    for des in ("B1 (D20)", "B1y"):
        hug = [get(des, j, "R2c") for j in ("shoulder_pitch", "elbow")]
        hug = [h for h in hug if h]
        bas = [r for r in rows if r["design"] == des and r["case"].startswith("R2d") and "option" not in r["mount"]]
        fails = [r for r in rows if r["design"] == des and r["hi"] > T_LIMIT and "option" not in r["mount"]]
        if not hug:
            continue
        h = min(hug, key=lambda r: r["cont_lo"] / r["torque"])
        P += ["", f"**{des}:** hug at 0.25 m: {h['joint']} {h['nom']:.0f} °C nominal, {h['hi']:.0f} °C band worst ({h['hot']:.0f} °C in a 35 °C room); "
              f"continuous torque in the capped sleeve {h['cont_nom']:.1f} N·m nominal / {h['cont_lo']:.1f} N·m band worst against {h['torque']:.1f} N·m demanded "
              f"(**{h['cont_nom'] / h['torque']:.2f}× / {h['cont_lo'] / h['torque']:.2f}× continuous**; without the cap the sleeve gives "
              f"{continuous(h['rated'], dt_rated(network(h['act'])['K'])):.1f} N·m, the ≈ 13.9 N·m the review quoted). "
              + (f"Basket 2-min carry from hot: worst {max(b['hi'] for b in bas):.0f} °C band worst. " if bas else "")
              + ("Rows that leave the limit in the band: " + "; ".join(f"{r['joint']} {r['case'].split(',')[0]} (nominal {r['nom']:.0f}, band {r['hi']:.0f} °C, "
                                                                       f"{'unlimited' if r['t_lo'] is None or math.isinf(r['t_lo']) else '%.0f min' % (r['t_lo'] / 60)} to the limit at the worst corner)" for r in fails) + "."
                 if fails else "No row leaves the limit anywhere in the band.")]
    yo = get("B1y", "upper_yaw", "worst 1 kg", True)
    yb = get("B1y", "upper_yaw", "worst 1 kg")
    if yo and yb:
        P += ["", f"**B1y yaw (DM-J4340-2EC bare under the cover):** the worst 1 kg hold ({yb['torque']:.1f} of {yb['rated']:.1f} N·m rated) settles at "
              f"{yb['nom']:.0f} °C nominal ({yb['lo']:.0f}–{yb['hi']:.0f} °C band) and reaches 100 °C in {yb['t_nom'] / 60:.0f} min nominal, "
              f"{yb['t_lo'] / 60:.0f} min at the band's worst corner; continuous {yb['cont_nom']:.1f} N·m nominal / {yb['cont_lo']:.1f} band worst. "
              f"With an aluminium finned ring on the motor's rear face through the cover (the belt joints' cap, ≈ $19 a joint): {yo['nom']:.0f} °C nominal, "
              f"{yo['hi']:.0f} °C band worst, continuous {yo['cont_nom']:.1f} / {yo['cont_lo']:.1f} N·m. The yaw's task day and basket duty are "
              "small; the long full-reach hold is what needs either the ring or a planner hold-time derate (the GIM6010 + 2:1 belt it replaces "
              "has 9.5 N·m rated at the joint and its cap)."]
    L = ["", f"**Reading.** Band = every corner of h_o {BAND['h_o'][0]:g}–{BAND['h_o'][1]:g} W/m²K, h_i {BAND['h_i'][0]:g}–{BAND['h_i'][1]:g}, "
         f"cap contact {BAND['r_contact'][0]:g}–{BAND['r_contact'][1]:g} K/W, F_WC {BAND['f_wc'][0]:.0%}–{BAND['f_wc'][1]:.0%}, "
         f"local air +{BAND['cover_dt'][0]:g}…{BAND['cover_dt'][1]:g} K, τ {BAND['tau'][0]:.0f}–{BAND['tau'][1]:.0f} s (2-min carries); room {AMBIENT:.0f} °C, "
         f"and the band's worst corner again in a {AMBIENT_HOT:.0f} °C room. "
         f"{len(bad)} of {len(rows)} duty rows leave the 100 °C limit somewhere in the band: "
         + "; ".join(f"{r['design']} {r['joint']} {r['case'].split(',')[0]} (nominal {r['nom']:.0f} °C, band hi {r['hi']:.0f} °C)" for r in bad[:8])
         + ("" if len(bad) <= 8 else "; …") + ". A pass that holds only at the nominal constants is not a pass yet: it is a number the bench must confirm."]
    return L + P


def main():
    from . import sim
    from .params import BASELINE
    from .spring import workspace_margins
    L = sim.limits()
    r = sim.run(video=False, can_mass=0.6, perturb=WORST)
    rows, worst = [], dict(old=0.0, sleeve=0.0, cap=0.0, wall=0.0, machined=0.0, stock=0.0)
    for n, f in r["force"].items():
        if n == "a_fing_a_r":
            continue
        j = n[2:].replace("_r", "")
        act_name = BASELINE.arm.joints[ACT_OF[j]][0] if j in ACT_OF else ""
        nw = network(act_name or "GIM6010-8")
        rated = L[n][0] * WORST["derate"]
        rms = math.sqrt(np.mean(np.square(f)))
        x2 = (rms / rated) ** 2
        t_old = AMBIENT + 10.0 + DT_RATED * x2
        sleeved = j in ACT_OF
        if sleeved:
            t_sl, t_cap = AMBIENT + COVER_DT + dt_rated(nw["K"]) * x2, AMBIENT + COVER_DT + dt_rated(nw["K_cap"]) * x2
            wall = AMBIENT + COVER_DT + dt_rated(nw["K"]) * x2 * nw["wall_frac"]
        else:                                        # wrist roll servo, lift and wheels: not in an arm sleeve
            t_sl = t_cap = t_old
            wall = AMBIENT
        caps = {ck: (AMBIENT + COVER_DT + dt_rated(network(act_name or "GIM6010-8", cap_kind=ck)["K_cap"]) * x2 if sleeved else t_old) for ck in CAPS}
        worst = {k: max(worst[k], v) for k, v in dict(old=t_old, sleeve=t_sl, cap=t_cap, wall=wall, **caps).items()}
        rows.append((j, act_name, rms, rated, t_old, t_sl, t_cap, wall, sleeved))
    nw = network("GIM6010-8")
    nc = network("GIM6010-8", mount="cover")
    hold = []
    for j, m in workspace_margins().items():
        k = network(m["act"])
        x2 = (m["max"] / m["rated"]) ** 2
        dts, dtc = dt_rated(k["K"]) * x2, dt_rated(k["K_cap"]) * x2
        hold.append((j, m["max"], m["rated"], AMBIENT + COVER_DT + dts, hold_time(dts), AMBIENT + COVER_DT + dtc, hold_time(dtc),
                     m["rated"] * math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(k["K"]))))
    ok = lambda b: "✅" if b else "❌"
    fmt = lambda t: "unlimited ✅" if math.isinf(t) else f"{t / 60:.0f} min"
    Lr = ["# Motor thermal check (generated by `python -m rnd.thermal`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
          f"Ambient {AMBIENT:.0f} °C, limit {T_LIMIT:.0f} °C, ΔT at rated (open, still air) {DT_RATED:.0f} K, τ {TAU:.0f} s; h_o {H_OUT:.0f} / h_i {H_IN:.0f} W/m²K, "
          f"F_WC {F_WC:.0%}, +{COVER_DT:.0f} K local air under the covers, ASA {K_ASA} W/mK × {T_WALL * 1000:.0f} mm (all ESTIMATE until the M1 bench logs a real motor's temperature).", "",
          "| Mounting (GIM6010-8 cartridge) | Case-to-ambient vs open (K) | ΔT at rated | Continuous torque for the 100 °C limit |", "|---|---|---|---|",
          f"| Open in still air + 10 K (the old model) | 1.00 | {DT_RATED + 10:.0f} K | {math.sqrt((T_LIMIT - AMBIENT - 10) / DT_RATED):.2f} × rated |",
          f"| **Closed ASA sleeve, printed cap, plastic coupling (as drawn)** | **{nw['K']:.2f}** | **{dt_rated(nw['K']):.0f} K** | **{math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(nw['K'])):.2f} × rated** |",
          f"| Sleeve + aluminium rear cap, finned face to the room | {nw['K_cap']:.2f} | {dt_rated(nw['K_cap']):.0f} K | {min(1.0, math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(nw['K_cap']))):.2f} × rated |",
          f"| Inline motor bare under the R17 cover (GIM6010-size frame; section 3 uses each motor's own frame) | {nc['K']:.2f} | {dt_rated(nc['K']):.0f} K | {min(1.0, math.sqrt((T_LIMIT - AMBIENT - COVER_DT) / dt_rated(nc['K']))):.2f} × rated |", "",
          "## 1. Task day: fetch cycle back to back, Monte Carlo worst corner", "",
          "| Actuator | RMS torque/force | Rated (derated ×0.75) | Winding, old model (open + 10 K) | **Winding, closed sleeve** | Winding, sleeve + Al cap | Sleeve wall (ASA ≤ 75 °C) |",
          "|---|---|---|---|---|---|---|"]
    Lr += [f"| {j}{f' ({a})' if a else ''} | {rms:.2f} | {rt:.2f} | {to:.0f} °C | **{ts:.0f} °C** {ok(ts <= T_LIMIT)} | {tc:.0f} °C {ok(tc <= T_LIMIT)} | "
           f"{(f'{w:.0f} °C ' + ok(w <= CAN_LIMIT['ASA'])) if sl else 'not in a sleeve (old model)'} |" for j, a, rms, rt, to, ts, tc, w, sl in rows]
    Lr += ["", f"**Hottest winding on a non-stop task day: {worst['sleeve']:.0f} °C in the closed sleeve** (the old open + 10 K model said {worst['old']:.0f} °C; "
           f"with an aluminium rear cap {worst['cap']:.0f} °C; limit {T_LIMIT:.0f} °C). The sleeve wall runs cooler than the old model said "
           f"({worst['wall']:.0f} °C: the trapped air inside holds the heat back), so **the limit is the winding, not the ASA**. "
           "**Day-1 actions (Fable M7):** a 10 k NTC on every GIM sleeve plus the drive's own temperature in the bring-up log; an aluminium "
           "rear cap on the first cartridge (Step 1 of the M1 kit) to measure the difference; vents in the R17 covers over each cap.", "",
           "## 2. Worst static hold (worst workspace pose, 1 kg payload, nominal motors)", "",
           "| Joint | Worst static torque (N·m) | Rated (N·m) | Steady winding, sleeve | Time to the limit | With Al cap | Time to the limit | Continuous torque in the sleeve (N·m) |",
           "|---|---|---|---|---|---|---|---|"]
    Lr += [f"| {j} | {mx:.1f} | {rt:.1f} | {Ts:.0f} °C | {fmt(ts)} | {Tc:.0f} °C | {fmt(tc)} | {cont:.1f} |" for j, mx, rt, Ts, ts, Tc, tc, cont in hold]
    lim = [h[4] for h in hold if not math.isinf(h[4])]
    Lr += ["", ("Every joint can hold its worst pose indefinitely." if not lim else
                f"**Some joints can't hold their worst pose forever in the closed sleeve (shortest {min(lim) / 60:.0f} min).** The planner must "
                "derate long holds at full reach (the driver's thermal model already does this). Normal reaching is far below the worst pose.")]
    # 3. heavy-carry designs in the BOM's mount, with the sensitivity band (Astra #3 major 11)
    allrows = []
    for lab, dd, brows, b70 in designs():
        allrows += design_duties(dd, lab, brows, basket70=b70, cap_kinds=tuple(CAPS))
    drows = [r for r in allrows if r.get("cap_kind") in (None, REAR_CAP)]
    pairs, cv = cap_compare(allrows)
    Lr += ["", "## 3. D20 and B1y in the mount the BOM pays for, with a sensitivity band (Astra review #3 major 11)", "",
           "Belt joints: closed sleeve + aluminium finned rear cap (bom.py / m1_kit AL_CAP). Inline joints (the roll in every design, "
           "the B1y DM-J4340 yaw): bare under the R17 cover (cover Ø+20 mm, +40 mm, 2.5 mm ASA, ESTIMATE), no cap, no metal path. "
           "Task day = section 1's fetch cycle on that design (worst corner, motors at 75 % torque constant). Hug = R2c at the spec "
           "0.25 m (rnd/payload_study.py), equal shares. Basket = the worst R2d carry pose (rnd/basket.py, equal shares), 2 minutes "
           "started from the joint's task-day temperature in the same band corner.", ""]
    Lr += design_section(drows)
    Lr += design_text(drows)
    Lr += ["", "**What the bench must measure to keep these passes** (Step 1 lever rig, then the first arm, motor in its final closed mount "
           "with the cover on, restrained output, at a logged room temperature):",
           "1. **Winding rise at rated torque in the final mount**, at equilibrium (≥ 40 min, 4 τ), by winding resistance or the drive's "
           "estimate, plus case, cap, sleeve wall, cover skin and local air NTCs. Each row's 'bench must show' column is the most that rise "
           f"may be for that duty to stay ≤ {T_LIMIT:.0f} °C in a {AMBIENT:.0f} °C room (+{COVER_DT:.0f} K local air); scale by (τ/τ_rated)² for other loads.",
           "2. **The cap's real contact resistance** (motor rear face → aluminium cap → fins): ΔT across the pad at a known heater or copper "
           "loss. The 'cap contact' column is the largest value that keeps the steady duty under the limit at the nominal h_o.",
           "3. **h_o of the finished cover** (vents open / blocked): cap and skin temperature against room at a known power; the band's "
           f"{BAND['h_o'][0]:g} W/m²K corner is a blocked vent or a cap against a cushion.",
           "4. **The thermal time constant τ** from the heating curve (the 2-min basket rise and the hug hold time scale with it).",
           "5. **Repeat at the depleted-pack voltage** and with the neighbouring drives warm (pitch beside the torso electronics; yaw "
           "under the roll motor for B1y), then the two-minute basket cycle from hot."]
    # 4. D29 #5: stock heatsink vs the machined cap, same duties (cost_down.md; params PIP_REAR_CAP)
    ok = lambda T: "✅" if T <= T_LIMIT else "❌"
    st, mc = CAPS["stock"], CAPS["machined"]
    Lr += ["", f"## 4. D29 #5: stock round heatsink instead of the machined rear cap (PIP_REAR_CAP; section 3 above uses **{REAR_CAP}**)", "",
           f"Machined cap: fin area {mc['fin']:g} × the disc, contact {mc['r_contact']:g} K/W (band {mc['band']['r_contact'][0]:g}–{mc['band']['r_contact'][1]:g}). "
           f"Stock Ø90 × 10 LED heatsink, faced and drilled: fin area {st['fin']:g} × the disc (band {st['band']['fin'][0]:g}–{st['band']['fin'][1]:g}, ESTIMATE from "
           f"the listing geometry), contact {st['r_contact']:g} K/W (band {st['band']['r_contact'][0]:g}–{st['band']['r_contact'][1]:g}: an unmeasured extruded face). "
           "Every other constant and corner as section 3. **Pass** = no row within the limit with the machined cap (nominal or band worst at 30 °C) "
           "leaves it with the stock part, and the task day stays within the limit at the band's worst.", "",
           f"- Section 1 task day (worst corner, nominal constants): hottest winding {worst['machined']:.0f} °C machined → {worst['stock']:.0f} °C stock.",
           f"- Section 3: {cv['n']} capped rows compared; new nominal failures {cv['new_nom']}, new band failures {cv['new_hi']}; task day band worst "
           f"≤ {cv['task_hi_max']:.0f} °C; largest rise {cv['d_nom_max']:+.1f} K nominal, {cv['d_hi_max']:+.1f} K at the band's worst"
           + (f" ({cv['worst'][1]['design']} {cv['worst'][1]['joint']} {cv['worst'][1]['case'].split(',')[0]}: {cv['worst'][0]['hi']:.0f} → {cv['worst'][1]['hi']:.0f} °C)" if cv["worst"] else "") + ".",
           f"- **Thermal re-check: {'PASS' if cv['ok'] else 'FAIL'}.**", "",
           "| Design | Joint | Duty | Machined: nominal / band hi | Stock: nominal / band hi | Contact needed (stock) |", "|---|---|---|---|---|---|"]
    rcf = lambda r: "—" if r["rise_allowed"] is None else ("no cap passes" if r["rc_max"] is None else ("any" if math.isinf(r["rc_max"]) else f"≤ {r['rc_max']:.2f} K/W"))
    Lr += [f"| {a['design']} | {a['joint']} ({a['act']}{', option' if 'option' in a['mount'] else ''}) | {a['case'].split(',')[0]} | {a['nom']:.0f} / {a['hi']:.0f} °C {ok(a['hi'])} | "
           f"{b['nom']:.0f} / {b['hi']:.0f} °C {ok(b['hi'])} | {rcf(b)} |" for a, b in pairs]
    (DOCS / "thermal.md").write_text("\n".join(Lr) + "\n")
    (DOCS / "thermal.json").write_text(json.dumps({"task_max_c": worst["sleeve"], "task_max_c_open_model": worst["old"], "task_max_c_al_cap": worst["cap"],
                                                   "hold_min_s": min(lim) if lim else None, "can_max_c": worst["wall"],
                                                   "k_sleeve": nw["K"], "k_cap": nw["K_cap"], "dt_rated_sleeve": dt_rated(nw["K"]),
                                                   "designs": drows, "band": BAND, "rear_cap": REAR_CAP,
                                                   "d29_stock_cap": dict(ok=cv["ok"], n=cv["n"], new_nom=cv["new_nom"], new_hi=cv["new_hi"],
                                                                         task_hi_max=cv["task_hi_max"], d_nom_max=cv["d_nom_max"], d_hi_max=cv["d_hi_max"],
                                                                         task_day_machined=worst["machined"], task_day_stock=worst["stock"])}, default=float))
    print("\n".join(Lr))


if __name__ == "__main__":
    main()
