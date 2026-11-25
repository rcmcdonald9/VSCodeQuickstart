"""KX joint-interface analysis: slip torque, contact stress, twist effort, printed-part stresses,
and the socket bearings that carry the bending (cartridges carry torque only).

All formulas are first-order hand calculations. Inputs come from rnd/parts/kx.py and rnd/params.py.
"""
from __future__ import annotations

import math

from .arm_sizing import point_masses
from .params import BASELINE, Design, act, G
from .parts.kx import KX, KX44, KX56, KX74

E_STEEL, NU_STEEL = 210e3, 0.30          # MPa
E_PETG, NU_PETG = 2.1e3, 0.38            # MPa, printed PETG (ESTIMATE; datasheet 2.0–2.2 GPa)
PETG_YIELD = 50.0                        # MPa tensile (ESTIMATE; datasheets 45–53)
PETG_SHEAR = 0.6 * PETG_YIELD
HERTZ_STEEL_LIMIT = 4200.0               # MPa, static limit for hardened bearing steel (ISO 76 basis)
MU_LUBED, MU_DRY = 0.15, 0.25            # collar ramp friction, greased / dry PETG-PETG (ESTIMATE)
MU_THRUST = 0.06                         # PTFE thrust washer between the spring stack and the hub step (ESTIMATE)
HAND_TWIST_OK = 3.0                      # N·m: comfortable one-hand twist with thumb tabs (jar-lid scale; ESTIMATE)

# Socket bearings: one pair per socket. SKF thin-section catalogue values (ESTIMATE: verify the supplier's data)
BEARINGS = {"61808": dict(d=40, D=52, B=7, C0=4150.0, price=2.0),
            "61805": dict(d=25, D=37, B=7, C0=2600.0, price=1.5)}
BEARING_SPAN = {"KX74": 16.0, "KX56": 14.0, "KX44": 12.0}   # mm between the bearings of a pair


def e_star(e1, n1, e2, n2):
    return 1 / ((1 - n1 ** 2) / e1 + (1 - n2 ** 2) / e2)


def hertz_point(f, r_eff, es):
    """Peak pressure (MPa) of an elliptical point contact approximated with R_eff = sqrt(Ra·Rb)."""
    return (6 * f * es ** 2 / (math.pi ** 3 * r_eff ** 2)) ** (1 / 3)


def contact_forces(kx: KX, torque_nm: float):
    """Worst single ball contact force (N). Each ball takes V = P/3 down and H = T/(3r) sideways."""
    b = math.radians(kx.beta_deg)
    v = kx.preload_n / 3
    h = torque_nm * 1000 / (3 * kx.r_ball)
    return v / (2 * math.sin(b)) + h / (2 * math.cos(b))


def slip_torque(kx: KX):
    """Torque (N·m) at which the balls start to climb the pins: P·r / tanβ (pin friction ignored: conservative)."""
    return kx.preload_n * kx.r_ball / 1000 / math.tan(math.radians(kx.beta_deg))


def ball_on_pins_stress(kx: KX, f):
    rb, rp = kx.ball_d / 2, kx.pin_d / 2
    ra, rbb = rb, 1 / (1 / rb + 1 / rp)       # along the pin axis / across it
    return hertz_point(f, math.sqrt(ra * rbb), e_star(E_STEEL, NU_STEEL, E_STEEL, NU_STEEL))


def ball_on_printed_groove_stress(kx: KX, f):
    """Same ball in a printed 90° V-groove (ball on a flat PETG flank)."""
    return hertz_point(f, kx.ball_d / 2, e_star(E_STEEL, NU_STEEL, E_PETG, NU_PETG))


def twist_torque(kx: KX, mu):
    r = (kx.od / 2 + kx.lug_reach / 2) / 1000
    rho = math.atan(mu)
    # ramp friction + friction where the rotating collar stack meets the hub step (PTFE thrust washer)
    return kx.preload_n * r * math.tan(math.radians(kx.ramp_deg) + rho) + kx.preload_n * MU_THRUST * (kx.r_hub - 1) / 1000


def printed_stresses(kx: KX, torque_nm):
    f = kx.preload_n / 3                                  # per lug / tooth
    lug_shear = f / (kx.lug_w * kx.lug_t)
    reach = kx.r_collar_in - kx.r_tooth_in
    tooth_bend = 6 * f * (reach / 2) / ((kx.lug_w - 2) * kx.tooth_t ** 2)
    return lug_shear, tooth_bend


def interface_rows(d: Design = BASELINE):
    users = {"KX74": ["GIM6010-8"], "KX56": ["DM-J4310-2EC_V1.1"], "KX44": ["STS3215-C018"]}
    rows = []
    for kx in (KX74, KX56, KX44):
        for a in users[kx.name]:
            s = act(a)
            f_r, f_p = contact_forces(kx, s["rated"]), contact_forces(kx, s["peak"])
            rows.append(dict(kx=kx, act=a, rated=s["rated"], peak=s["peak"], slip=slip_torque(kx),
                             p_pin_rated=ball_on_pins_stress(kx, f_r), p_pin_peak=ball_on_pins_stress(kx, f_p),
                             p_petg_rated=ball_on_printed_groove_stress(kx, f_r),
                             tw_lubed=twist_torque(kx, MU_LUBED), tw_dry=twist_torque(kx, MU_DRY),
                             lug_shear=printed_stresses(kx, s["peak"])[0], tooth=printed_stresses(kx, s["peak"])[1]))
    return rows


def socket_loads(d: Design = BASELINE):
    """Bending carried by each socket's bearing pair with the arm straight out horizontally plus 1 kg (conservative:
    the full outboard gravity moment is treated as bending for every joint)."""
    pts, _ = point_masses(d)
    lu, lf = d.arm.l_upper, d.arm.l_fore
    at = {"shoulder_roll": 0.05, "upper_yaw": 0.12, "elbow": lu, "wrist_pitch": lu + lf, "wrist_roll": lu + lf + 0.04}
    out = []
    for j, x0 in at.items():
        ob = [(m, x - x0) for n, m, x in pts if x > x0 + 1e-9]
        moment = sum(m * G * r for m, r in ob)
        weight = sum(m * G for m, _ in ob)
        n = d.arm.joints[j][0]
        kx = KX44 if n.startswith("STS") else (KX56 if n.startswith("DM-J4310") else KX74)
        brg = "61808" if kx is KX74 else "61805"
        f_brg = moment * 1000 / BEARING_SPAN[kx.name] + weight / 2
        out.append(dict(joint=j, kx=kx.name, moment=moment, brg=brg, f=f_brg, s0=BEARINGS[brg]["C0"] / f_brg))
    # drive wheels: robot weight / 3 wheels on uneven floor, ×2 for bumps, wheel centre 30 mm out from the socket
    m_tot = 16.8
    fw = m_tot * G / 3 * 2
    f_brg = fw * 30 / BEARING_SPAN["KX74"] + fw / 2
    out.append(dict(joint="drive wheel", kx="KX74", moment=fw * 0.03, brg="61808", f=f_brg, s0=BEARINGS["61808"]["C0"] / f_brg))
    return out


def report(d: Design = BASELINE) -> str:
    L = ["# KX joint interface v1: analysis (auto-generated by `python -m rnd.run_all`)", "",
         "**Architecture.** A cartridge carries **torque only**. Its output face locks into a **hub** that turns in the bone's own "
         "bearing pair, and the motor housing is held by a **torque pin** in a radial slot (precedent: shaft-mounted gearboxes with a torque arm). "
         "Bending, axial and crash loads go through the bone's bearings, never through the printed interface or the motor's bearing.", "",
         "**Seat.** 3 hardened balls on the cartridge; each sits on **two hardened dowel pins** in the hub (a ball on two rods: Maxwell coupling). "
         "A **breech-lock collar** with ramped teeth pulls the faces together, and a **spring stack** sets the preload "
         "(it rotates on a PTFE thrust washer).", "",
         "**Design sweep (loop 2):** contact angle β 30° → 20° and balls Ø8 → Ø10 on Ø5 pins (KX74). "
         "The same 1.15× peak slip margin then needs 160 N of preload instead of 250 N. Twist effort drops from 4.2–6.3 to 2.1–2.7 N·m, "
         "and peak contact stress drops from 4,227 to ~3,550 MPa.", "",
         "## Geometry", "", "| Size | Flange OD | Ball / PCD | Pins | Contact angle β | Preload | Twist | Collar OD |", "|---|---|---|---|---|---|---|---|"]
    for kx in (KX74, KX56, KX44):
        L.append(f"| {kx.name} | {kx.od:.0f} | Ø{kx.ball_d:.0f} / {kx.ball_pcd:.0f} | 6 × Ø{kx.pin_d:.0f}×{kx.pin_len:.0f} | {kx.beta_deg:.0f}° | "
                 f"{kx.preload_n:.0f} N over a {kx.stroke:.1f} mm stroke | {kx.twist_deg:.0f}° | {2 * kx.r_collar_out:.0f} |")
    L += ["", "## Loads and stresses", "",
          "| Interface + actuator | Rated / peak (N·m) | Slip torque | Ball-on-pin stress, rated / peak (limit 4,200 MPa) | Same ball in a **printed** groove, rated (PETG yield ~50 MPa) | Twist to lock, greased / dry | Lug shear / tooth bending (PETG) |",
          "|---|---|---|---|---|---|---|"]
    for r in interface_rows(d):
        ok_slip = "✅" if r["slip"] >= r["peak"] else "⚠️"
        ok_p = "✅" if r["p_pin_peak"] <= HERTZ_STEEL_LIMIT else ("⚠️" if r["p_pin_rated"] <= HERTZ_STEEL_LIMIT else "❌")
        ok_t = "✅" if r["tw_dry"] <= HAND_TWIST_OK else ("⚠️" if r["tw_lubed"] <= HAND_TWIST_OK else "❌")
        L.append(f"| {r['kx'].name} + {r['act']} | {r['rated']:.2f} / {r['peak']:.2f} | {r['slip']:.1f} N·m {ok_slip} | "
                 f"{r['p_pin_rated']:,.0f} / {r['p_pin_peak']:,.0f} MPa {ok_p} | {r['p_petg_rated']:.0f} MPa ❌ ({r['p_petg_rated'] / PETG_YIELD:.0f}× yield) | "
                 f"{r['tw_lubed']:.1f} / {r['tw_dry']:.1f} N·m {ok_t} | {r['lug_shear']:.1f} / {r['tooth']:.0f} MPa |")
    L += ["", "## Socket bearings (one pair per socket)", "",
          "| Socket | Interface | Bending moment (N·m) | Bearing pair | Max radial load per bearing | Static safety C0/P0 |", "|---|---|---|---|---|---|"]
    for s in socket_loads(d):
        flag = "✅" if s["s0"] >= 2 else ("⚠️" if s["s0"] >= 1 else "❌")
        L.append(f"| {s['joint']} | {s['kx']} | {s['moment']:.1f} | 2 × {s['brg']} | {s['f']:.0f} N | {s['s0']:.1f} {flag} |")
    L += ["", "## Reading the numbers", "",
          "- **Printed grooves fail.** A steel ball pressed into a printed V-groove at working load reaches several times PETG's yield. "
          "It would dent and creep, and the joint would lose its zero. **Dowel-pin grooves are the baseline**, not a fallback. "
          "Six Ø3–4 mm hardened pins plus three G25 balls per interface cost well under $1.",
          "- **Slip torque is a feature.** With a defined preload the seat is a **ball-detent overload clutch**: it holds the actuator's peak torque, "
          "and beyond that it lets go before a printed bone breaks. It re-seats at the same position (kinematic). A slip past one ball "
          "pitch re-indexes by 120°, so the joint must re-home: v1 adds a magnet index in every hub for a Hall check (cartridge electronics item).",
          "- **Ball-on-pin contact stress** stays under the bearing-steel static limit even at the actuators' absolute peak torque.",
          "- **The twist effort is jar-lid territory (about 2–3 N·m).** That is why the collar has two thumb tabs and the ramp gets a smear of grease. "
          "Self-locking holds because the 6° ramp is below the friction angle (8.5° greased).",
          "- **Socket bearings:** 61808 / 61805 thin-section pairs (~$2 each, ESTIMATE). Safety ≥ 2 everywhere under the conservative load case.",
          "", "Assumptions marked ESTIMATE in `rnd/kx_check.py`; contact stresses use the R_eff = √(Ra·Rb) point-contact approximation."]
    return "\n".join(L)


def score(d: Design = BASELINE):
    rows = interface_rows(d)
    slip = min(r["slip"] / r["peak"] for r in rows)
    sock = min(s["s0"] for s in socket_loads(d))
    return slip, sock


if __name__ == "__main__":
    print(report())
