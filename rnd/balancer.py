"""Shoulder-pitch gravity balancer: zero-free-length spring via cable (Anglepoise / Herder / French & Widden principle).

Geometry: pulley A sits on the torso a metres *above* the pitch axis O. Anchor B is a pin on the pitch joint's
balancer disc, b metres from O **along the arm** (below O when the arm hangs). A cable runs B → over A → down into the
torso to a caged spring (A9: through a 2:1 tackle into a die spring). The cage is designed so the spring is at its free length when the cable length
A–B would be zero, so the spring force is k·|AB| ("zero free length").
Then the torque about O is k·a·b·sin(pitch) exactly, for any pitch: the same law the workspace study assumed.
K is adjusted by sliding pulley A along a slot (a changes), never by pre-tension (which would break the law).

Loop 47 (Astra review #2b finding 8):
- **Pin-B sign.** In rnd/parts/shoulder.py pin B is cut at local y = −30 on the disc and laid along +Y with Rot(−90, 0, 0),
  which puts it at **+30 mm Z: above the axis, opposite the arm.** With B opposite the arm the torque is −k·a·b·sin(pitch):
  it adds to gravity instead of cancelling it, and the hanging span is a − b = 20 mm instead of a + b. The model (B along
  the arm) is the intended law; the CAD must flip (exact change in the loop-47 report; shoulder.py is not edited here).
- **Exact spring.** The "≈ 12 N/mm, L0 115" green die spring does not exist: the catalogue ISO 10243 green Ø25/Ø12.5 ×
  115 is 18.7 N/mm ± 10 % (Berger Tools V25-115). With it the old geometry (a 50, b 30) gives K = 7.0 N·m, not 4.5, and
  at the 65 mm slot end the deflection is (65 + 30)/2 = 47.5 mm = 41 % of L0, over the 40 % ISO maximum.
  New geometry for the real spring: b = 20 mm, a = 40–60 mm (nominal 48): K = 3.7–5.6 N·m around 4.5.
"""
from __future__ import annotations

import math

import numpy as np

from .params import BASELINE, Design

A_NOM, A_RANGE = 0.048, (0.040, 0.060)   # m, pulley A height above O (slot). |a-b| ≥ 15 mm so pin B clears pulley A at pitch 180°
B = 0.020                                # m, anchor pin radius (loop 47: 30 → 20 for the real 18.7 N/mm spring; loop 7: 40 → 30)
R_PULLEY = 0.0055                        # m, pulley A radius: U604ZZ grooved bearing pulley (A9: was an ungrooved Ø12 printed pulley)
CABLE_BREAK_N = 2000.0                   # 2 mm Dyneema/UHMWPE braid (ESTIMATE; typical 150–250 daN)
SPRING_SAFE_STROKE = 0.105               # m, cage stroke available (kept for the score signature)
R_SHAFT = 0.020                          # m, pitch shaft OD/2 (shoulder.py: Ø40 tube through the balancer plane)
# A9 (loop 40, external review finding 10): the loop-7 spring could not exist. Zero free length makes the spring deflect by
# the whole A–B span at k = 3 N/mm: a Ø25 compression spring that does that without overstress is ≈ 210 mm long. Fix: a 2:1
# tackle (the cable runs B → A → a moving pulley on the spring plate → anchored), so the spring sees half the travel at four
# times the rate, and a stock ISO 10243 die spring (rectangular wire, high energy density) fits the cage.
TACKLE = 2
CAGE_L = 0.135
CAGE_ID = 0.0248                         # shoulder.py cage: Cylinder(14) − Cylinder(14 − 1.6) → Ø24.8 bore
# Exact part (loop 47): ISO 10243 green (light load), hole Ø25, rod Ø12.5, L0 115 mm, R = 18.7 N/mm ± 10 %.
# Berger Tools part V25-115 (berger-tools.co.uk, "V - Green Die Spring ISO10243 Light Load"); the same ISO size is sold by
# Accu (TJL25-115), Dayton/Lamina and Raymond. ISO 10243 green: travel 25 % L0 long life, 30 % medium life, 40 % = Fmax.
DIE = dict(name="ISO 10243 green (light load) die spring Ø25 hole / Ø12.5 rod × L0 115 mm, R 18.7 N/mm ± 10 % (Berger Tools V25-115; Accu TJL25-115)",
           L0=0.115, rate=18700.0, rate_tol=0.10, max_frac=0.40, medium_frac=0.30, long_frac=0.25, hole=0.025, price=12.0)
OLD = dict(a=0.050, a_max=0.065, b=0.030)  # loop-40 geometry (for the before/after)


def spring_rate(K: float, a: float = A_NOM, b: float = B) -> float:
    """Cable-side rate needed for a design K (N/m)."""
    return K / (a * b)


def k_cable() -> float:
    """Cable-side rate that the real die spring gives behind the tackle (N/m)."""
    return DIE["rate"] / TACKLE ** 2


def torque(theta: float, k: float, a: float, b: float, rp: float = R_PULLEY) -> float:
    """Exact spring torque about O (N·m) with a finite pulley at A. theta = pitch from hanging (rad). B lies along the arm."""
    Bp = np.array([b * math.sin(theta), -b * math.cos(theta)])          # anchor (x forward, z up)
    Ap = np.array([0.0, a])
    d = Ap - Bp
    L = float(np.linalg.norm(d))
    l_t = math.sqrt(max(L * L - rp * rp, 1e-12))
    ang = math.atan2(d[1], d[0]) + math.asin(rp / L) * (1 if Bp[0] >= 0 else -1)
    u = np.array([math.cos(ang), math.sin(ang)])
    F = k * l_t                              # cage zeroed so F = k·(free span length)
    return float(Bp[0] * F * u[1] - Bp[1] * F * u[0]), F, l_t


def cad_pin_b():
    """Pin B in the shoulder-module frame as shoulder.py cuts it (disc local (0, −30, 0), laid along +Y by Rot(−90, 0, 0)).
    The arm hangs along −Z in that frame (robot.py: the arm sits at −YAW_DROP below the pitch axis at pitch 0)."""
    from build123d import Pos, Rot
    p = (Rot(-90, 0, 0) * Pos(0, -30, 0)).position
    return float(p.X) + 0.0, round(float(p.Y), 6) + 0.0, float(p.Z) + 0.0


def shaft_clear(a: float, b: float, r: float = R_SHAFT + 0.001):
    """The A–B cable runs in the balancer plane, which the Ø40 pitch shaft passes through. Pitch range (deg from hanging)
    where the straight cable would cut the shaft: distance from O to line AB = a·b·sin θ / |AB| < r."""
    bad = [t for t in range(0, 181) if a * b * math.sin(math.radians(t)) / math.hypot(b * math.sin(math.radians(t)), a + b * math.cos(math.radians(t))) < r]
    return (min(bad), max(bad)) if bad else None


def study(d: Design = BASELINE, a: float = A_NOM, b: float = B):
    K = d.arm.spring_k_nm
    k = k_cable()
    th = np.radians(np.linspace(-200, 60, 261))
    K_here = k * a * b
    ideal = K_here * np.sin(th)
    vals = [torque(t, k, a, b) for t in th]
    real = np.array([v[0] for v in vals])
    F = np.array([v[1] for v in vals])
    span = np.array([v[2] for v in vals])
    err = np.max(np.abs(real - ideal))
    adj = (k * A_RANGE[0] * b, k * A_RANGE[1] * b)
    defl = float(span.max()) / TACKLE
    defl_max = (A_RANGE[1] + b) / TACKLE                         # at the top of the slot, arm hanging
    fits = defl_max <= DIE["max_frac"] * DIE["L0"] and DIE["L0"] + 0.012 <= CAGE_L
    a_for_K = K / (k * b)
    return dict(K=K, K_here=K_here, k=k, a_for_K=a_for_K, err=err, Fmax=float(F.max()), stroke=float(span.max() - span.min()),
                span_max=float(span.max()), adj=adj,
                cable_sf=CABLE_BREAK_N / F.max(), ks=DIE["rate"], defl=defl, defl_max=defl_max, Fs=TACKLE * float(F.max()),
                Fs_max=DIE["rate"] * defl_max, F_iso=DIE["rate"] * DIE["max_frac"] * DIE["L0"],
                energy=0.5 * DIE["rate"] * defl ** 2, fits=fits, clear=shaft_clear(a, b), cage_ok=CAGE_ID >= DIE["hole"] + 0.0005,
                clear_ab=abs(a - b) >= 0.015 and abs(A_RANGE[0] - b) >= 0.015)


def old_check():
    """The loop-40 numbers Astra checked, and what the real spring does to them."""
    o = OLD
    frac = (o["a_max"] + o["b"]) / TACKLE / DIE["L0"]
    return dict(frac=frac, K_real=k_cable() * o["a"] * o["b"], frac_nom=(o["a"] + o["b"]) / TACKLE / DIE["L0"])


def report(d: Design = BASELINE) -> str:
    s = study(d)
    o = old_check()
    pb = cad_pin_b()
    sign_ok = pb[2] < 0
    f = lambda x: f"{x:.0%}"
    rng = s["clear"]
    return "\n".join([
        "# Shoulder balancer (auto-generated by `python -m rnd.balancer`)", "",
        f"Zero-free-length cable-spring balancer. Pulley A sits {A_RANGE[0] * 1000:.0f}–{A_RANGE[1] * 1000:.0f} mm above the pitch axis in a slot; pin B sits on the "
        f"balancer disc {B * 1000:.0f} mm from the axis, **along the arm**. A cable runs B → A → (2:1 tackle) → a caged die spring in the torso, giving "
        "τ = k·a·b·sin(pitch) at every angle. Precedent: the Anglepoise lamp, Herder's spring balancers, French & Widden cable balancers.", "",
        "## Loop 47 (Astra #2b finding 8)", "",
        f"- **Pin-B sign: {'OK' if sign_ok else 'WRONG in the CAD'}.** shoulder.py puts pin B at ({pb[0]:.0f}, {pb[1]:.0f}, **{pb[2]:+.0f}**) mm in the module frame; "
        "the arm hangs along −Z. B must lie along the arm (−Z when hanging). Opposite the arm the torque is −k·a·b·sin(pitch): it doubles the gravity load "
        f"instead of cancelling it, and the hanging span is a − b = {(OLD['a'] - OLD['b']) * 1000:.0f} mm, not a + b = {(OLD['a'] + OLD['b']) * 1000:.0f} mm (Astra's 20 vs 80 mm).",
        f"- **The 41 %:** at the old 65 mm slot end the spring deflection is ({OLD['a_max'] * 1000:.0f} + {OLD['b'] * 1000:.0f})/2 = {(OLD['a_max'] + OLD['b']) / 2 * 1000:.1f} mm = "
        f"**{o['frac']:.1%} of L0** (ISO green maximum 40 %; {o['frac_nom']:.0%} at the nominal 50 mm, already over the 30 % medium-life line). Worse, the 12 N/mm "
        f"spring the model assumed is not a catalogue part: the real green Ø25 × 115 is 18.7 N/mm, which makes the old geometry K = {o['K_real']:.1f} N·m instead of {d.arm.spring_k_nm} N·m.",
        f"- **Fix (this file):** keep the spring and the 2:1 tackle, change the geometry for the real rate: b = {B * 1000:.0f} mm, slot a = {A_RANGE[0] * 1000:.0f}–{A_RANGE[1] * 1000:.0f} mm. "
        f"K = {d.arm.spring_k_nm} N·m needs a = {s['a_for_K'] * 1000:.1f} mm.",
        f"- **Cable vs shaft:** the straight A–B cable passes within the Ø{2 * R_SHAFT * 1000:.0f} shaft for pitch "
        + (f"0–{rng[1]}° from hanging (both directions) whenever the balancer plane holds the shaft. **The pin must sit beyond the shaft end** (a crank or end disc "
           "past the tube, the harness leaving the bore through a side window first) or the cable has to wrap the shaft, which breaks the law." if rng else "never ✅."), "",
        "| Quantity | Value |", "|---|---|",
        f"| Spring (exact part) | {DIE['name']} |",
        f"| Cable-side rate behind the 2:1 tackle, k = R/4 | **{s['k'] / 1000:.2f} N/mm** |",
        f"| K at the nominal a = {A_NOM * 1000:.0f} mm | {s['K_here']:.2f} N·m (design {s['K']:.1f}) |",
        f"| K adjustment by sliding A ({A_RANGE[0] * 1000:.0f}–{A_RANGE[1] * 1000:.0f} mm) | {s['adj'][0]:.1f}–{s['adj'][1]:.1f} N·m; rate ± 10 % still reaches {s['K']:.1f} N·m: "
        f"a = {s['K'] / (s['k'] * 1.1 * B) * 1000:.0f}–{s['K'] / (s['k'] * 0.9 * B) * 1000:.0f} mm |",
        f"| Spring deflection, nominal (arm hanging) | {s['defl'] * 1000:.1f} mm = {f(s['defl'] / DIE['L0'])} of L0 (medium life ≤ {f(DIE['medium_frac'])}) {'✅' if s['defl'] <= DIE['medium_frac'] * DIE['L0'] + 1e-4 else '⚠️'} |",
        f"| Spring deflection at the slot end a = {A_RANGE[1] * 1000:.0f} mm | {s['defl_max'] * 1000:.1f} mm = **{f(s['defl_max'] / DIE['L0'])}** of L0 (ISO max {f(DIE['max_frac'])}) {'✅' if s['fits'] else '❌'} |",
        f"| Spring force, nominal / slot end / ISO Fmax | {s['Fs']:.0f} / {s['Fs_max']:.0f} / {s['F_iso']:.0f} N |",
        f"| Stored energy (nominal, hanging) | {s['energy']:.1f} J |",
        f"| Cage: L0 + 12 ≤ {CAGE_L * 1000:.0f} mm; bore Ø{CAGE_ID * 1000:.1f} vs Ø25 spring hole | {'✅' if DIE['L0'] + 0.012 <= CAGE_L else '❌'}; {'✅' if s['cage_ok'] else '❌ bore too small: open the cage to Ø25.5–26'} |",
        f"| B clears A at pitch 180° (|a − b| ≥ 15 mm over the slot) | {'✅' if s['clear_ab'] else '❌'} ({(A_RANGE[0] - B) * 1000:.0f} mm at the bottom of the slot) |",
        f"| Law error from the Ø{2 * R_PULLEY * 1000:.0f} mm pulley (max over −200…60°) | {s['err']:.2f} N·m ({s['err'] / s['K_here']:.0%} of K) |",
        f"| Cable safety (2 mm UHMWPE, ESTIMATE {CABLE_BREAK_N:.0f} N) | {s['cable_sf']:.0f}× |", "",
        "Life: ISO 10243 green allows 25 % of L0 for long life, 30 % for medium life and 40 % at Fmax. The spring works between |a − b|/2 (arm up) and (a + b)/2 "
        "(arm hanging); most cycles are pitch 0 ↔ 90°, i.e. (a + b)/2 ↔ √(a² + b²)/2. Coupon C-bal: 10⁵ cycles at the slot end before the bulk buy.",
        "Cable: B → U604 pulley A → U604 moving pulley on the spring plate → anchor screw on the cage.",
        "**Detach hazard (A9):** with the arm off, the spring torque K·sin(pitch) is unopposed. Detach and attach only with the arm hanging (pitch 0, "
        "where the torque is zero), with the **park pin** in: a Ø6 pin through the bracket wall into a hole in the balancer disc at pitch 0. The pitch motor holds the "
        "shaft as well while powered.",
        "**Failure mode:** with the cable cut, the arm is simply unbalanced; the pitch motor still holds it (see the workspace margins without the spring in spring.md)."])


def score(d: Design = BASELINE):
    s = study(d)
    return s["err"] / s["K_here"], s["fits"]


if __name__ == "__main__":
    from pathlib import Path
    md = report()
    (Path(__file__).resolve().parent.parent / "docs" / "rnd" / "balancer.md").write_text(md + "\n\n![shoulder pitch module](img/shoulder_pitch_v0.png)\n")
    print(md)
