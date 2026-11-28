"""Actuator system identification (real2sim): bring-up step 7 "sysid", after "gravity" (sw/bringup.py).

The Asimov sim2real lesson: model the actuators carefully and randomise only what is actually random, instead of
randomising around an unmeasured drive. For each online joint, at low torque, with the safety supervisor in the loop:

1. **Friction: constant-velocity sweeps.** ± sweeps (± the jog amplitude) at 4 speeds (10–60 % of the jog speed),
   centred where gravity loads the joint most so the gear stays on one flank (no backlash crossing mid-sweep).
   Over the same positions the gravity torque (and any error in the twin's gravity) is identical in both directions, so
   (τ̄₊ − τ̄₋)/2 = (F_c + B·v)/K. A line through the four speeds gives the Coulomb F_c and viscous B (drive units ÷ K).
2. **Effective torque constant K** (true joint torque / drive-reported torque): the 20 %-speed sweep repeated with a
   known test mass (the Step-1 lever weight, `sysid.test_mass_kg`) at the grasp site. Sliding at the same speed, the
   friction is the same with and without the mass, and the mass's torque is known from kinematics alone (the arm's own
   mass model cancels in the difference): per pass Δτ̄ = Δḡ / K, K = Σ Δg·Δτ / Σ Δτ². (Static holds were tried first:
   the joint stops anywhere inside its stiction band, which biased K by 8–14 %.)
   Joints where the mass makes < 5 % of rated torque keep K = 1 and say "not observable here".
   **The Step-1 rig (`kt_method: absolute`, sw/config/rig_step1.yaml):** its guard is interlocked, so no weight can be
   hung mid-session; but there the whole load is known (bar, carrier and discs weighed to ±1 g, radii from the CAD), so
   K comes from the mean of the ± sweeps of all four speeds against the twin's gravity directly: per position bin
   (τ̄₊ + τ̄₋)/2 = g / K (friction and viscous terms cancel), K = Σ g·τ̄ / Σ τ̄².
3. **Backlash: direction reversals.** A slow triangle (±2.5°, 0.1 rad/s) at the joint's zero-gravity angle (from the
   twin). The drive's encoder is on the motor side; inside the gap the motor turns with ≈ 0 torque, so the motor travel
   while |τ| < ½ F_c at each reversal is the backlash.
4. **Command → response latency:** 30 torque steps (± 8 % rated feed-forward, random phase): host time from sending the
   step to the first feedback frame that shows it (round trip: TX + drive + RX + one control cycle). Cross-check: a
   1–20 Hz torque chirp, lag of the cross-correlation peak.

The fit goes to `actuators_<serial>.yaml` (sw/config on the bench; the report directory for a virtual run), which
rnd/twin.py `actuator_model()`, sw/bench_env.py and sw/virtual_arm.py load when present (datasheet model otherwise). A
rig's fit (a bare motor output, not the arm joint) goes to sw/config/rig_<name>/ and is tagged with the rig, so the arm
twin never loads it; carrying it to the arm's yaw (2:1 stage) is a conversion (friction ×2 plus the stage's own, backlash
÷2 plus the belt's), not a copy.
"""
from __future__ import annotations

import datetime as _dt
import math
from pathlib import Path

import numpy as np
import yaml

D2R = math.pi / 180
DEFAULTS = dict(joints=["yaw", "wpitch"], speeds_frac=[0.1, 0.2, 0.35, 0.6], test_mass_kg=0.5, backlash_amp_deg=2.5,
                backlash_speed=0.1, latency_steps=30, latency_frac=0.08, chirp_s=3.0)
NOT_RANDOMIZED = ("Once measured these are fixed facts of each drive (per serial), not domain-randomised: Coulomb + viscous "
                  "friction, effective torque constant, backlash, the command → response latency distribution (use the "
                  "measured histogram, not 0–9 ms uniform), encoder zero (calibrated), drive torque clamp, link masses "
                  "(CAD + the gravity step's residual). Re-measured at the start and end of the soak test: if they drift, "
                  "model the drift (e.g. temperature) instead of widening a randomisation range.")
STILL_RANDOMIZED = ("Still randomised (actually random from task to task): payload mass / COM, contact friction (object, "
                    "table, finger pads), object pose and size, lighting and camera pose; sensor noise at the measured σ.")


def _cfg(bu):
    return {**DEFAULTS, **(bu.cfg.get("sysid") or {})}


def _sup(bu):
    if bu.io.estop_active():
        bu.arm()
    sup = bu.supervisor()
    bu.start(sup)
    bu.enable_all(sup)
    return sup


def _hold(bu, sup, tgt, approach=3 * D2R, settle=0.4, avg=0.3):
    """Mean measured τ, q for every online joint at `tgt` (dict), approached from both sides (friction cancels)."""
    taus, qs = [], []
    for sgn in (+1, -1):
        bu._move(sup, {j: tgt[j] + sgn * approach for j in tgt}, settle=0.05)
        bu._move(sup, tgt, settle=settle)
        out = bu.run(sup, int(avg / bu.dt), lambda k, fb: bu._cmd_hold(fb))
        taus.append(np.mean([f.t for f in out], 0))
        qs.append(np.mean([f.q for f in out], 0))
    return (taus[0] + taus[1]) / 2, (qs[0] + qs[1]) / 2


NBINS = 10


def _sweep(bu, sup, j, v, lo, hi, keep=False):
    """+v from lo to hi then −v back. Samples are kept where the joint is in the middle 60 % of the range (by MEASURED
    position, so both passes cover the same positions despite the tracking lag) and at steady speed (within 15 % of the
    command: start-up transients excluded). → (τ̄₊ − τ̄₋)/2, (v̄₊ − v̄₋)/2; keep: → per pass, NBINS position bins of
    (q bin centre, mean τ, mean full q)."""
    k = bu.names.index(j)
    bu._move(sup, {j: lo}, settle=0.25)
    passes = []
    for s, e in ((lo, hi), (hi, lo)):
        T = abs(e - s) / v
        n = int(math.ceil(T / bu.dt))
        sgn = 1.0 if e > s else -1.0
        rec = []
        for i in range(n):
            t = (i + 1) * bu.dt
            bu.hold_q[j] = s + sgn * v * min(t, T)
            bu.hold_v[j], bu.hold_a[j] = (sgn * v if t < T else 0.0), 0.0
            vv, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
            if vv.stop:
                raise RuntimeError(f"supervisor stop: {vv.reasons}")
            rec.append((fb.q[k], fb.t[k], fb.v[k], fb.q.copy(), sgn * v))
        bu.hold_v[j] = bu.hold_a[j] = 0.0
        bu._move(sup, {j: e}, settle=0.2)
        passes.append(rec)
    a, b = lo + 0.2 * (hi - lo), hi - 0.2 * (hi - lo)
    edges = np.linspace(a, b, NBINS + 1)
    out = []
    for rec in passes:
        sel = [r for r in rec if a <= r[0] <= b and abs(r[2] - r[4]) < 0.15 * abs(r[4])]
        if len(sel) < 10:
            sel = [r for r in rec if a <= r[0] <= b]
        q = np.array([r[0] for r in sel])
        tau = np.array([r[1] for r in sel])
        vel = np.array([r[2] for r in sel])
        bins = []
        for i in range(NBINS):
            m = (q >= edges[i]) & (q < edges[i + 1])
            if m.sum():
                bins.append((0.5 * (edges[i] + edges[i + 1]), float(tau[m].mean()), np.mean([sel[x][3] for x in np.flatnonzero(m)], 0)))
        # equal weight per position bin: the passes are compared over the same positions
        out.append((float(np.mean([bb[1] for bb in bins])), float(vel.mean()), bins))
    if keep:
        return out
    return (out[0][0] - out[1][0]) / 2, (out[0][1] - out[1][1]) / 2


def _y_v(r):
    """Friction point from a kept sweep: only the bins both passes have."""
    c0 = {round(bb[0], 6): bb[1] for bb in r[0][2]}
    c1 = {round(bb[0], 6): bb[1] for bb in r[1][2]}
    common = sorted(set(c0) & set(c1))
    y = float(np.mean([c0[c] - c1[c] for c in common])) / 2
    return y, (r[0][1] - r[1][1]) / 2


def _loaded_centre(bu, sup, j, A):
    """Centre of the friction / K sweeps: where gravity loads the joint most (nearest the park angle among the angles
    within 90 % of the largest |g|), so the gear stays on one flank over the whole sweep. Sweeping through the
    zero-gravity angle flips the load across the backlash mid-sweep and the torque glitches (first rehearsal: yaw
    viscous −19 %, K −7 %)."""
    from sw.bringup import ARM
    k = bu.names.index(j)
    lo, hi = sup.lo[k] + A + 0.05, sup.hi[k] - A - 0.05
    park = bu.jc(j)["park"] * D2R
    if hi <= lo:
        return park
    q6 = bu.full_q(np.array([bu.hold_q.get(x, 0.0) for x in bu.names]))
    xs = np.linspace(lo, hi, 121)
    g = []
    for x in xs:
        q6[ARM.index(j)] = x
        g.append(abs(bu.twin.gravity(q6)[ARM.index(j)]))
    g = np.array(g)
    if g.max() < 0.02 * bu.J[j]["rated_nm"]:
        return float(np.clip(park, lo, hi))
    ok = xs[g >= 0.9 * g.max()]
    return float(ok[np.argmin(np.abs(ok - park))])


def _zero_gravity_angle(bu, j, lo, hi):
    k6 = bu.names.index(j)
    from sw.bringup import ARM
    q6 = bu.full_q(np.array([bu.hold_q.get(x, 0.0) for x in bu.names]))
    xs = np.linspace(lo, hi, 181)
    g = []
    for x in xs:
        q6[ARM.index(j)] = x
        g.append(bu.twin.gravity(q6)[ARM.index(j)])
    g = np.array(g)
    park = bu.jc(j)["park"] * D2R
    cand = [xs[i] - g[i] * (xs[i + 1] - xs[i]) / (g[i + 1] - g[i]) for i in range(len(xs) - 1) if g[i] * g[i + 1] <= 0 and g[i] != g[i + 1]]
    if not cand:
        return float(xs[int(np.argmin(np.abs(g)))]), float(np.min(np.abs(g)))
    c = min(cand, key=lambda x: abs(x - park))
    return float(c), 0.0
    _ = k6


def _backlash(bu, sup, j, q0, amp, speed, fc_rep):
    k = bu.names.index(j)
    KP, KD = bu.gains()
    bu._move(sup, {j: q0 - amp}, speed=bu.cfg["cal_speed"], settle=0.3)
    half = 2 * amp / speed
    n = int(4 * 2 * half / bu.dt)
    log = []
    for i in range(n):
        t = (i + 1) * bu.dt
        ph = (t / half) % 2.0
        x = q0 - amp + (2 * amp) * (ph if ph < 1 else 2 - ph)
        q, kp, kd, tff = bu._cmd_hold(bu.feedback(), active=j)
        q6 = bu.full_q(np.array([x if y == j else bu.hold_q.get(y, 0.0) for y in bu.names]))
        from sw.bringup import ARM
        q[k], kp[k], kd[k], tff[k] = x, KP[k], KD[k], bu.twin.gravity(q6)[ARM.index(j)]
        vv, fb = bu.cycle(sup, q, kp, kd, tff)
        if vv.stop:
            raise RuntimeError(f"supervisor stop: {vv.reasons}")
        log.append((t, x, fb.q[k], fb.t[k], ph < 1))
    bu.hold_q[j] = log[-1][1]
    L = np.array(log, float)
    tf = np.convolve(L[:, 3], np.ones(5) / 5, mode="same")
    th = 0.5 * fc_rep
    up = L[:, 4] > 0.5
    edges = np.flatnonzero(np.diff(up.astype(int)) != 0) + 1
    segs = np.split(np.arange(len(L)), edges)[1:]            # skip the first (approach) segment
    out = []
    for sg in segs:
        sg = sg[: max(5, len(sg) // 2)]                       # the gap is crossed in the first half after a reversal
        m = sg[np.abs(tf[sg]) < th]
        if len(m) >= 3:
            out.append(float(L[m, 2].max() - L[m, 2].min()))
    return (float(np.median(out)) if out else None), out, float(th)


def _latency(bu, sup, j, nsteps, frac, rng):
    k = bu.names.index(j)
    step = frac * bu.J[j]["rated_nm"]
    samples = []
    extra = 0.0
    rt0 = len(bu.plant.rt_log) if bu.plant is not None else 0
    for i in range(nsteps):
        pre = int(rng.integers(15, 25))
        base = []
        for _ in range(pre):
            q, kp, kd, tff = bu._cmd_hold(bu.feedback())
            tff[k] += extra
            vv, fb = bu.cycle(sup, q, kp, kd, tff)
            if fb.age is not None:
                base.append(fb.t[k])
        b0 = float(np.mean(base[-5:]))
        new = step if extra <= 0 else -step
        t_send = bu.now()                                      # the next cycle's command carries the step
        seen = None
        for c in range(40):
            q, kp, kd, tff = bu._cmd_hold(bu.feedback())
            tff[k] += new
            if c == 0:
                t_send = bu.now()
            vv, fb = bu.cycle(sup, q, kp, kd, tff)
            if vv.stop:
                raise RuntimeError(f"supervisor stop: {vv.reasons}")
            fresh = bu.drivers[j].state.get("fresh")
            if seen is None and fresh and abs(fb.t[k] - b0 - (new - extra)) < 0.5 * abs(new - extra):
                seen = bu.now() - t_send
        extra = new
        if seen is not None:
            samples.append(seen * 1000)
    # chirp cross-check
    n = int(3.0 / bu.dt)
    xs, ys = [], []
    for i in range(n):
        t = i * bu.dt
        f = 1.0 + (20.0 - 1.0) * t / 3.0
        u = 0.5 * step * math.sin(2 * math.pi * (1.0 * t + 0.5 * (19.0 / 3.0) * t * t))
        q, kp, kd, tff = bu._cmd_hold(bu.feedback())
        tff[k] += u
        vv, fb = bu.cycle(sup, q, kp, kd, tff)
        xs.append(u)
        ys.append(fb.t[k])
        _ = f
    x, y = np.array(xs), np.array(ys)
    y = y - np.convolve(y, np.ones(51) / 51, mode="same")
    lags = np.arange(0, 16)
    cc = np.array([np.dot(x[: len(x) - L], y[L:]) for L in lags])
    i = int(np.argmax(cc))
    if 0 < i < len(cc) - 1:
        d = 0.5 * (cc[i - 1] - cc[i + 1]) / (cc[i - 1] - 2 * cc[i] + cc[i + 1])
    else:
        d = 0.0
    chirp_ms = (i + d) * bu.dt * 1000 + bu.dt * 1000        # +1 cycle: sent at the end of cycle k, seen at the end of k+1+lag
    truth = None
    if bu.plant is not None:
        rt = np.array(bu.plant.rt_log[rt0:]) * 1000
        if len(rt):
            truth = dict(mean=round(float(rt.mean()), 3), p95=round(float(np.percentile(rt, 95)), 3), max=round(float(rt.max()), 3))
    S = np.array(samples) if samples else np.array([np.nan])
    return dict(n=len(samples), mean=round(float(np.nanmean(S)), 3), p50=round(float(np.nanmedian(S)), 3),
                p95=round(float(np.nanpercentile(S, 95)), 3), max=round(float(np.nanmax(S)), 3), chirp=round(float(chirp_ms), 3),
                histogram_ms={str(int(b)): int(np.sum(np.round(S) == b)) for b in np.unique(np.round(S[np.isfinite(S)]))}), truth


def _operator_mass(bu, sup, kg, timeout=120.0):
    """The operator hangs (kg > 0) or removes the test weight while the loop keeps holding the joints (heartbeat on)."""
    bu.op.hang_test_mass(kg)
    t_end = bu.now() + timeout
    while not bu.op.test_mass_ready():
        vv, fb = bu.cycle(sup, *bu._cmd_hold(bu.feedback()))
        if vv.stop:
            raise RuntimeError(f"supervisor stop while the test weight was handled: {vv.reasons}")
        if bu.now() > t_end:
            raise RuntimeError("operator did not confirm the test weight")
    for _ in range(int(0.5 / bu.dt)):                       # let it settle
        bu.cycle(sup, *bu._cmd_hold(bu.feedback()))


def _mass_torque(bu, j, qs, m_kg):
    """Mean torque of the test mass (m_kg at the grasp site, or {joint: kg} on the rig's levers) about joint j over the
    joint states qs (twin kinematics only: the arm's own mass model cancels in the with/without difference)."""
    from sw.bringup import ARM
    from sw.rig import capture, set_mass
    tw = bu.twin
    base = capture(tw.m)
    k6 = ARM.index(j)
    g0, g1 = [], []
    for q in qs:
        g0.append(tw.gravity(bu.full_q(q))[k6])
    set_mass(tw.m, m_kg, base)
    for q in qs:
        g1.append(tw.gravity(bu.full_q(q))[k6])
    set_mass(tw.m, 0, base)
    return float(np.mean(g1) - np.mean(g0))


def _kt_absolute(bu, sweeps):
    """K per joint from the ± sweeps alone (the Step-1 rig: the lever's whole gravity is known from weighed parts): per
    position bin both passes have, (τ̄₊ + τ̄₋)/2 = g/K, g = the twin's gravity at the bin's mean pose."""
    from sw.bringup import ARM
    out = {}
    for j, runs in sweeps.items():
        k6 = ARM.index(j)
        g, t = [], []
        for r in runs:
            b0 = {round(bb[0], 6): bb for bb in r[0][2]}
            for bb in r[1][2]:
                c = round(bb[0], 6)
                if c in b0:
                    q = 0.5 * (np.asarray(b0[c][2]) + np.asarray(bb[2]))
                    g.append(bu.twin.gravity(bu.full_q(q))[k6])
                    t.append(0.5 * (b0[c][1] + bb[1]))
        g, t = np.array(g), np.array(t)
        rated = bu.J[j]["rated_nm"]
        if not len(g) or np.max(np.abs(g)) < 0.05 * rated:
            out[j] = dict(kt_scale=None, why=f"lever gravity ≤ {np.max(np.abs(g), initial=0):.2f} N·m over the sweeps (< 5 % of rated): not observable",
                          dg=g.round(4).tolist(), dtau=t.round(4).tolist(), method="absolute")
            continue
        K = float(np.dot(g, t) / np.dot(t, t))
        out[j] = dict(kt_scale=K, dg=g.round(4).tolist(), dtau=t.round(4).tolist(), resid_nm=float(np.abs(g - K * t).max()), method="absolute")
    return out


def _kt(bu, sup, cfgs, base_sweeps):
    """Effective torque constant per joint from sliding sweeps with and without the test mass. While sliding at the same
    speed the friction is the same in both runs (a static hold stops anywhere inside the stiction band, which biased K by
    8–14 % in the first rehearsal), so per pass Δτ̄_reported = Δḡ_mass / K."""
    m_kg = cfgs["test_mass_kg"]
    _operator_mass(bu, sup, m_kg)
    loaded = {}
    for j, (v, lo, hi) in base_sweeps["args"].items():
        loaded[j] = _sweep(bu, sup, j, v, lo, hi, keep=True)
        bu._move(sup, {j: bu.jc(j)["park"] * D2R}, settle=0.2)
    _operator_mass(bu, sup, 0.0)
    out = {}
    for j in loaded:
        dg, dt = [], []
        for p0, p1 in zip(base_sweeps["res"][j], loaded[j]):     # same direction, same position bin: same friction
            b0 = {round(bb[0], 6): bb for bb in p0[2]}
            for bb in p1[2]:
                c = round(bb[0], 6)
                if c in b0:
                    dg.append(_mass_torque(bu, j, [bb[2]], m_kg))
                    dt.append(bb[1] - b0[c][1])
        dg, dt = np.array(dg), np.array(dt)
        rated = bu.J[j]["rated_nm"]
        if np.max(np.abs(dg)) < 0.05 * rated:
            out[j] = dict(kt_scale=None, why=f"test mass makes ≤ {np.max(np.abs(dg)):.2f} N·m here (< 5 % of rated): not observable",
                          dg=dg.round(4).tolist(), dtau=dt.round(4).tolist())
            continue
        K = float(np.dot(dg, dt) / np.dot(dt, dt))
        out[j] = dict(kt_scale=K, dg=dg.round(4).tolist(), dtau=dt.round(4).tolist(), resid_nm=float(np.abs(dg - K * dt).max()))
    return out


def run(bu, step):
    """The sysid step (called by BringUp.step_sysid). Fills step.checks / data, writes actuators_<serial>.yaml."""
    cfgs = _cfg(bu)
    rng = np.random.default_rng(1)
    sup = _sup(bu)
    for j in bu.names:
        bu.hold_q.setdefault(j, bu.feedback().q[bu.names.index(j)])
    bu._move(sup, {j: bu.jc(j)["park"] * D2R for j in bu.names}, settle=0.3)
    rows = {}
    kt_base = dict(res={}, args={})
    todo = [j for j in bu.names if j in (cfgs.get("joints") or bu.names)]
    for j in bu.names:
        if j not in todo:
            step.notes.append(f"ℹ️ {j}: not identified in this run (bringup.yaml sysid.joints = {cfgs.get('joints')}: the Step-1 joints); datasheet model kept")
    if not todo:
        step.check("at least one joint to identify", False, f"online {bu.names}, sysid.joints {cfgs.get('joints')}")
        bu.disable_all()
        return
    for j in todo:
        k = bu.names.index(j)
        c = bu.jc(j)
        park, A = c["park"] * D2R, c["jog"] * D2R
        centre = _loaded_centre(bu, sup, j, A)
        lo, hi = float(np.clip(centre - A, sup.lo[k], sup.hi[k])), float(np.clip(centre + A, sup.lo[k], sup.hi[k]))
        bu._move(sup, {j: centre}, speed=bu.cfg["cal_speed"], settle=0.2)
        vj = bu.cfg["jog_speed_frac"] * bu.J[j]["max_speed_rad_s"]
        pts = []
        kt_base.setdefault("all", {})[j] = []
        for i, f in enumerate(cfgs["speeds_frac"]):
            r = _sweep(bu, sup, j, f * vj, lo, hi, keep=True)
            pts.append(_y_v(r))
            kt_base["all"][j].append(r)
            if i == 1:
                kt_base["res"][j], kt_base["args"][j] = r, (f * vj, lo, hi)
        P = np.array([(v, y) for y, v in pts])
        X = np.c_[np.ones(len(P)), P[:, 0]]
        (a, b), *_ = np.linalg.lstsq(X, P[:, 1], rcond=None)
        resid = P[:, 1] - X @ np.array([a, b])
        bu._move(sup, {j: park}, settle=0.2)
        # backlash at the zero-gravity angle
        q0, g_res = _zero_gravity_angle(bu, j, sup.lo[k] + 0.1, sup.hi[k] - 0.1)
        amp = cfgs["backlash_amp_deg"] * D2R
        bl, bl_all, th = _backlash(bu, sup, j, q0, amp, cfgs["backlash_speed"], max(a, 1e-3))
        bu._move(sup, {j: park}, speed=bu.cfg["cal_speed"], settle=0.3)
        lat, lat_truth = _latency(bu, sup, j, cfgs["latency_steps"], cfgs["latency_frac"], rng)
        rows[j] = dict(points=P.round(5).tolist(), fc_rep=float(a), b_rep=float(b), fit_resid=float(np.abs(resid).max()),
                       backlash_rad=bl, backlash_samples=[round(x, 5) for x in bl_all], backlash_pose_deg=round(math.degrees(q0), 1),
                       backlash_threshold_nm=round(th, 4), latency_ms=lat, latency_truth_ms=lat_truth)
    absolute = cfgs.get("kt_method") == "absolute"
    kt = _kt_absolute(bu, kt_base["all"]) if absolute else _kt(bu, sup, cfgs, kt_base)
    bu._move(sup, {j: bu.jc(j)["park"] * D2R for j in bu.names}, settle=0.2)
    bu.disable_all()
    # assemble, check, write
    date = _dt.date.today().isoformat()
    serials = {r["joint"]: r.get("serial") for r in bu.steps["enumerate"].data.get("devices", [])}
    kinds = {r["joint"]: r.get("kind") for r in bu.steps["enumerate"].data.get("devices", [])}
    from rnd.twin import ACTUATOR_DATASHEET
    rig = bu.cfg.get("rig")
    if rig:
        from sw.rig import load as _rig_load
        datasheet = _rig_load(rig)["actuators"]
    else:
        datasheet = ACTUATOR_DATASHEET
    out_dir = Path(bu.actuator_dir)
    out_dir.mkdir(parents=True, exist_ok=True)
    fits = {}
    for j, r in rows.items():
        K = kt[j]["kt_scale"]
        Ku = K if K is not None else 1.0
        ds = datasheet[j]
        fit = dict(joint=j, kind=kinds.get(j), serial=serials.get(j), date=date,
                   source="virtual bench (rehearsal)" if bu.plant is not None else "bench (sw/bringup.py sysid)",
                   rig=rig, stage=(f"bare motor output on the {rig} rig lever (ratio 1.0): NOT the arm joint; convert before using it "
                                   "for the arm (friction × stage ratio + the stage's own, backlash ÷ ratio + the belt's)") if rig else
                   "arm joint (motor + stage)",
                   load=bu.cfg.get("load"),
                   coulomb_nm=round(Ku * r["fc_rep"], 5), viscous_nms=round(Ku * r["b_rep"], 5),
                   kt_scale=None if K is None else round(K, 5), backlash_rad=None if r["backlash_rad"] is None else round(r["backlash_rad"], 6),
                   latency_ms=r["latency_ms"],
                   datasheet=dict(coulomb_nm=ds["coulomb_nm"], viscous_nms=ds["viscous_nms"], backlash_rad=ds["backlash_rad"]),
                   fit=dict(friction_points_v_tau=r["points"], friction_resid_nm=round(r["fit_resid"], 5),
                            kt=dict({x: y for x, y in kt[j].items() if x != "kt_scale"}),
                            backlash_samples_rad=r["backlash_samples"], backlash_pose_deg=r["backlash_pose_deg"]),
                   not_randomized=NOT_RANDOMIZED, still_randomized=STILL_RANDOMIZED)
        fits[j] = fit
        path = out_dir / f"actuators_{fit['serial']}.yaml"
        path.write_text("# Fitted actuator model (sw/sysid.py, bring-up step sysid). Joint-side units: N·m, N·m·s/rad, rad, ms.\n"
                        "# kt_scale = true joint torque / drive-reported torque. Loaded by rnd/twin.py actuator_model(), sw/bench_env.py,\n"
                        "# sw/virtual_arm.py. Replace by re-running the step; do not hand-edit.\n" + yaml.safe_dump(fit, sort_keys=False, allow_unicode=True))
        fit["file"] = str(path)
        rated = bu.J[j]["rated_nm"]
        step.check(f"{j}: friction F_c {fit['coulomb_nm']:.3f} N·m, B {fit['viscous_nms']:.3f} N·m·s/rad (4 speeds, line fit)",
                   fit["coulomb_nm"] >= -0.01 * rated and fit["viscous_nms"] >= -0.05 and r["fit_resid"] < 0.05 * rated,
                   f"max residual {r['fit_resid']:.4f} N·m; datasheet model F_c {ds['coulomb_nm']}, B {ds['viscous_nms']}")
        if K is None:
            step.notes.append(f"ℹ️ {j}: torque constant {kt[j]['why']}; K = 1 kept (datasheet)")
        else:
            how = ("the ± sweeps against the weighed lever's gravity (kt_method absolute)" if absolute else
                   f"sweeps with the {cfgs['test_mass_kg']} kg test mass")
            step.check(f"{j}: effective torque constant K = {K:.3f} (true / reported) from {how}",
                       0.8 < K < 1.2, f"Δτ {kt[j]['dtau']} vs mass torque {kt[j]['dg']} N·m (K outside 0.8–1.2: wrong KT_NM_PER_A, ratio or sign)")
        step.check(f"{j}: backlash {'—' if fit['backlash_rad'] is None else f'{math.degrees(fit['backlash_rad']):.3f}°'} at {r['backlash_pose_deg']}° (zero gravity)",
                   fit["backlash_rad"] is not None and fit["backlash_rad"] < 2 * D2R,
                   f"{len(r['backlash_samples'])} reversals, |τ| < {r['backlash_threshold_nm']:.3f} N·m; > 2°: a loose stage")
        L = r["latency_ms"]
        step.check(f"{j}: command → response {L['mean']:.1f} ms mean, p95 {L['p95']:.1f}, max {L['max']:.1f} ms (chirp {L['chirp']:.1f})",
                   L["n"] >= 0.8 * cfgs["latency_steps"] and L["p95"] <= 1000 * sup.fb_max_age,
                   f"{L['n']} steps; p95 must stay under the supervisor's stale-feedback bound {1000 * sup.fb_max_age:.0f} ms")
    step.data = dict(fits=fits, rows=rows, kt=kt, config=cfgs)
    if bu.plant is not None:
        step.data["truth_vs_fit"] = truth_vs_fit(bu.plant, fits, rows)
    step.summary = "; ".join(f"{j} F_c {f['coulomb_nm']:.3f} B {f['viscous_nms']:.3f} K {f['kt_scale'] if f['kt_scale'] is not None else '—'} "
                             f"backlash {'—' if f['backlash_rad'] is None else f'{math.degrees(f['backlash_rad']):.2f}°'} "
                             f"latency {f['latency_ms']['mean']:.1f} ms → {Path(f['file']).name}" for j, f in fits.items())


def truth_vs_fit(arm, fits, rows):
    """Virtual run only: the plant's hidden actuator values vs the fit, errors in %."""
    tr = arm.truth()
    out = {}
    for j, f in fits.items():
        t = tr.get(j, {})
        lt = rows[j].get("latency_truth_ms") or {}

        def e(fit, true):
            return None if fit is None or true in (None, 0) else round(100 * (fit - true) / true, 2)
        out[j] = dict(coulomb=dict(true=t.get("coulomb_nm"), fit=f["coulomb_nm"], err_pct=e(f["coulomb_nm"], t.get("coulomb_nm"))),
                      viscous=dict(true=t.get("viscous_nms"), fit=f["viscous_nms"], err_pct=e(f["viscous_nms"], t.get("viscous_nms"))),
                      kt=dict(true=t.get("kt_err"), fit=f["kt_scale"], err_pct=e(f["kt_scale"], t.get("kt_err"))),
                      backlash=dict(true=t.get("backlash_rad"), fit=f["backlash_rad"], err_pct=e(f["backlash_rad"], t.get("backlash_rad"))),
                      latency=dict(true=lt.get("mean"), fit=f["latency_ms"]["mean"], err_pct=e(f["latency_ms"]["mean"], lt.get("mean"))))
    return out
