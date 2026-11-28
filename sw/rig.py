"""The M1 Step-1 lever rig as the software sees it (Fable review 2026-11-28, Critical C1).  python -m sw.rig  (sizing)

Before this, every Step-1 tool ran the ARM: the yaw stage at ratio 2.0 with the forearm in the arm fixture, so on the real
rig (a lever bolted straight to a bare GIM6010-8) every angle and speed would have been doubled, every torque halved in
the supervisor's eyes, the 2 kg soak weight 118 % of rated, and calibration would have sought ±155° stops that do not
exist. A *rig profile* (sw/config/rig_step1.yaml) now describes the machine on the bench, and the bring-up, soak, sysid
and the virtual bench load it with `--rig step1`:

- the CAN map with the true ratio (1.0: bare outputs, no stage) and only the two Step-1 motors (`can_map`);
- the joint description of the bare motors (5 / 11 N·m, 3 / 7 N·m, software range −10° … +90°) for the supervisor
  (`description`);
- the bring-up settings for one *session load* (`apply(cfg, rig, load)`): the rubber stops of rnd/parts/rig.py at −15°
  (reference, just past hanging) and +100° (span), calibration against them, gravity poses on the levers, and the discs
  on each lever for that session (`load_kg`: the twin and the virtual plant both carry them). The guard is interlocked,
  so weights are never changed inside a session: the rig's torque-constant fit uses the weighed lever (sysid
  `kt_method: absolute`) instead of hanging a test weight;
- a MuJoCo model of the rig (two independent pendulum levers; the four arm joints that are not on the rig are inert
  placeholders on their own axis so the six-joint bring-up code runs unchanged) as the twin and the virtual plant (`xml`);
- where masses go (`mass_sites` / `set_mass`: the arm's grasp site, or one weight bolt per lever);
- the sizing: lever torques, the soak load, calibration torques, stop energies and bumper forces (`sizing`, printed by
  `python -m sw.rig` and quoted in docs/M1_RIG.md).
The arm profile (sw/config/can_map.yaml, sw/description/joints.json, sw/config/bringup.yaml) is unchanged and is Step 2's.
"""
from __future__ import annotations

import copy
import functools
import math
from pathlib import Path

import numpy as np
import yaml

ROOT = Path(__file__).resolve().parent
ARM = ["pitch", "roll", "yaw", "elbow", "wpitch", "wroll"]
LEVERS = ["yaw", "wpitch"]
G = 9.81
D2R = math.pi / 180


@functools.lru_cache(maxsize=None)
def _load(name):
    return yaml.safe_load((ROOT / "config" / f"rig_{name}.yaml").read_text())


def load(name="step1"):
    """The rig profile dict (a fresh copy)."""
    return copy.deepcopy(_load(name))


def _rig(r):
    return load(r) if isinstance(r, str) else r


def can_map(rig="step1"):
    """The rig's CAN map in can_map.yaml format, tagged with the rig name (sw/can_map.check validates against the rig)."""
    r = _rig(rig)
    return dict(r["can"], rig=r["name"])


def description(rig="step1"):
    """Joint rows in sw/description/joints.json format (radians) for the joints on the rig."""
    out = []
    for j in _rig(rig)["joints"]:
        j = dict(j)
        j["lower"], j["upper"] = j.pop("lower_deg") * D2R, j.pop("upper_deg") * D2R
        out.append(j)
    return out


def actuator_dir(rig="step1"):
    """Where the rig's fitted actuator models live (never sw/config itself: the arm twin must not load a bare-motor fit)."""
    return ROOT / "config" / f"rig_{_rig(rig)['name']}"


def actuator_model(rig, joint, config_dir=None):
    """Rig datasheet (ESTIMATE) for the bare output, replaced by a fit from the rig's actuator directory when present."""
    from rnd.twin import measured_actuators
    r = _rig(rig)
    base = dict(r["actuators"][joint], source="rig datasheet (ESTIMATE)")
    fit = measured_actuators(config_dir or actuator_dir(r)).get(joint)
    if fit:
        base.update({k: v for k, v in fit.items() if v is not None})
        base["source"] = f"fitted ({fit['file']})"
    return base


def disc_kg(rig="step1"):
    d = _rig(rig)["rig"]["disc"]
    return d["rho"] * math.pi / 4 * (d["d"] ** 2 - d["hole"] ** 2) * d["t"]


def load_kg(rig="step1", load="bringup"):
    """{joint: kg of discs on the lever} for a session load (on top of the always-on carrier)."""
    r = _rig(rig)
    return {j: round(n * disc_kg(r), 4) for j, n in r["rig"]["loads"][load].items()}


def apply(cfg, rig="step1", load="bringup"):
    """sw/config/bringup.yaml settings → the same settings for the rig with one session load (a new dict). A loaded session
    (anything but 'bringup') skips the span check: the +100° stop is uphill and would need more than cal_frac_max."""
    r = _rig(rig)
    out = copy.deepcopy(cfg)
    b = copy.deepcopy(r["bringup"])
    out["joints"] = b.pop("joints")
    for k, v in b.items():
        if isinstance(v, dict) and isinstance(out.get(k), dict) and k in ("sysid", "soak"):
            out[k] = {**out[k], **v}
        else:
            out[k] = v
    out["sysid"].pop("test_mass_kg", None)              # no weight hung mid-session on the rig (kt_method absolute)
    out["soak"]["mass_kg"] = 0.0                         # the soak discs are the session load, on from the start
    out["load"], out["load_kg"] = load, load_kg(r, load)
    if load != "bringup":
        for c in out["joints"].values():
            c["span_check"] = False
    out["rig"] = r["name"]
    return out


def lever_plane_y(rig, joint):
    """Y of the lever bar's mid-plane: post face + standoffs + motor plate + motor + hub + half the bar, on the motor's side."""
    g = _rig(rig)["rig"]
    m, f = g["motors"][joint], g["frame"]
    return m["side"] * (f["post"] / 2 + f["standoff"] + f["plate_t"] + m["h"] + g["hub"]["t"] + g["bar"]["t"] / 2)


# ------------------------------------------------------------------------------------------------ MuJoCo model of the rig
def xml(rig="step1", config_dir=None):
    """MJCF: two pendulum levers on horizontal axes (yaw: GIM, wpitch: DM; +θ swings each lever toward +X), each with its
    bar, hub and weight carrier (the always-on part of the load) and a `mass_<joint>` site at the weight bolt for the
    discs. pitch, roll, elbow, wroll are NOT on the rig: inert 10 g placeholders on vertical axes (no gravity load) that
    the virtual bench holds in its fixture."""
    r = _rig(rig)
    g = r["rig"]
    bar, hub, dd = g["bar"], g["hub"], g["disc"]
    bar_kg = bar["l"] * bar["w"] * bar["t"] * bar["rho"]
    hub_kg = hub["rho"] * math.pi / 4 * hub["d"] ** 2 * hub["t"]
    car_t = g["carrier_kg"] / (dd["rho"] * math.pi / 4 * dd["d"] ** 2)          # an equivalent steel disc for the carrier
    zc = -(bar["l"] / 2 - bar["tail"])
    lo, hi = g["stops_deg"][0] - 5, g["stops_deg"][1] + 5
    bodies = []
    for j in LEVERS:
        A = actuator_model(r, j, config_dir)
        bodies.append(f"""
 <body name="lever_{j}" pos="0 {lever_plane_y(r, j):.4f} {g['axis_z']}">
  <joint name="{j}" axis="0 -1 0" range="{lo} {hi}" damping="{A['viscous_nms']}" armature="{A['armature']}"/>
  <geom name="hub_{j}" type="cylinder" size="{hub['d'] / 2} {hub['t'] / 2}" euler="90 0 0" mass="{hub_kg:.5f}" rgba=".6 .6 .62 1"/>
  <geom name="bar_{j}" type="box" pos="0 0 {zc:.4f}" size="{bar['w'] / 2} {bar['t'] / 2} {bar['l'] / 2}" mass="{bar_kg:.5f}" rgba=".75 .75 .78 1"/>
  <geom name="carrier_{j}" type="cylinder" pos="0 0 {-g['r_mass']}" size="{dd['d'] / 2} {car_t / 2:.5f}" euler="90 0 0" mass="{g['carrier_kg']}" rgba=".3 .3 .32 1"/>
  <site name="mass_{j}" pos="0 0 {-g['r_mass']}" size="0.005"/>
 </body>""")
    ph = "".join(f"""
 <body name="unused_{j}" pos="{0.5 + 0.05 * k} 0.5 0.05"><joint name="{j}" axis="0 0 1" range="-3.2 3.2" damping="0.01"/>
  <geom type="sphere" size="0.01" mass="0.01" contype="0" conaffinity="0"/></body>""" for k, j in enumerate(("pitch", "roll", "elbow", "wroll")))
    return f"""<mujoco model="pip_m1_step1_rig">
<option timestep="0.002"/>
<worldbody>
 <geom name="bench" type="plane" size="1 1 .1" rgba=".7 .7 .72 1"/>{''.join(bodies)}{ph}
</worldbody></mujoco>"""


class RigEnv:
    """The attributes of sw.bench_env.BenchEnv that the bring-up twin and the virtual plant use, for the rig model with
    the session's discs on the levers (`load`: {joint: kg})."""

    def __init__(self, rig="step1", config_dir=None, load=None):
        import mujoco
        from sw.collision import add_point_mass
        self.mj = mujoco
        self.m = mujoco.MjModel.from_xml_string(xml(rig, config_dir))
        for b, kg, p in points(self.m, load or {}):
            add_point_mass(self.m, b, kg, p)
        self.d = mujoco.MjData(self.m)
        jid = {j: mujoco.mj_name2id(self.m, mujoco.mjtObj.mjOBJ_JOINT, j) for j in ARM}
        self.qadr = [int(self.m.jnt_qposadr[jid[j]]) for j in ARM]
        self.dadr = [int(self.m.jnt_dofadr[jid[j]]) for j in ARM]
        self.site = None                                   # no grasp site: masses go on mass_<joint> (mass_sites)

    def reset(self, seed=None):
        self.mj.mj_resetData(self.m, self.d)
        self.mj.mj_forward(self.m, self.d)


def make_env(cfg, config_dir=None):
    """The plant / twin environment for a bring-up cfg: the rig (with the session load) when cfg names one, else the arm's
    bench env."""
    if cfg.get("rig"):
        return RigEnv(cfg["rig"], config_dir, cfg.get("load_kg"))
    from sw.bench_env import BenchEnv
    return BenchEnv(render=False)


# ------------------------------------------------------------------------------------------------ test / soak masses
def mass_sites(m):
    """{key: (body id, pos in body frame)}: '*' = the arm's grasp site (one mass loads every joint), else one per rig
    lever (`mass_<joint>` sites)."""
    import mujoco
    out = {}
    for i in range(m.nsite):
        n = mujoco.mj_id2name(m, mujoco.mjtObj.mjOBJ_SITE, i) or ""
        if n == "grasp":
            out["*"] = (int(m.site_bodyid[i]), m.site_pos[i].copy())
        elif n.startswith("mass_"):
            out[n[5:]] = (int(m.site_bodyid[i]), m.site_pos[i].copy())
    return out


def points(m, spec):
    """[(body, kg, pos)] for a mass spec: a number (the arm: at the grasp; the rig: that much on every lever) or
    {joint: kg} (the rig: per lever; the arm: the sum at the grasp)."""
    if not spec:
        return []
    sites = mass_sites(m)
    if "*" in sites:
        kg = float(sum(spec.values())) if isinstance(spec, dict) else float(spec)
        return [(sites["*"][0], kg, sites["*"][1])] if kg > 0 else []
    if isinstance(spec, dict):
        return [(sites[j][0], float(kg), sites[j][1]) for j, kg in spec.items() if j in sites and kg]
    return [(b, float(spec), p) for b, p in sites.values()]


def capture(m):
    """The mass / COM / inertia of every mass-site body now (the base that set_mass restores)."""
    return {b: (float(m.body_mass[b]), m.body_ipos[b].copy(), m.body_inertia[b].copy()) for b, _ in mass_sites(m).values()}


def set_mass(m, spec, base):
    """Restore the mass-site bodies to `base`, then add the point masses of `spec` (0 / None / {} = none)."""
    from sw.collision import add_point_mass
    for b, (mass, ipos, inertia) in base.items():
        m.body_mass[b], m.body_ipos[b], m.body_inertia[b] = mass, ipos.copy(), inertia.copy()
    for b, kg, p in points(m, spec):
        add_point_mass(m, b, kg, p)


def describe_mass(spec):
    if isinstance(spec, dict):
        return ", ".join(f"{j} lever +{kg:.3f} kg" for j, kg in spec.items() if kg) or "none"
    return f"{spec} kg"


def describe_load(cfg):
    """Operator text for the session load of a rig cfg."""
    r = _rig(cfg["rig"])
    n = r["rig"]["loads"][cfg.get("load", "bringup")]
    return (f"session load '{cfg.get('load', 'bringup')}': on each lever the bar, hub and weight carrier ({r['rig']['carrier_kg']} kg at "
            f"{r['rig']['r_mass']} m) plus " + ", ".join(f"{j} {k} discs ({k * disc_kg(r):.3f} kg)" for j, k in n.items())
            + ", bolted through, nylocs tight; guard on and its interlock closed")


# ------------------------------------------------------------------------------------------------ sizing
def _lever(r, j, n_disc):
    g = r["rig"]
    bar, hub, dd = g["bar"], g["hub"], g["disc"]
    bar_kg = bar["l"] * bar["w"] * bar["t"] * bar["rho"]
    hub_kg = hub["rho"] * math.pi / 4 * hub["d"] ** 2 * hub["t"]
    a, b = bar["tail"], bar["l"] - bar["tail"]
    J_bar = bar_kg / bar["l"] * (a ** 3 + b ** 3) / 3 + bar_kg * bar["w"] ** 2 / 12
    J_hub = 0.5 * hub_kg * (hub["d"] / 2) ** 2
    mw = g["carrier_kg"] + n_disc * disc_kg(r)
    J_w = mw * g["r_mass"] ** 2 + 0.5 * mw * (dd["d"] / 2) ** 2
    arm = r["actuators"][j]["armature"]
    m_r = bar_kg * (bar["l"] / 2 - bar["tail"]) + mw * g["r_mass"]          # first moment about the axis (kg·m)
    return dict(m_w=mw, g_max=G * m_r, J=J_bar + J_hub + J_w + arm, J_lever=J_bar + J_hub + J_w, m=bar_kg + hub_kg + mw, m_r=m_r,
                bar_kg=bar_kg, hub_kg=hub_kg)


def _soak_rms(gmax, lim_deg, jog_deg, margin_deg=3.0, n=2000):
    """RMS gravity torque of the soak's 0.25 Hz sine: centre where gravity loads the joint most inside the supervisor's
    range shrunk by the amplitude (sw/sysid._loaded_centre), ± the jog amplitude."""
    hi = (lim_deg - margin_deg - jog_deg) * D2R - 0.05
    c = min(math.pi / 2, hi)
    t = np.linspace(0, 2 * math.pi, n, endpoint=False)
    q = c + jog_deg * D2R * np.sin(t)
    return float(np.sqrt(np.mean((gmax * np.sin(q)) ** 2))), math.degrees(c)


def _t_cal(rated, g_max, ref_deg, other_deg, span, rest=0.0):
    """sw/bringup.step_calibrate's calibration torque: max(0.15 rated, 1.4 × gravity max over the seek + 0.10 rated); the
    seek covers rest ± 15°, the reference stop and, with the span check, the other stop."""
    pts = [rest - 15, rest + 15, ref_deg] + ([other_deg] if span else [])
    a, b = min(pts) * D2R, max(pts) * D2R
    xs = np.linspace(a, b, 181)
    gm = float(np.max(np.abs(g_max * np.sin(xs))))
    return max(0.15 * rated, 1.4 * gm + 0.10 * rated), gm


def sizing(rig="step1"):
    """Per lever: gravity torque (lever horizontal) per session load, the soak's RMS fraction of rated, the calibration
    torque per load, stop energies, bumper deflection / force, hub and gear loads at a stop hit."""
    r = _rig(rig)
    g = r["rig"]
    desc = {j["name"]: j for j in description(r)}
    cfg = r["bringup"]
    k = g["bumper"]["k_n_per_m"]
    lo_s, hi_s = g["stops_deg"]
    out = {}
    for j in LEVERS:
        d = desc[j]
        rated, peak = d["rated_nm"], d["peak_nm"]
        lv = {s: _lever(r, j, g["loads"][s][j]) for s in g["loads"]}
        base, soak = lv["bringup"], lv["soak"]
        jc = cfg["joints"][j]
        rms, centre = _soak_rms(soak["g_max"], math.degrees(d["upper"]), jc["jog"])
        cal = {s: _t_cal(rated, v["g_max"], jc["stop"][1], jc["other"], jc["span_check"] and s == "bringup") for s, v in lv.items()}
        # stop energies (the soak load: the largest inertia and gravity); runaways start at the far end of the software range
        U = lambda th: -soak["g_max"] * math.cos(th)                     # potential (J), hanging = minimum
        ke_cap = 0.5 * soak["J"] * g["motors"][j]["no_load_rad_s"] ** 2

        def run(tau, frm, to):
            return min(tau * abs(to - frm) - (U(to) - U(frm)), ke_cap)
        lo, hi = d["lower"], d["upper"]
        cases = [("calibration seek 0.3 rad/s", "lower", 0.5 * soak["J"] * 0.3 ** 2),
                 (f"jog {0.1 * d['max_speed_rad_s']:.2f} rad/s", "either", 0.5 * soak["J"] * (0.1 * d["max_speed_rad_s"]) ** 2),
                 ("supervisor speed fault 4 rad/s", "either", 0.5 * soak["J"] * 4.0 ** 2),
                 (f"runaway up at the flash limit ({rated:g} N·m), −10° → {hi_s}°", "upper", run(rated, lo, hi_s * D2R)),
                 (f"runaway down at the flash limit, +90° → {lo_s}°", "lower", run(rated, hi, lo_s * D2R)),
                 (f"runaway down at the peak {peak:g} N·m (flash limit not enforced)", "lower", run(peak, hi, lo_s * D2R))]
        stops = []
        for name, which, e in cases:
            dlt = math.sqrt(2 * e / k)
            F = k * dlt
            alpha = F * g["r_mass"] / soak["J"]
            hub = F * abs(1 - soak["m_r"] * g["r_mass"] / soak["J_lever"])   # pivot reaction, stop at r (centre of percussion)
            gear = r["actuators"][j]["armature"] * alpha + rated              # rotor deceleration through the gear + motor torque
            stops.append(dict(case=name, stop=which, energy_j=round(e, 3), defl_mm=round(dlt * 1000, 1), strain=round(dlt / g["bumper"]["h"], 2),
                              force_n=round(F), hub_n=round(hub), gear_nm=round(gear, 1)))
        out[j] = dict(rated_nm=rated, peak_nm=peak, disc_kg=round(disc_kg(r), 4),
                      loads={s: dict(discs=g["loads"][s][j], mass_kg=round(v["m_w"], 3), g_horizontal_nm=round(v["g_max"], 3),
                                     frac_rated=round(v["g_max"] / rated, 3), t_cal_nm=round(cal[s][0], 3), t_cal_frac=round(cal[s][0] / rated, 3),
                                     g_at_ref_stop_nm=round(abs(v["g_max"] * math.sin(jc["stop"][1] * D2R)), 3),
                                     J=round(v["J"], 5)) for s, v in lv.items()},
                      soak_rms_nm=round(rms, 3), soak_rms_frac=round(rms / rated, 3), soak_centre_deg=round(centre, 1),
                      sag_testable=base["g_max"] >= 0.08 * rated, cal_ok=all(c[0] <= cfg["cal_frac_max"] * rated for c in cal.values()),
                      centre_of_percussion_m=round(soak["J_lever"] / soak["m_r"], 3), stop_r_m=g["r_mass"],
                      stall_at_stop_n=round(peak / g["r_mass"], 1), stops=stops)
    return out


def main():
    s = sizing()
    for j, v in s.items():
        print(f"{j}: rated {v['rated_nm']} N·m, disc {v['disc_kg']} kg")
        for n, L in v["loads"].items():
            print(f"  {n:8s} {L['discs']} discs, {L['mass_kg']} kg at the bolt: {L['g_horizontal_nm']} N·m horizontal = {L['frac_rated']:.0%} rated; "
                  f"calibration torque {L['t_cal_nm']} N·m = {L['t_cal_frac']:.0%} (gravity at the reference stop {L['g_at_ref_stop_nm']}); J {L['J']} kg·m²")
        print(f"  soak RMS {v['soak_rms_nm']} N·m = {v['soak_rms_frac']:.0%} rated (centre {v['soak_centre_deg']}°); sag test "
              f"{'OK' if v['sag_testable'] else 'NOT testable'}; calibration within cal_frac_max: {v['cal_ok']}")
        print(f"  centre of percussion {v['centre_of_percussion_m']} m vs stop r {v['stop_r_m']} m; static stall at the stop {v['stall_at_stop_n']} N")
        for c in v["stops"]:
            print(f"    {c['case']:62s} [{c['stop']:6s}] {c['energy_j']:7.3f} J  bumper {c['defl_mm']:5.1f} mm ({c['strain']:.0%})  "
                  f"{c['force_n']:5d} N  hub {c['hub_n']:4d} N  gear {c['gear_nm']} N·m")


if __name__ == "__main__":
    main()
