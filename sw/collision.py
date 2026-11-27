"""Current-based collision detection (D23 / R17, SW2).  Used by sw/safety.py Supervisor; tested in sw/test_s1.py.

The drive reports phase current; torque at the joint is τ_m = Kt · i · ratio. The twin (MuJoCo model of the arm with the
declared payload) predicts what that torque should be for the measured motion: inertia + Coriolis + gravity + its joint
damping. A contact adds an external torque τ_ext that the twin does not predict. Rather than differentiating the encoder
twice to get q̈ (noisy), the detector is a **generalised-momentum observer** (De Luca et al. 2003; Haddadin et al. 2017,
"Robot collisions: a survey on detection, isolation and identification"):

    p = M(q)·q̇,   ṗ = τ_m + τ_passive − b(q, q̇) + Ṁ·q̇ + τ_ext      (b = Coriolis + gravity, MuJoCo qfrc_bias)
    r = K_O · ( p − p₀ − ∫ (τ_m + τ_passive − b + Ṁ·q̇ + r) dt )   ⇒   ṙ = K_O·(τ_ext − r)

so the residual r is τ_ext through a first-order filter with time constant 1/K_O. The velocity noise enters r unfiltered
(r ∝ K_O·M·q̇), so the detector uses r_f = r through a second first-order filter (T_FILT), which is still an unbiased
estimate of τ_ext (no velocity filtering, so no lag error during accelerations). A joint trips when
|r_f,i| > thr_i + rel_i·|τ_m,i| for N_TRIP consecutive cycles (the relative term absorbs the torque-constant error, which
grows with the load). The supervisor latches the "collision" fault and drops to a **compliant hold**: no position gain,
light damping, the twin's gravity torque as feed-forward, so a person can push the arm away; reset() clears it.
"""
from __future__ import annotations

import numpy as np

K_OBS = 150.0          # 1/s observer gain: 6.7 ms time constant
T_FILT = 0.006         # s, residual filter (velocity-noise rejection)
N_TRIP = 2             # consecutive 2 ms cycles over the threshold (debounce) → 4 ms
KP_COMPLIANT = 5.0     # N·m/rad: a soft spring toward an anchor that follows the arm (τ = 0.5 s), so model error doesn't let it sag
KD_COMPLIANT = 1.0     # N·m·s/rad damping in the compliant hold (bench defaults)
T_ANCHOR = 0.5         # s, the hold anchor follows the measured position: a push moves the arm, and it doesn't spring back


class MujocoTwin:
    """Arm dynamics from a MuJoCo model: M(q) (arm block), b(q, q̇), passive damping, gravity."""

    def __init__(self, model, qadr, dadr, payload_body=None, payload_pos=(0.0, 0.0, 0.0)):
        import mujoco
        self.mj = mujoco
        self.m = model
        self.d = mujoco.MjData(model)
        self.qadr, self.dadr = list(qadr), list(dadr)
        self._M = np.zeros((model.nv, model.nv))
        self.pb, self.ppos = payload_body, np.asarray(payload_pos, float)
        if payload_body is not None:
            self._base = (float(model.body_mass[payload_body]), model.body_ipos[payload_body].copy(), model.body_inertia[payload_body].copy())
        self.payload = 0.0

    def set_payload(self, kg):
        """Declared payload at the grasp point (the supervisor's set_payload)."""
        if self.pb is None:
            return
        b = self.pb
        self.m.body_mass[b], self.m.body_ipos[b], self.m.body_inertia[b] = self._base[0], self._base[1].copy(), self._base[2].copy()
        if kg > 0:
            add_point_mass(self.m, b, kg, self.ppos)
        self.payload = float(kg)

    def _set(self, q, qd):
        self.d.qvel[:] = 0
        for k, (a, v) in enumerate(zip(self.qadr, self.dadr)):
            self.d.qpos[a] = q[k]
            self.d.qvel[v] = 0.0 if qd is None else qd[k]

    def dyn(self, q, qd):
        """(M arm block, b = C·q̇ + g, τ_passive) at (q, q̇)."""
        self._set(q, qd)
        self.mj.mj_forward(self.m, self.d)
        if hasattr(self.d, "qM"):                    # MuJoCo ≤ 3.x: mj_fullM(m, dst, qM); 4.x: mj_fullM(m, d, dst)
            self.mj.mj_fullM(self.m, self._M, self.d.qM)
        else:
            self.mj.mj_fullM(self.m, self.d, self._M)
        ix = np.ix_(self.dadr, self.dadr)
        return self._M[ix].copy(), self.d.qfrc_bias[self.dadr].copy(), self.d.qfrc_passive[self.dadr].copy()

    def gravity(self, q):
        self._set(q, None)
        self.mj.mj_forward(self.m, self.d)
        return self.d.qfrc_bias[self.dadr].copy()

    def inverse(self, q, qd, qdd):
        """Inverse dynamics (for a computed-torque feed-forward): M·q̈ + b − τ_passive."""
        M, b, tp = self.dyn(q, qd)
        return M @ np.asarray(qdd, float) + b - tp


def add_point_mass(model, body, mass, pos):
    """Add a point mass (kg) at pos (body frame) to a body whose inertial frame is axis-aligned with it (payload in hand)."""
    m0, c0 = float(model.body_mass[body]), model.body_ipos[body].copy()
    mt = m0 + mass
    c = (m0 * c0 + mass * np.asarray(pos, float)) / mt
    I = model.body_inertia[body].copy()
    for (mm, cc) in ((m0, c0), (mass, np.asarray(pos, float))):       # parallel-axis terms about the new COM (diagonal only)
        dlt = cc - c
        I += mm * np.array([dlt[1] ** 2 + dlt[2] ** 2, dlt[0] ** 2 + dlt[2] ** 2, dlt[0] ** 2 + dlt[1] ** 2])
    model.body_mass[body], model.body_ipos[body], model.body_inertia[body] = mt, c, I


class CollisionDetector:
    """Momentum observer + per-joint thresholds. thr: N·m absolute; rel: fraction of |τ_m|; joints with thr = inf are
    not monitored (e.g. the STS3215 wrist roll, whose current reading is too coarse)."""

    def __init__(self, twin, thr, rel, k_obs=K_OBS, n_trip=N_TRIP, t_filt=T_FILT):
        self.twin = twin
        self.t_filt = float(t_filt)
        self.thr, self.rel = np.asarray(thr, float), np.asarray(rel, float)
        self.k = float(k_obs)
        self.n_trip = int(n_trip)
        self.reset()

    def reset(self):
        self.r = None
        self.rf = None
        self.integ = None
        self.p0 = None
        self.M_prev = None
        self.count = None
        self.tripped = False
        self.peak_ratio = 0.0             # max |r| / threshold seen while not tripped (for logs and tuning)
        self.joint = None

    def update(self, q, qd, tau_m, dt):
        """One control cycle. Returns True on the cycle the detector trips (and stays True until reset())."""
        q, qd, tau_m = (np.asarray(x, float) for x in (q, qd, tau_m))
        M, b, tp = self.twin.dyn(q, qd)
        p = M @ qd
        n = len(q)
        if self.r is None:
            self.r, self.rf, self.integ, self.p0, self.M_prev, self.count = np.zeros(n), np.zeros(n), np.zeros(n), p.copy(), M, np.zeros(n, int)
            return self.tripped
        mdot_qd = (M - self.M_prev) @ qd / dt
        self.M_prev = M
        self.integ += (tau_m + tp - b + mdot_qd + self.r) * dt
        self.r = self.k * (p - self.p0 - self.integ)
        self.rf += (self.r - self.rf) * dt / (self.t_filt + dt)
        lim = self.thr + self.rel * np.abs(tau_m)
        over = np.abs(self.rf) > lim
        self.count = np.where(over, self.count + 1, 0)
        if not self.tripped:
            ratio = np.abs(self.rf) / lim
            self.peak_ratio = max(self.peak_ratio, float(np.max(np.where(np.isfinite(lim), ratio, 0.0))))
            if np.any(self.count >= self.n_trip):
                self.tripped = True
                self.joint = int(np.argmax(self.count))
        return self.tripped
