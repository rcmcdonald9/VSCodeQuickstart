"""Finite-element checks of the critical printed parts (linear elastic, tetrahedral; gmsh + scikit-fem).  python -m rnd.fea

Material: printed PETG, E 1.8 GPa, ν 0.38. Allowables:
- 25 MPa in-layer (50 MPa × 0.5 safety)
- **12 MPa across layers**: layer adhesion ×0.5 then safety ×2 (ESTIMATE). Peak von Mises is compared with this
  conservative value, because the stress direction vs the layers isn't resolved here.

Each case lists its loads (worst static ×1.5 dynamic, or actuator peak for belts). Peak stress is reported at the
99.5th percentile of element stresses, to drop single-element singularities at sharp corners.
"""
from __future__ import annotations

import functools
import json
import math
import os
import tempfile
from pathlib import Path

import numpy as np

DOCS = Path(__file__).resolve().parent.parent / "docs" / "rnd"
E, NU = 1800.0, 0.38                   # MPa
ALLOW = 12.0


def mesh_part(part, h=3.0):
    """Mesh with gmsh; on a PLC failure (self-intersecting facets after OCC import) retry with healing and other sizes."""
    last = None
    for k, hh in enumerate((h, h * 0.85, h * 1.15, h * 0.7, h * 1.3)):
        try:
            return _mesh_part(part, hh, heal=k > 0)
        except Exception as e:                     # gmsh raises a plain Exception
            last = e
    raise last


def _mesh_part(part, h=3.0, heal=False):
    import gmsh
    import meshio
    from build123d import export_step
    from skfem import MeshTet
    with tempfile.TemporaryDirectory() as td:
        stp = Path(td) / "p.step"
        export_step(part, str(stp))
        gmsh.initialize()
        gmsh.option.setNumber("General.Terminal", 0)
        if heal:
            for opt in ("Geometry.OCCFixDegenerated", "Geometry.OCCFixSmallEdges", "Geometry.OCCFixSmallFaces", "Geometry.OCCSewFaces"):
                gmsh.option.setNumber(opt, 1)
        gmsh.model.occ.importShapes(str(stp))
        gmsh.model.occ.synchronize()
        gmsh.option.setNumber("Mesh.CharacteristicLengthMax", h)
        gmsh.option.setNumber("Mesh.CharacteristicLengthMin", h * 0.35)
        try:
            gmsh.model.mesh.generate(3)
        except Exception:
            gmsh.finalize()
            raise
        msh = Path(td) / "p.msh"
        gmsh.write(str(msh))
        gmsh.finalize()
        m = meshio.read(str(msh))
    tets = np.vstack([c.data for c in m.cells if c.type == "tetra"])
    P = m.points
    vol = np.abs(np.einsum("ij,ij->i", P[tets[:, 1]] - P[tets[:, 0]],
                           np.cross(P[tets[:, 2]] - P[tets[:, 0]], P[tets[:, 3]] - P[tets[:, 0]]))) / 6
    tets = tets[vol > 1e-6 * np.median(vol)]                  # drop zero-volume slivers (they make the stiffness matrix singular)
    used = np.unique(tets)
    remap = -np.ones(len(m.points), int)
    remap[used] = np.arange(len(used))
    return MeshTet(m.points[used].T, remap[tets].T)


def solve(mesh, fixed, loads):
    """fixed(x) -> bool mask on facet midpoints; loads: list of (select(x) -> bool, total force vector N)."""
    from skfem import Basis, ElementTetP1, ElementVector, FacetBasis, LinearForm, asm, condense, solve as sk_solve
    from skfem.models.elasticity import lame_parameters, linear_elasticity
    e = ElementVector(ElementTetP1())
    ib = Basis(mesh, e)
    K = asm(linear_elasticity(*lame_parameters(E, NU)), ib)
    f = np.zeros(K.shape[0])
    bnd = mesh.boundary_facets()
    mids = mesh.p[:, mesh.facets[:, bnd]].mean(axis=1)
    for sel, F in loads:
        facets = bnd[sel(mids)]
        if len(facets) == 0:
            raise ValueError("load selection is empty")
        fb = FacetBasis(mesh, e, facets=facets)
        area = asm(LinearForm(lambda v, w: v[0] * 0 + 1.0), FacetBasis(mesh, ElementTetP1(), facets=facets)).sum()
        t = np.asarray(F, float) / area

        @LinearForm
        def trac(v, w):
            return t[0] * v[0] + t[1] * v[1] + t[2] * v[2]
        f += asm(trac, fb)
    fixed_facets = bnd[fixed(mids)]
    if len(fixed_facets) == 0:
        raise ValueError("fixed selection is empty")
    D = ib.get_dofs(facets=fixed_facets).all()
    u = sk_solve(*condense(K, f, D=D))
    # element von Mises (P1 → constant strain per element)
    p, t4 = mesh.p, mesh.t
    vm = np.empty(t4.shape[1])
    lam, mu = lame_parameters(E, NU)
    ux = u[ib.nodal_dofs[0]], u[ib.nodal_dofs[1]], u[ib.nodal_dofs[2]]
    for k in range(t4.shape[1]):
        X = p[:, t4[:, k]]
        Jm = np.vstack([X[:, 1] - X[:, 0], X[:, 2] - X[:, 0], X[:, 3] - X[:, 0]]).T
        if abs(np.linalg.det(Jm)) < 1e-9:                     # degenerate (sliver) tet from the mesher: no stress contribution
            vm[k] = 0.0
            continue
        Ji = np.linalg.inv(Jm)
        G = np.array([[-1, -1, -1], [1, 0, 0], [0, 1, 0], [0, 0, 1]]) @ Ji           # ∇N (4×3)
        Ue = np.array([[ux[c][t4[i, k]] for c in range(3)] for i in range(4)])     # 4×3
        grad = Ue.T @ G                                                            # 3×3
        eps = 0.5 * (grad + grad.T)
        sig = 2 * mu * eps + lam * np.trace(eps) * np.eye(3)
        s = sig - np.trace(sig) / 3 * np.eye(3)
        vm[k] = math.sqrt(1.5 * np.sum(s * s))
    disp = np.sqrt(sum(c ** 2 for c in ux))
    return dict(vm99=float(np.quantile(vm, 0.995)), vmax=float(vm.max()), dmax=float(disp.max()), n=t4.shape[1])


def cases():
    from build123d import Pos
    from .params import BASELINE as d, G
    from .parts import arm, base, shoulder
    from .parts.kx import KX74, collar, HUB_STEP, LIP_T
    C = []
    # 1. KX74 collar: teeth pulled up by 1.5 × preload, the lip held down by the spring stack
    kx = KX74
    z_lip = -HUB_STEP - kx.spring_t - LIP_T
    zt = kx.lug_back_z - kx.stroke
    C.append(("KX74 collar (locked): teeth at 1.5 × preload (240 N)", collar(kx), 1.6,
              lambda x: (np.abs(x[2] - (z_lip + LIP_T)) < 0.3) & (np.hypot(x[0], x[1]) < kx.r_collar_in + 0.5),
              [(lambda x, zt=zt: (np.abs(x[2] - zt) < 1.2) & (np.hypot(x[0], x[1]) < kx.r_collar_in - 0.5), (0, 0, 240.0))]))
    # 2. Elbow housing (upper bone): bonded to the CF neck tube; the forearm load (outboard 3 kg ×1.5, applied on the +Y side
    #    link at FORE_Y) split over the two elbow 61808 bosses. (The motor-torque reaction path is still open: rnd/integration.py.)
    #    Belt tension no longer reaches the shell: it closes inside the elbow belt cassette (case 5).
    ub = max(arm.upper_bone_with_caps().solids(), key=lambda s: s.volume)   # D19: bolted seat caps taken as bonded
    w_out = 3.0 * G * 1.5
    zE, zS, hy = -arm.ELBOW_Z, -arm.ELBOW_Z + arm.BELT_C, arm.HOUSING["y"] / 2
    fp, fm = w_out * (arm.FORE_Y + hy) / (2 * hy), -w_out * (arm.FORE_Y - hy) / (2 * hy)
    C.append(("Upper bone / elbow housing: forearm load ×1.5 on the elbow bosses", ub, 3.0,
              lambda x: (x[2] > -80) & (np.hypot(x[0], x[1]) < 23),          # bonded to the CF neck tube in its socket
              [(lambda x: (np.hypot(x[0], x[2] - zE) < 26.5) & (np.abs(x[1] - hy) < 4.5), (fp, 0, 0)),
               (lambda x: (np.hypot(x[0], x[2] - zE) < 26.5) & (np.abs(x[1] + hy) < 4.5), (fm, 0, 0))]))
    # 3. Wheel pod: 1/3 robot weight × 2 (bumps) radial + 0.5 g lateral on the bearing seats, bolted under the deck
    pod = base.pod()
    fw = 25.6 * G / 3 * 2
    bbp = pod.bounding_box()
    C.append(("Wheel pod: wheel load 168 N (×2 bumps) + 0.5 g lateral", pod, 2.0,
              lambda x, zt=bbp.max.X: x[0] > zt - 4.5,
              [(lambda x: np.hypot(x[0], x[1]) < 27, (-fw, 0.5 * fw, 0.0))]))
    # 4. Shoulder bracket: V-band moment 19 N·m (×1.5 incl.) as opposite forces on the two 61808 seats + belt peak 691 N
    br = shoulder.bracket()
    F_m = 19.0 / ((shoulder.W_OUT - shoulder.W_IN) / 1000)
    C.append(("Shoulder pitch bracket: arm moment 19 N·m + belt peak 691 N", br, 3.0,
              lambda x, x0=-shoulder.BELT_C / 2 - 10 - 95 + 8: x[0] < x0,
              [(lambda x: (np.hypot(x[0], x[2]) < 27) & (np.abs(x[1] - shoulder.W_OUT) < 4.5), (0, 0, -F_m)),
               (lambda x: (np.hypot(x[0], x[2]) < 27) & (np.abs(x[1] - shoulder.W_IN) < 4.5), (0, 0, F_m)),
               (lambda x: (np.hypot(x[0] + shoulder.BELT_C, x[2]) < 20.5) & (np.abs(np.abs(x[1] - (shoulder.W_IN + shoulder.W_OUT) / 2) - 13) < 5), (691.0, 0, 0))]))
    # 5. Elbow belt cassette plate: hangs on its 6708 (elbow axis); the small hub pulls the bushing seat with 60 % of the
    #    peak belt load (the pulley sits a little nearer plate B).
    cp = arm.cassette_plate("B")
    C.append(("Elbow belt cassette plate: 60 % of the belt peak (415 N)", cp, 2.0,
              lambda x: np.hypot(x[0], x[2] - zE) < 25.3,
              [(lambda x: np.hypot(x[0], x[2] - zS) < 19.4, (0, 0, -0.6 * 691.0))]))
    C += c1_tie_cases()
    C.append(carriage_case())
    # 6. Roll link: bolted to the arm-side ferrule; roll belt peak 768 N between the seats (half per wall) + the arm moment
    #    19 N·m as a couple on the two 61808 seats (walls 25.4 mm apart)
    from .parts import roll as rl
    if rl.INLINE:
        C += inline_roll_cases()
        C += [case_torque_hanger()]
        return C
    W1, W2, RY, RC = rl.W1, rl.W2, rl.ROLL_Y, rl.ROLL_C
    Fc = 19.0 / ((W2[0] + W2[1] - W1[0] - W1[1]) / 2000)
    inw = lambda x, W: (x[0] > W[0] - 0.2) & (x[0] < W[1] + 0.2)
    C.append((f"Roll link: roll belt peak 768 N + arm moment 19 N·m (couple {Fc:.0f} N)", rl.roll_link(), 2.5,
              lambda x: (np.abs(x[1] - 8) < 0.3) & (np.hypot(x[0], x[2]) < 45),
              [(lambda x, W=W: (np.hypot(x[1] - RY, x[2]) < 26.3) & inw(x, W), (0, 0, 384.0 + sg * Fc)) for W, sg in ((W1, 1), (W2, -1))]
              + [(lambda x, W=W: (np.hypot(x[1] - RY, x[2] - RC) < 19.4) & inw(x, W), (0, 0, -384.0)) for W in (W1, W2)]))
    # 7. Clevis + yaw block: held on the roll shaft by the clevis arms; yaw belt peak 691 N between the seats + the upper-arm
    #    moment 19 N·m as a couple on the two yaw 61808 seats + its weight
    P1, P2, YC = rl.P1, rl.P2, rl.YAW_C
    Fy = 19.0 / ((P1[0] + P1[1] - P2[0] - P2[1]) / 2000)
    inp = lambda x, P: (x[2] > P[0] - 0.2) & (x[2] < P[1] + 0.2)
    C.append((f"Clevis + yaw block: yaw belt peak 691 N + arm moment 19 N·m (couple {Fy:.0f} N)", rl.clevis_and_yaw_block() + rl.yaw_plate_bot(), 2.5,   # loop 44 made the bottom plate a bolted print: taken as bonded here
              lambda x: (np.hypot(x[1] - RY, x[2]) < 20.5) & ((x[0] < W1[0] - 1) | (x[0] > W2[1] + 1)),
              [(lambda x, P=P: (np.hypot(x[0], x[1] - RY) < 26.3) & inp(x, P), (-345.0 + sg * Fy, 0, -28.0)) for P, sg in ((P1, 1), (P2, -1))]
              + [(lambda x, P=P: (np.hypot(x[0] + YC, x[1] - RY) < 19.4) & inp(x, P), (345.0, 0, 0)) for P in (P1, P2)]))
    C.append(case_torque_hanger())
    return C


def case_torque_hanger():
    # 8. Standard torque hanger (wheel version): GIM peak 11 N·m on the Ø6 pin at r 30 → 367 N on the fork wall
    from .parts import base as bs, cartridge_m as cm
    reach = (bs.Z1 - bs.DECK_T) - bs.Z_AX - cm.TORQUE_PIN["r"]
    return ("Torque hanger (wheel): GIM peak 11 N·m → 367 N on the fork", cm.torque_hanger(reach, slot_in=4.0), 1.5,
            lambda x, r=reach: np.abs(x[0] - r) < 0.3,
            [(lambda x: (np.abs(x[1] - 3.2) < 0.3) & (x[0] < 10) & (x[2] > 2) & (x[2] < 9), (0, 367.0, 0))])


def _half_annulus_centroid(r0, r1):
    return 4 * (r1 ** 3 - r0 ** 3) / (3 * math.pi * (r1 ** 2 - r0 ** 2))


def c1_tie_cases():
    """C1 (Fable review 2026-11-27): the printed parts of the two tie-rod joints (rnd/joints.py), held at their bearing seats.
    The tie preload (× 0.75 creep) is a pair of opposite face loads (ring seat ↔ crank / flange face); the joint wrench is
    a couple on the two halves of the loaded face + the pull-off. Service = the worst basket/hug × 1.5 (no gap, so the
    clamped interface is taken as bonded). Fault = the worst 140 N bump / stall, applied × 0.5 together with the preload:
    linear, so its margin vs 12 MPa equals the full fault vs 24 MPa (≈ the across-layer ultimate, ESTIMATE); the joint
    gaps at that load, so the bonded interface over-states the stress at the tension edge."""
    from .joints import DOCS as JD, SERVICE
    from .parts import roll as rl, shoulder as sh
    try:
        rows = json.loads((JD / "joints.json").read_text())["joints"]
    except (OSError, ValueError, KeyError):
        rows = []

    def worst(joint, service, default):
        m = [(r["M"], r["N"]) for r in rows if r["joint"] == joint and "M" in r and (r["case"] in SERVICE) == service]
        return max(m) if m else default
    C = []
    # pitch tube + hub (module frame, axis Y): ring seat in the hub's detach-face recess, crank on the tube's inboard end,
    # moment on the ferrule face (r 21–45.5 at y = 20), bearings A and B on the tube (r 20)
    h = sh.HUB_TIE
    Fp = h["n"] * h["preload"] * h["creep"]
    y_face, y_seat, y_in = sh.Y_FER + 18, sh.Y_FER + 18 - h["recess"], sh.Y_TUBE_IN
    zc = _half_annulus_centroid(21.2, 45.5)
    rho = lambda x: np.hypot(x[0], x[2])
    part = sh.pitch_shaft() + sh.pitch_hub()
    fixed = lambda x: (np.abs(rho(x) - 20.0) < 0.35) & (((x[1] > sh.W_IN - 3.5) & (x[1] < sh.W_IN + 3.5)) | ((x[1] > sh.W_OUT - 3.5) & (x[1] < sh.W_OUT + 3.5)))
    for tag, (M, N), k in (("service: basket/hug × 1.5", worst("pitch hub ↔ tube", True, (29.6, 0.0)), 1.0),
                           ("fault: bump/stall × 0.5", worst("pitch hub ↔ tube", False, (184.0, 343.0)), 0.5)):
        F = M * 1000 / (2 * zc)
        C.append((f"Pitch tube + hub (C1 tie, {h['n']} × M4 at {Fp / 1e3:.1f} kN), {tag}, M {M:.0f} N·m, pull-off {N:.0f} N", part, 3.0, fixed,
                  [(lambda x: (np.abs(x[1] - y_seat) < 0.3) & (rho(x) > 11.0) & (rho(x) < 21.0), (0, -Fp * k, 0)),
                   (lambda x: (np.abs(x[1] - y_in) < 0.3) & (rho(x) > 10.5), (0, Fp * k, 0)),
                   (lambda x: (np.abs(x[1] - y_face) < 0.3) & (rho(x) > 21.2) & (x[2] > 0), (0, (F + N / 2) * k, 0)),
                   (lambda x: (np.abs(x[1] - y_face) < 0.3) & (rho(x) > 21.2) & (x[2] < 0), (0, (-F + N / 2) * k, 0))]))
    # yaw tube (roll frame, axis Z at y = ROLL_Y): ring seat in the top counterbore, the 6061 flange on the end face,
    # bearings A (top plate) and B (bottom plate)
    y = rl.YAW_TIE
    Fp = y["n"] * y["preload"] * y["creep"]
    z_seat, z_end = rl.yaw_tie_levels()["seat"], rl.FL_TOP - rl.FL_REC
    zc = _half_annulus_centroid(14.1, 20.0)
    rh = lambda x: np.hypot(x[0], x[1] - rl.ROLL_Y)
    fixed = lambda x: (np.abs(rh(x) - 20.0) < 0.35) & (((x[2] > rl.P1[0]) & (x[2] < rl.P1[1])) | ((x[2] > rl.YAW_BRG_B[0]) & (x[2] < rl.YAW_BRG_B[1])))
    for tag, (M, N), k in (("service: basket/hug × 1.5", worst("yaw flange ↔ tube", True, (32.5, 100.0)), 1.0),
                           ("fault: bump/stall × 0.5", worst("yaw flange ↔ tube", False, (98.5, 166.0)), 0.5)):
        F = M * 1000 / (2 * zc)
        C.append((f"Yaw tube (C1 tie, {y['n']} × M3 at {Fp / 1e3:.1f} kN), {tag}, M {M:.0f} N·m, pull-off {N:.0f} N", rl.yaw_shaft(), 3.0, fixed,
                  [(lambda x: (np.abs(x[2] - z_seat) < 0.3) & (rh(x) > 8.2) & (rh(x) < 17.0), (0, 0, -Fp * k)),
                   (lambda x: (np.abs(x[2] - z_end) < 0.3) & (rh(x) > 12.0) & (x[0] > 0), (0, 0, (Fp / 2 + F - N / 2) * k)),
                   (lambda x: (np.abs(x[2] - z_end) < 0.3) & (rh(x) > 12.0) & (x[0] < 0), (0, 0, (Fp / 2 - F - N / 2) * k))]))
    return C


@functools.lru_cache(maxsize=1)
def carriage_wrench():
    """Worst service wrench (× 1.5) one arm puts on the carriage's side face, from the twin with the D20 basket design
    (rnd/basket.py: D20 CAD masses, 4.54 kg per hand, every collision-free IK solution of the basket targets, + the hug
    poses of rnd/joints.py): everything outboard of the pitch joint (sh_r subtree) + the static pitch module
    (params pitch_module_mass, taken at the pitch axis, ESTIMATE), about the centre of the carriage's −Y side face
    (torso frame: x MAST_X, y −(W/2 + 13), z +10 mm from the pitch axis). → (F N, M N·m) in the robot frame."""
    import mujoco
    from .basket import POSES as BPOSES, HALF_SPAN, PER_HAND, design
    from .joints import _model, _q, DYN, G
    from .parts import upper
    db = design()
    R, _ = _model(db)
    m, dat = R.m, R.d
    bid = lambda n: mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, n)
    sh, torso, fore, pay = bid("sh_r"), bid("torso"), bid("fore_r"), bid("payload_r")
    sub = [b for b in range(m.nbody) if any(p == sh for p in _ancestors(m, b))]
    off = np.array([upper.MAST_X / 1000, -(upper.W / 2 + 13) / 1000, 0.010])          # torso frame (= robot frame, lift along z)

    def wr(q, ext=(), payload_on=True):
        dat.qpos[R.adr] = q
        mujoco.mj_kinematics(m, dat)
        P = dat.xpos[torso] + dat.xmat[torso].reshape(3, 3) @ off
        pts = [(dat.xipos[b], m.body_mass[b]) for b in sub if payload_on or b != pay] + [(dat.xpos[sh], db.arm.pitch_module_mass)]
        F, M = np.zeros(3), np.zeros(3)
        for x, mass in pts:
            fg = np.array([0, 0, -G * mass])
            F, M = F + fg, M + np.cross(x - P, fg)
        for x, fe in ext:
            F, M = F + fe, M + np.cross(x - P, fe)
        return F * DYN, M * DYN
    best = None
    for name, z, xs, rule in BPOSES:
        for x in xs:
            for q in R.solve(np.array([x, -HALF_SPAN, z]), [0, 0, -1.0], seeds=6):
                w = wr(q)
                if best is None or np.linalg.norm(w[1]) > np.linalg.norm(best[1]):
                    best = (*w, f"basket {name}, x {x:.2f}")
    for yaw in (-45, 0, 45):
        for el in (-90, 90):
            q = _q(0, 0, yaw, el)
            dat.qpos[R.adr] = q
            mujoco.mj_kinematics(m, dat)
            xh = dat.xpos[fore] + dat.xmat[fore].reshape(3, 3) @ np.array([0, 0, -0.30])
            w = wr(q, [(xh, np.array([0, 0, -G * PER_HAND]))], payload_on=False)
            if np.linalg.norm(w[1]) > np.linalg.norm(best[1]):
                best = (*w, f"hug yaw {yaw}°, elbow {el}°")
    return tuple(best[0].round(1)), tuple(best[1].round(2)), best[2]


def _ancestors(m, b):
    out = []
    while b > 0:
        out.append(b)
        b = m.body_parentid[b]
    return out


def carriage_case():
    """C3 follow-up (wave 4, D20 only): the lift carriage with its side walls thinned 12 → 8 mm over the slot under the
    D20 pitch cans (rnd/parts/upper.py carriage, read here with arm_motors = d20 whatever the run's motor set). Held at
    the inner faces of the 8 × 625 bearing bosses (all DOF: the belt clamp / pawl's vertical reaction is taken there
    too, ESTIMATE). Each side face (|y| = W/2 + 13, outside the slot and the bosses) takes one arm's wrench
    (carriage_wrench(), the left arm mirrored): the force as a uniform traction, the moment as couples between the
    upper/lower halves (normal, M_x) and the front/back halves (vertical for M_y, normal for M_z), each split at the
    face's area centroid. The shoulder-module ↔ carriage interface is not drawn (upper.py: "both pitch modules bolt
    to it"), so the load is spread over the whole side face: ESTIMATE until it is."""
    import dataclasses
    from build123d import Box as _Box, Pos as _Pos
    from .parts import upper
    D0 = upper.D
    upper.D = dataclasses.replace(D0, arm=dataclasses.replace(D0.arm, arm_motors="d20"))
    try:
        L = 750.0
        part = upper.carriage(L)
    finally:
        upper.D = D0
    F, M, pose = carriage_wrench()
    yf = upper.W / 2 + 13
    xm, zc = upper.MAST_X, L + 10.0
    faces = [f for f in part.faces() if abs(abs(f.center().Y) - yf) < 0.01 and abs(abs(f.normal_at().Y) - 1) < 1e-6 and f.center().Y < 0]
    A = sum(f.area for f in faces)
    cx = sum(f.area * f.center().X for f in faces) / A
    cz = sum(f.area * f.center().Z for f in faces) / A

    def cen(box):
        parts = [(f & box) for f in faces]
        a = sum(p.area for p in parts)
        return a, sum(p.area * p.center().X for p in parts) / a, sum(p.area * p.center().Z for p in parts) / a
    big = 400.0
    _, _, zu = cen(_Pos(cx, 0, cz + big / 2) * _Box(big, big, big))
    _, _, zl = cen(_Pos(cx, 0, cz - big / 2) * _Box(big, big, big))
    _, xf, _ = cen(_Pos(cx + big / 2, 0, cz) * _Box(big, big, big))
    _, xb, _ = cen(_Pos(cx - big / 2, 0, cz) * _Box(big, big, big))
    Mx, My, Mz = (1000 * v for v in M)                                  # N·mm about (cx, −yf, cz) (the wrench is about the face centre)
    fu = -Mx / (zu - zl)                                                # Fy on the upper half (lower −fu): M_x = −fu·(zu − zl)
    gf = -My / (xf - xb)                                                # Fz on the front half (back −gf): M_y = −gf·(xf − xb)
    hf = Mz / (xf - xb)                                                 # Fy on the front half (back −hf): M_z = hf·(xf − xb)
    on = lambda x, s: (np.abs(x[1] - s * yf) < 0.3)
    loads = []
    for s in (-1, 1):                                                   # s = −1: the right arm (the twin's); s = +1: the left, mirrored in XZ
        k = -s                                                          # mirror: F_y, M_x, M_z change sign (so fu, hf do), F_x, F_z, M_y keep it
        loads += [(lambda x, s=s: on(x, s), (F[0], k * F[1], F[2])),
                  (lambda x, s=s: on(x, s) & (x[2] > cz), (0, k * fu, 0)), (lambda x, s=s: on(x, s) & (x[2] <= cz), (0, -k * fu, 0)),
                  (lambda x, s=s: on(x, s) & (x[0] > cx), (0, k * hf, gf)), (lambda x, s=s: on(x, s) & (x[0] <= cx), (0, -k * hf, -gf))]
    hw = upper.W / 2
    fixed = lambda x: (((np.abs(np.abs(x[0] - xm) - hw) < 0.3) & (np.abs(x[1]) < 15.2)) | ((np.abs(np.abs(x[1]) - hw) < 0.3) & (np.abs(x[0] - xm) < 15.2)))
    return (f"Lift carriage (D20 slot, side walls 8 mm): one arm per side, F ({F[0]:.0f}, {F[1]:.0f}, {F[2]:.0f}) N, "
            f"M ({M[0]:.0f}, {M[1]:.0f}, {M[2]:.0f}) N·m × 1.5 ({pose})", part, 3.0, fixed, loads)


def inline_roll_cases(M=19.0, T=35.1, belt=691.0, weight=28.0):
    """D24 inline roll (PIP_ROLL_DRIVE=inline): the equivalents of cases 6–7. The roll output is carried by the GIM8108-36's
    output bearing (W2, +X) and the rear 61806 on the roll link's Ø30 stub (W1, −X); the arm moment 19 N·m is a couple
    between them, and the roll torque (the drive's 35.1 N·m limit, rated) reacts through the motor's 8 × M4 (PCD 84) into the
    roll-link floor and cup."""
    from .parts import roll as rl
    W1, W2, RY = rl.W1, rl.W2, rl.ROLL_Y
    span = ((W2[0] + W2[1]) - (W1[0] + W1[1])) / 2000
    Fc = M / span
    xf1 = rl.X_MOT[0]                                              # the motor body seats on the cup ring (floor + cup carry the 8 × M4)
    arm_m = ((W2[0] + W2[1]) / 2 - xf1) / 1000                     # output bearing → floor face: the housing carries Fc × arm_m as a moment
    n, rp = rl.MOT36["rear_n"], rl.MOT36["rear_pcd"] / 2
    holes = []
    for k in range(n):
        a = 2 * math.pi * k / n + rl.REAR_ANG0
        y, z = RY + rp * math.cos(a), rp * math.sin(a)
        ft = T / (n * rp / 1000)                                    # tangential (torque about +X): direction (−sin a, cos a) in (y, z)
        fx = Fc * arm_m * (z / 1000) / sum(((rp * math.sin(2 * math.pi * j / n + rl.REAR_ANG0)) / 1000) ** 2 for j in range(n))
        holes.append((lambda x, y=y, z=z: (np.hypot(x[1] - y, x[2] - z) < 2.6) & (x[0] > rl.X_FLOOR[0] - 0.2) & (x[0] < xf1 + 0.2),
                      (-fx, -ft * math.sin(a), ft * math.cos(a) - Fc / n)))
    C = [(f"Roll link (D24 inline): rear 61806 stub + GIM8108-36 floor, arm moment {M:.0f} N·m (couple {Fc:.0f} N) + roll torque {T:.0f} N·m", rl.roll_link_inline(), 2.0,
          lambda x: (np.abs(x[1] - 8) < 0.3) & (np.hypot(x[0], x[2]) < 45),
          [(lambda x: (np.abs(np.hypot(x[1] - RY, x[2]) - rl.BRG_R["d"] / 2) < 0.3) & (x[0] > W1[0] - 0.2) & (x[0] < W1[1] + 0.2), (0, 0, Fc))] + holes)]
    # 7 (inline): clevis + yaw block + rear arm (bonded) held at the output-flange bolt holes and the 61806 seat; yaw belt
    # peak 691 N between the seats + the upper-arm moment 19 N·m as a couple on the yaw 61808 seats (as case 7)
    P1, P2, YC = rl.P1, rl.P2, rl.YAW_C
    Fy = M / ((P1[0] + P1[1] - P2[0] - P2[1]) / 2000)
    inp = lambda x, P: (x[2] > P[0] - 0.2) & (x[2] < P[1] + 0.2)
    xa = rl.X_ARM_F
    na, pa = rl.ADAPT["n"], rl.ADAPT["pcd"] / 2
    out_holes = [(RY + pa * math.cos(2 * math.pi * k / na + math.pi / na), pa * math.sin(2 * math.pi * k / na + math.pi / na)) for k in range(na)]
    fixed = lambda x: (((x[0] > xa[0] - 0.2) & (x[0] < xa[1] + 0.2) & np.any([np.hypot(x[1] - y, x[2] - z) < 2.8 for y, z in out_holes], axis=0))
                       | ((np.hypot(x[1] - RY, x[2]) < rl.BRG_R["D"] / 2 + 0.3) & (x[0] > W1[0] - 0.2) & (x[0] < W1[1] + 0.2)))
    C.append((f"Clevis + yaw block (D24 inline, + rear arm): yaw belt {'peak ' if belt == 691.0 else ''}{belt:.0f} N + arm moment {M:.0f} N·m (couple {Fy:.0f} N)",
              rl.clevis_and_yaw_block() + rl.yaw_plate_bot() + rl.roll_arm_rear(), 2.5, fixed,
              [(lambda x, P=P: (np.hypot(x[0], x[1] - RY) < 26.3) & inp(x, P), (-belt / 2 + sg * Fy, 0, -weight)) for P, sg in ((P1, 1), (P2, -1))]
              + [(lambda x, P=P: (np.hypot(x[0] + YC, x[1] - RY) < 19.4) & inp(x, P), (belt / 2, 0, 0)) for P in (P1, P2)]))
    return C


def main(argv=()):
    """python -m rnd.fea [--only "Roll link,Clevis"]: --only runs the cases whose names start with one of the prefixes and
    merges them into the existing report (existing rows starting with one of the prefixes are replaced). The default roll
    drive (params.ROLL_DRIVE: inline since 2026-11-27) writes fea.md/json; PIP_ROLL_DRIVE=belt writes fea_belt.md/json."""
    import sys
    from .parts import roll as rl
    argv = list(argv) or sys.argv[1:]
    only = argv[argv.index("--only") + 1].split(",") if "--only" in argv else None
    from .params import ROLL_DRIVE
    stem = "fea" if os.environ.get("PIP_ROLL_DRIVE", ROLL_DRIVE) == "inline" else f"fea_{ROLL_DRIVE}"
    key = lambda n: n.split(":")[0]
    res = {}
    if only and (DOCS / f"{stem}.json").exists():
        res = json.loads((DOCS / f"{stem}.json").read_text())
        res = {k: v for k, v in res.items() if not any(k.startswith(o) for o in only)}
    for name, part, h, fixed, loads in cases():
        if only and not any(name.startswith(o) for o in only):
            continue
        mesh = mesh_part(part, h)
        r = solve(mesh, fixed, loads)
        res = {k: v for k, v in res.items() if key(k) != key(name)}
        res[name] = r
        print(name, r, flush=True)
    order = [n for n, *_ in cases()]
    names = sorted(res, key=lambda n: next((i for i, o in enumerate(order) if key(o) == key(n)), 99))
    rows = []
    for name in names:
        r = res[name]
        sf = ALLOW / r["vm99"]
        rows.append(f"| {name} | {r['n']:,} | {r['vm99']:.1f} MPa (max {r['vmax']:.1f}) | {r['dmax']:.2f} mm | {sf:.1f}× {'✅' if sf >= 1.0 else '❌'} |")
    title = "# FEA of critical printed parts (generated by `python -m rnd.fea`)" if stem == "fea" else \
        f"# FEA, {ROLL_DRIVE} roll variant (generated by `PIP_ROLL_DRIVE={ROLL_DRIVE} python -m rnd.fea`)"
    md = [title, "", __doc__.split("\n\n", 1)[1].strip(), "",
          "| Case | Elements | Von Mises (99.5th pct) | Max displacement | Margin vs 12 MPa |", "|---|---|---|---|---|"] + rows
    (DOCS / f"{stem}.md").write_text("\n".join(md))
    (DOCS / f"{stem}.json").write_text(json.dumps({k: res[k] for k in names}))
    print("\n".join(rows))


if __name__ == "__main__":
    main()


def solve2(mesh, fixed, loads, order=2):
    """A8 (loop 40, external review finding 9): P1 or P2 tets, vectorised von Mises at the quadrature points, and the
    rigid-body motion (translation + rotation, least squares) of each loaded region and of the whole part, for the
    compliance chain (rnd/stiffness.py)."""
    from skfem import Basis, ElementTetP1, ElementTetP2, ElementVector, FacetBasis, LinearForm, asm, condense, solve as sk_solve
    from skfem.models.elasticity import lame_parameters, linear_elasticity
    el = ElementTetP2() if order == 2 else ElementTetP1()
    e = ElementVector(el)
    ib = Basis(mesh, e)
    K = asm(linear_elasticity(*lame_parameters(E, NU)), ib)
    f = np.zeros(K.shape[0])
    bnd = mesh.boundary_facets()
    mids = mesh.p[:, mesh.facets[:, bnd]].mean(axis=1)
    regions = []
    for sel, F in loads:
        facets = bnd[sel(mids)]
        if len(facets) == 0:
            raise ValueError("load selection is empty")
        fb = FacetBasis(mesh, e, facets=facets)
        area = asm(LinearForm(lambda v, w: v[0] * 0 + 1.0), FacetBasis(mesh, ElementTetP1(), facets=facets)).sum()
        t = np.asarray(F, float) / area

        @LinearForm
        def trac(v, w):
            return t[0] * v[0] + t[1] * v[1] + t[2] * v[2]
        f += asm(trac, fb)
        regions.append(np.unique(mesh.facets[:, facets]))
    fixed_facets = bnd[fixed(mids)]
    D = ib.get_dofs(facets=fixed_facets).all()
    u = sk_solve(*condense(K, f, D=D))
    lam, mu = lame_parameters(E, NU)
    G = ib.interpolate(u).grad                                       # (3, 3, nel, nqp)
    eps = 0.5 * (G + np.transpose(G, (1, 0, 2, 3)))
    tr = eps[0, 0] + eps[1, 1] + eps[2, 2]
    sig = 2 * mu * eps
    for i in range(3):
        sig[i, i] += lam * tr
    dev = sig.copy()
    for i in range(3):
        dev[i, i] -= (sig[0, 0] + sig[1, 1] + sig[2, 2]) / 3
    vm = np.sqrt(1.5 * np.sum(dev ** 2, axis=(0, 1))).max(axis=1)    # per element: worst quadrature point
    un = np.vstack([u[ib.nodal_dofs[c]] for c in range(3)])           # vertex displacements (3, nverts)

    def rigid(nodes):
        X = mesh.p[:, nodes].T
        c = X.mean(axis=0)
        U = un[:, nodes].T
        A = np.zeros((3 * len(nodes), 6))
        for k, x in enumerate(X - c):
            A[3 * k:3 * k + 3, :3] = np.eye(3)
            A[3 * k:3 * k + 3, 3:] = np.array([[0, x[2], -x[1]], [-x[2], 0, x[0]], [x[1], -x[0], 0]])   # ω × x
        sol = np.linalg.lstsq(A, U.reshape(-1), rcond=None)[0]
        return dict(c=c.tolist(), t=sol[:3].tolist(), w=sol[3:].tolist())
    disp = np.sqrt((un ** 2).sum(axis=0))
    return dict(vm99=float(np.quantile(vm, 0.995)), vmax=float(vm.max()), dmax=float(disp.max()), n=mesh.t.shape[1],
                regions=[rigid(r) for r in regions], order=order)
