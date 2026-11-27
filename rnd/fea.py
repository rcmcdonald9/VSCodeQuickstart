"""Finite-element checks of the critical printed parts (linear elastic, tetrahedral; gmsh + scikit-fem).  python -m rnd.fea

Material: printed PETG, E 1.8 GPa, ν 0.38. Allowables:
- 25 MPa in-layer (50 MPa × 0.5 safety)
- **12 MPa across layers**: layer adhesion ×0.5 then safety ×2 (ESTIMATE). Peak von Mises is compared with this
  conservative value, because the stress direction vs the layers isn't resolved here.

Each case lists its loads (worst static ×1.5 dynamic, or actuator peak for belts). Peak stress is reported at the
99.5th percentile of element stresses, to drop single-element singularities at sharp corners.
"""
from __future__ import annotations

import json
import math
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
    # 6. Roll link: bolted to the arm-side ferrule; roll belt peak 768 N between the seats (half per wall) + the arm moment
    #    19 N·m as a couple on the two 61808 seats (walls 25.4 mm apart)
    from .parts import roll as rl
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
    # 8. Standard torque hanger (wheel version): GIM peak 11 N·m on the Ø6 pin at r 30 → 367 N on the fork wall
    from .parts import base as bs, cartridge_m as cm
    reach = (bs.Z1 - bs.DECK_T) - bs.Z_AX - cm.TORQUE_PIN["r"]
    C.append(("Torque hanger (wheel): GIM peak 11 N·m → 367 N on the fork", cm.torque_hanger(reach, slot_in=4.0), 1.5,
              lambda x, r=reach: np.abs(x[0] - r) < 0.3,
              [(lambda x: (np.abs(x[1] - 3.2) < 0.3) & (x[0] < 10) & (x[2] > 2) & (x[2] < 9), (0, 367.0, 0))]))
    return C


def main():
    rows, res = [], {}
    for name, part, h, fixed, loads in cases():
        mesh = mesh_part(part, h)
        r = solve(mesh, fixed, loads)
        res[name] = r
        sf = ALLOW / r["vm99"]
        rows.append(f"| {name} | {r['n']:,} | {r['vm99']:.1f} MPa (max {r['vmax']:.1f}) | {r['dmax']:.2f} mm | {sf:.1f}× {'✅' if sf >= 1.0 else '❌'} |")
        print(rows[-1], flush=True)
    md = ["# FEA of critical printed parts (generated by `python -m rnd.fea`)", "", __doc__.split("\n\n", 1)[1].strip(), "",
          "| Case | Elements | Von Mises (99.5th pct) | Max displacement | Margin vs 12 MPa |", "|---|---|---|---|---|"] + rows
    (DOCS / "fea.md").write_text("\n".join(md))
    (DOCS / "fea.json").write_text(json.dumps({k: v for k, v in res.items()}))


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
