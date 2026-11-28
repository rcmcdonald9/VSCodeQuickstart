"""Collision hulls of the CAD arm for the MuJoCo twin (one convex piece per STL; MuJoCo collides convex hulls).

Frames match the twin: upper pieces in the upper-arm body frame (origin at the yaw joint, arm along -Z), forearm pieces in
the forearm body frame (origin on the elbow axis). CAD +Y = the arm's outboard side (forearm side link). Right-arm copies
are mirrored in Y (the right arm sits at -Y). Units: mm; the twin scales by 0.001.
"""
from __future__ import annotations

from pathlib import Path

from build123d import Box, Cylinder, Plane, Pos, Rot, export_stl, mirror

from . import arm, cartridge_m as cm, roll
from .kx import KX74

OUT = Path(__file__).resolve().parents[1] / "out" / (("collision" if not roll.INLINE else "collision_inline") + ("_d20" if arm.D20 else ""))   # D24: one hull set per roll drive, D20: per arm-motor set (= twin.COLL)


def arm_sh_x():
    from ..params import BASELINE
    return BASELINE.arm.shoulder_x * 1000


TORSO_HALF_W = 390 / 2                 # rnd/parts/upper.py torso soft_shell width 390 mm


def arm_sh_y():
    from ..params import BASELINE
    return BASELINE.arm.shoulder_y * 1000


def _cap_belt():
    x0, z1, y1 = roll.XF - roll.ZC - roll.CAP_GAP[0], roll.ROLL_C + roll.CAP_GAP[1] - 12, roll.ROLL_Y + roll.CAP_GAP[2]
    return Pos((x0 + 33) / 2, (10 + y1) / 2, (38 + z1) / 2) * Box(33 - x0, y1 - 10, z1 - 38)


def _sh_inline():
    """D24 inline roll, static side (pitch-body frame): GIM8108-36 + floor, the roll-link frame, the spring ring."""
    x0, x1 = roll.X_FLOOR[0], roll.ROLL_SPR_X[1] + 1.6
    return {"rolllink": Pos((x0 + x1) / 2, 13, 6) * Box(x1 - x0, 10, 68),
            "rollcart": Pos(x0, roll.ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(roll.R_MOT, roll.X_MOT[1] - x0, align=arm.MIN),
            "cap": Pos(roll.ROLL_SPR_X[0], roll.ROLL_Y, 0) * Rot(0, 90, 0) * Cylinder(40, x1 - roll.ROLL_SPR_X[0], align=arm.MIN)}


def _roll_inline():
    """D24 inline roll, roll body (origin on the roll axis): yaw block + band, the yaw cartridge + rear arm + fork, the +X arm."""
    x1 = roll.X_PL1 + 8
    zt = roll.P1[1] + 1.5
    zc = roll.ZF + roll.ZC + 2.0                   # D24 bolted yaw cartridge: rear cap top (was 36 with the KX74 can)
    return {"yawblock": Pos((-130 + x1) / 2, 0, (-roll.YAW_DROP + zt) / 2) * Box(x1 + 130, 100, zt + roll.YAW_DROP),
            "yawcart": Pos((-132 + roll.X_ARM_R[1]) / 2, 0, (zt + zc) / 2) * Box(roll.X_ARM_R[1] + 132, 106, zc - zt),
            "xarm": Pos(sum(roll.X_ARM_F) / 2, 0, (zt + roll.R_ARM) / 2) * Box(roll.X_ARM_F[1] - roll.X_ARM_F[0], 2 * roll.R_ARM, roll.R_ARM - zt)}


def pieces():
    h = arm.HOUSING
    cart_len = KX74.gap + KX74.face_t + cm.NECK + cm.FLOOR + arm.MOT["h"]          # D20: arm.MOT = the elbow motor
    y0 = -20.0                                                     # cartridge KX face plane (arm.assembly)
    fy, link_t = arm.FORE_Y, 22.0
    wy = arm.J4310["h"] + 4 + 6
    return {
        "upper": {
            # neck from the housing top to the yaw flange (it follows yaw_drop: the inline roll's bone is 12 mm longer); it
            # was a fixed 100 mm that reached ≈ 40 mm above the yaw joint into the roll body
            "neck": Pos(0, 0, h["z1"] - 2) * Cylinder(36, -6 - (h["z1"] - 2), align=arm.MIN) + Pos(0, 0, -6) * Cylinder(45, 6, align=arm.MIN),
            "housing": Pos(0, 0, (h["z0"] + h["z1"]) / 2) * Box(h["x"], h["y"] + 16, h["z1"] - h["z0"]),
            "elbow_cart": Pos(0, y0 - cart_len / 2, -arm.ELBOW_Z + arm.BELT_C) * Rot(90, 0, 0) * Cylinder(arm.MOT["od"] / 2 + 3.3, cart_len),
        },
        "fore": {
            "link": Pos(0, fy, -arm.FORE_L / 2 + 10) * Box(56, link_t, arm.FORE_L - 2 * 38),
            "wrist": Pos(0, arm.WRIST_Y0 - 3, -arm.FORE_L) * Rot(-90, 0, 0) * Cylinder(35, arm.WRIST_Y1 - arm.WRIST_Y0 + 3, align=arm.MIN),
        },
        # wrist-pitch body (hand frame = CAD hand frame): palm + KX56 collar; hand (roll) body: gripper, offset +40 mm
        "wristb": {
            "palm": Pos(7, 0, -22) * Box(74, 40, 94),
            "wpcollar": Pos(0, 23, 0) * Rot(-90, 0, 0) * Cylinder(43, 19, align=arm.MIN),
        },
        # pitch body ("sh", frame = detach plane at y 0): roll link + roll cartridge
        "sh": _sh_inline() if roll.INLINE else {
            "rolllink": Pos(0, 61, 38) * Box(92, 106, 168),
            "rollcart": Pos(roll.XF - roll.ZC / 2, roll.ROLL_Y, roll.ROLL_C) * Box(roll.ZC, 2 * roll.R_CAN + 6, 2 * roll.R_CAN + 6),   # D24 bolted roll cartridge
            "cap": _cap_belt(),                                                  # looks-pass shoulder cap (roll.shoulder_cap, D24 size)
        },
        # roll body (origin on the roll axis): clevis + yaw block + yaw cartridge
        "roll": _roll_inline() if roll.INLINE else {
            "yawblock": Pos(-38, 0, (-roll.YAW_DROP + 26) / 2) * Box(172, 92, roll.YAW_DROP + 26),
        },
        # torso body: the torso-side pitch modules (module frame shifted so the detach plane is at the pitch body's y = 0)
        "torso": {
            "pitchmod": Pos(arm_sh_x() - 45, min(arm_sh_y(), TORSO_HALF_W) - 67.5, 10) * Box(190, 135, 150),   # outboard face at the torso shell side (y 195), not the detach plane (y 200): the inline yaw block sits 19 mm off the plane
        },
        "hand": {
            "gripbase": Pos(5, 0, 40 - 98) * Box(104, 52, 30),
            "jaws": Pos(-2, 3, 40 - 128) * Box(74, 46, 48),
        },
    }


# ---------------------------------------------------------------------------------------------- hulls from the CAD
# §4 item 9 (2026-11-28): the hand-drawn boxes above lagged the CAD (roll x-arm, yaw block, sh_*, elbow cart 1–2 mm, and they
# left out the ferrule, the roll link's frame above z 28, the yaw hood and its D31 TPU skirt, the elbow-bay dome, the
# forearm fairings and the wrist boot). For the default build (inline roll, GIM6010 arm) each hull is now the convex hull
# of the CAD parts it stands for, covers included (robot.build(covers=True) content, so the skirt is in roll_yawcart when
# PIP_WEDGE_GUARD=skirt), in the twin body frame, at the reference pose (roll = yaw = elbow = wrist = 0). A part that is
# far from convex is clipped into slabs (clip: {axis: (lo, hi)} in the body frame), one hull each. Body frames (twin.py):
# sh = the pitch body = the roll-module frame; roll = roll-module frame less (0, ROLL_Y, 0); upper = arm.assembly frame
# (yaw joint); fore = elbow axis; wristb = wrist-pitch axis; hand = wristb + 40 mm (the wrist-roll body); torso = robot
# frame at the lift height. The belt roll and the D20 sets keep the drawn boxes (pieces()).
HULLS = {
    "sh": {
        "rolllink": [("mod", "roll_link", dict(x=(None, 5.0)))],          # motor cup + ferrule boss + inboard frame
        "rolllink2": [("mod", "roll_link", dict(x=(5.0, None)))],         # front frame + spring bar
        "rollcart": [("mod", "roll_gim_housing", None), ("cov_pitch", "cover_roll_front", None)],
        "cap": [("mod", "roll_spring_ring", None), ("mod", "roll_enc_pcb", None)],
        "ferrule": [("mod", "ferrule_m", None), ("shoulder_sh", "pitch_hub", dict(y=(-5.0, None)))],   # the hub between the shell (y 195) and the detach plane
    },
    "roll": {
        "yawblock": [("mod", n, dict(z=(None, -50.0)) if n == "yaw_block" else None)
                     for n in ("yaw_block", "yaw_band", "yaw_plate_bot", "yaw_flange", "yaw_spring_ring", "yaw_stop_pin", "yaw_shaft", "yaw_enc_pcb")],
        "yawcart": [("mod", "yaw_can", None), ("mod", "yaw_can_cap", None), ("mod", "yaw_tension_ring", None), ("mod", "roll_arm_rear", None),
                    ("cov_roll", "cover_yaw", None), ("cov_roll", "cover_yaw_tpu_skirt", None)],
        "xarm": [("mod", "yaw_block", dict(z=(-50.0, None), x=(40.0, None))), ("mod", "roll_outflange", None), ("mod", "roll_spring_hub", None)],
    },
    "upper": {
        "neck": [("arm_up", "upper_bone", dict(z=(-60.0, -6.0))), ("arm_up", "cf_neck", None)],
        "flange": [("arm_up", "upper_bone", dict(z=(-8.0, None)))],
        "housing": [("arm_up", "upper_bone", dict(z=(None, -55.0))), ("arm_up", "bone_cap", None), ("arm_up", "e_cassette", None),
                    ("arm_up", "e_seat_cap_m", None), ("arm_up", "e_seat_cap_p", None), ("arm_up", "e_belt", None),
                    ("arm_up", "enc_bracket", None), ("arm_up", "enc_pcb", None), ("cov_arm", "cover_collar_door", None)],
        "elbow_cart": [("arm_up", "e_can", None), ("arm_up", "e_can_cap", None), ("arm_up", "e_motor", None), ("cov_arm", "cover_elbow_bay", None)],
    },
    "fore": {
        "hub": [("arm_fore", "forearm", dict(z=(-45.0, None))), ("cov_fore", "cover_elbow_cap", None)],
        "shaft": [("arm_fore", "e_shaft40", None), ("arm_fore", "e_hub_core", None), ("arm_fore", "enc_magnet_holder", None)],
        "link": [("arm_fore", "forearm", dict(z=(-245.0, -35.0))), ("arm_fore", "cf_fore", None),
                 ("cov_fore", "cover_forearm_f", None), ("cov_fore", "cover_forearm_r", None)],
        "wrist": [("arm_fore", "forearm", dict(z=(None, -235.0))), ("arm_fore", "wrist_can", None), ("arm_fore", "wrist_motor", None),
                  ("arm_fore", "wrist_adapter", None), ("cov_fore", "cover_wrist_f", None), ("cov_fore", "cover_wrist_r", None)],
    },
    "wristb": {
        "plate": [("arm_wrist", "hand_palm", dict(y=(None, -17.0)))],
        "palm": [("arm_wrist", "hand_palm", dict(y=(-17.0, 17.0))), ("arm_wrist", "hand_wr_sleeve", None), ("arm_wrist", "hand_wr_cap", None),
                 ("arm_wrist", "hand_wr_brg_U", None), ("arm_wrist", "hand_wr_brg_L", None)],
        "wpcollar": [("arm_wrist", "hand_palm", dict(y=(17.0, None)))],
    },
    "hand": {
        "gripbase": [("arm_hand", n, None) for n in ("hand_grip_base", "hand_grip_servo_sleeve", "hand_grip_servo", "hand_wr_dog",
                                                      "hand_wr_spindle", "hand_wrist_cam", "hand_palm_tof")],
        "jaws": [("arm_hand", n, None) for n in ("hand_jaw_moving", "hand_jaw_fixed", "hand_finray_fixed", "hand_pad_fixed", "hand_finger_hook")],
    },
    "torso": {   # the static pitch module up to the torso shell's side face (y 195); the rest of the torso is twin primitives
        "pitchmod": [("shoulder", n, dict(y=(None, TORSO_HALF_W))) for n in ("bracket", "bracket_wall", "can", "can_cap", "motor",
                                                                          "bracket_wall_spring_ring", "pulleyA", "enc_pcb")],
    },
}


def _verts(p, tol=0.15):
    """Vertices of a tessellation of p (per face where the whole-solid mesher gives up: the hull only needs points)."""
    import numpy as np
    try:
        v, _ = p.tessellate(tol, 0.2)
        return np.array([(q.X, q.Y, q.Z) for q in v])
    except Exception:
        out = []
        for f in p.faces():
            try:
                v, _ = f.tessellate(tol, 0.2)
                out += [(q.X, q.Y, q.Z) for q in v]
            except Exception:
                for e in f.edges():
                    out += [tuple(e.position_at(t)) for t in np.linspace(0, 1, 32)]
        return np.array(out)


def _clip(V, F, clip):
    """Vertices of the mesh (V, F) clipped to the box `clip` ({axis: (lo, hi)}, None = open), the cut polygons included."""
    import numpy as np
    import trimesh
    if not clip:
        return V
    m = trimesh.Trimesh(V, F, process=False)
    for k, (lo, hi) in clip.items():
        a = "xyz".index(k)
        n = np.eye(3)[a]
        if lo is not None:
            m = m.slice_plane(n * lo, n)
        if hi is not None and len(m.vertices):
            m = m.slice_plane(n * hi, -n)
    return np.asarray(m.vertices)


def _mesh(p, tol=0.15):
    import numpy as np
    try:
        v, t = p.tessellate(tol, 0.2)
        return np.array([(q.X, q.Y, q.Z) for q in v]), np.array(t)
    except Exception:
        V, T = [], []
        for f in p.faces():
            try:
                v, t = f.tessellate(tol, 0.2)
            except Exception:
                continue
            T += [[i + len(V) for i in tri] for tri in t]
            V += [(q.X, q.Y, q.Z) for q in v]
        return np.array(V), np.array(T)


def sources():
    """{family: {name: solid in its twin body frame}} for HULLS."""
    from . import covers, shoulder
    sx, sy = arm_sh_x(), arm_sh_y()
    mod = {n: p for n, p, _ in roll.module()}
    A = {n: p for n, p, _ in arm.assembly(elbow_deg=0.0, wrist_deg=0.0)}
    sh = {n: p for n, p, _ in shoulder.module()}
    zf, zw = arm.ELBOW_Z, arm.ELBOW_Z + arm.FORE_L
    to_roll = Pos(0, -roll.ROLL_Y, 0)
    return {
        "mod": {n: (to_roll * p if n in roll.ROLL_OUT or n.startswith("yaw_") else p) for n, p in mod.items()},
        "cov_roll": {n: to_roll * p for n, p, _ in covers.roll_frame()},
        "cov_pitch": {n: p for n, p, _ in covers.pitch_frame()},
        "arm_up": A, "arm_fore": {n: Pos(0, 0, zf) * p for n, p in A.items()},
        "arm_wrist": {n: Pos(0, 0, zw) * p for n, p in A.items()}, "arm_hand": {n: Pos(0, 0, zw + 40.0) * p for n, p in A.items()},
        "cov_arm": {n: p for n, p, _ in covers.arm_frame()}, "cov_fore": {n: p for n, p, _ in covers.forearm_frame()},
        "shoulder": {n: Pos(sx, sy, 0) * Pos(0, -(shoulder.Y_FER + 18), 0) * p for n, p in sh.items()},
        "shoulder_sh": {n: Pos(0, -(shoulder.Y_FER + 18), 0) * p for n, p in sh.items()},
    }


def cad_hulls():
    """{(body, name): (hull points array, [source part names])}: the convex hull of each HULLS group (left arm)."""
    import numpy as np
    import trimesh
    src = sources()
    out = {}
    for body, groups in HULLS.items():
        for name, items in groups.items():
            P, used = [], []
            for fam, part, clip in items:
                p = src[fam].get(part)
                if p is None:                                   # e.g. the skirt with PIP_WEDGE_GUARD=none
                    continue
                if clip:
                    V, F = _mesh(p)
                    V = _clip(V, F, clip)
                else:
                    V = _verts(p)
                if len(V):
                    P.append(V)
                    used.append(part + ("" if not clip else " " + ",".join(f"{k}{lo if lo is not None else ''}…{hi if hi is not None else ''}" for k, (lo, hi) in clip.items())))
            hull = trimesh.convex.convex_hull(np.concatenate(P))
            out[(body, name)] = (hull, used)
    return out


def export():
    OUT.mkdir(parents=True, exist_ok=True)
    files = {}
    if roll.INLINE and not arm.D20:
        import json
        import numpy as np
        import trimesh
        for f in OUT.glob("*.stl"):                            # one set: hulls renamed or merged do not linger
            f.unlink()
        meta = {}
        for (body, n), (hull, used) in cad_hulls().items():
            for side in ("l", "r"):
                h = hull if side == "l" else trimesh.convex.convex_hull(hull.vertices * np.array([1.0, -1.0, 1.0]))
                f = OUT / f"{body}_{n}_{side}.stl"
                h.export(str(f))
                files[(body, n, side)] = f
            b = hull.bounds
            meta[f"{body}_{n}"] = dict(sources=used, volume_cm3=round(hull.volume / 1000, 1),
                                       bounds_mm=[[round(float(v), 1) for v in b[0]], [round(float(v), 1) for v in b[1]]])
        (OUT / "hulls.json").write_text(json.dumps(meta, indent=1))
        return files
    for body, parts in pieces().items():
        for n, p in parts.items():
            for side, q in (("l", p), ("r", mirror(p, Plane.XZ))):
                f = OUT / f"{body}_{n}_{side}.stl"
                export_stl(q, str(f))
                files[(body, n, side)] = f
    return files


if __name__ == "__main__":
    print(len(export()), "collision meshes →", OUT)
