"""Pip v4 hype video, step 1: export the STL scenes the Blender shots use (read-only use of the CAD in rnd/parts).

python -m rnd.video.export_scenes OUT_DIR
Writes OUT_DIR/<scene>/<part>.stl (mm) and OUT_DIR/manifest.json {scene: [{name, color, file, lift}]}.
Scenes: robot (default pose), robot_hero (arms raised), arm, roll, hand, kx (GIM cartridge + KX74 socket).
"""
from __future__ import annotations

import json
import sys
from pathlib import Path

from build123d import Pos, export_stl

from ..parts import arm, hand, robot, roll
from ..parts import cartridge_m as cm
from ..parts.kx import KX74, balls, collar, hub, male, pins, retainer, spring

HERO_POSES = {"l": dict(pitch=-95, elbow=-35, wrist=-30), "r": dict(pitch=-95, elbow=-35, wrist=-30)}


def kx_scene():
    """GIM6010 cartridge + KX74 male face, balls, and the socket (hub, pins, springs, retainer, collar), mated."""
    ad_z = cm.FLOOR + cm.MOTOR["h"]
    face = ad_z + cm.NECK + KX74.face_t              # mating plane of the cartridge
    from build123d import Rot
    sock = lambda p: Pos(0, 0, face + KX74.gap) * Rot(180, 0, 0) * p
    return [("can", cm.can(), "white"), ("motor", cm.motor_dummy(), "dark"), ("torque_pin", cm.torque_pin(), "steel"),
            ("adapter", Pos(0, 0, ad_z) * cm.motor_adapter(), "dark"), ("balls", Pos(0, 0, ad_z) * cm.adapter_balls(), "steel"),
            ("pins", sock(pins(KX74)), "steel"), ("hub", sock(hub(KX74)), "white"), ("spring", sock(spring(KX74)), "steel"),
            ("retainer", sock(retainer(KX74)), "white"), ("collar", sock(collar(KX74, True)), "orange")]


def main(out: Path):
    out.mkdir(parents=True, exist_ok=True)
    man = {}

    def dump(scene, parts, lift_names=()):
        d = out / scene
        d.mkdir(exist_ok=True)
        man[scene] = []
        for n, p, c in parts:
            f = d / f"{n}.stl"
            export_stl(p, str(f))
            man[scene].append(dict(name=n, color=c, file=str(f), lift=n in lift_names))
        print(scene, len(parts), "parts", flush=True)

    a = robot.build(0.75)
    lo = {n: p.bounding_box().center().Z for n, p, _ in robot.build(0.55)}
    lift_names = {n for n, p, _ in a if abs(p.bounding_box().center().Z - lo.get(n, 1e9)) > 150}
    dump("robot", a, lift_names)
    dump("robot_hero", robot.build(0.75, HERO_POSES), lift_names)
    dump("arm", arm.assembly(elbow_deg=-35, wrist_deg=-20))
    dump("roll", roll.module())
    dump("hand", hand.assembly(jaw_deg=20))
    dump("kx", kx_scene())
    (out / "manifest.json").write_text(json.dumps(man, indent=1))


if __name__ == "__main__":
    main(Path(sys.argv[1]))
