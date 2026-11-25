"""Export printable STLs and an exploded render assembly: python -m rnd.parts.export"""
from __future__ import annotations

from pathlib import Path

from build123d import Pos, Rot, export_stl

from . import cartridge_m as cm, cartridge_s as cs
from .kx import KX44, KX74, collar, hub, pins, retainer, spring

OUT = Path(__file__).resolve().parents[1] / "out"


def socket(kx, z, explode, x=0.0, locked=False):
    """Hub group flipped to face down onto a cartridge whose mating plane is at z (exploded upward)."""
    f = lambda p, dz=0: Pos(x, 0, z + kx.gap + explode + dz) * Rot(180, 0, 0) * p
    return {"hub": (f(hub(kx), explode * 0.6), "white"), "pins": (f(pins(kx), explode * 0.6), "steel"),
            "spring": (f(spring(kx), explode * 0.6 + 0), "steel"), "retainer": (f(retainer(kx), explode * 0.6), "white"),
            "collar": (f(collar(kx, locked)), "orange")}


def main():
    OUT.mkdir(exist_ok=True)
    printable = {"kx74_male_adapter_gim6010": cm.motor_adapter(), "kx74_hub": hub(KX74), "kx74_collar": Rot(180, 0, 0) * collar(KX74),
                 "kx74_retainer": retainer(KX74), "cartM_can": cm.can(), "coupon_gim6010_boltcircle": cm.coupon_motor(),
                 "kx44_male_horn_adapter": cs.horn_adapter(), "kx44_hub": hub(KX44), "kx44_collar": Rot(180, 0, 0) * collar(KX44),
                 "kx44_retainer": retainer(KX44), "cartS_sleeve": cs.sleeve()}
    for n, p in printable.items():
        export_stl(p, str(OUT / f"{n}.stl"))
    e = 34
    ad_z = cm.FLOOR + cm.MOTOR["h"]
    asm = {"m_can": (cm.can(), "white"), "m_motor": (cm.motor_dummy(), "dark"), "m_pin": (cm.torque_pin(), "steel"),
           "m_adapter": (Pos(0, 0, ad_z + e * 0.35) * cm.motor_adapter(), "dark"),
           "m_balls": (Pos(0, 0, ad_z + e * 0.35) * cm.adapter_balls(), "steel")}
    asm.update({f"m_{k}": v for k, v in socket(KX74, ad_z + e * 0.35 + cm.NECK + KX74.face_t, e).items()})
    xs = 120.0
    s_ad = cs.servo_top() + cs.SERVO["horn_t"]
    asm.update({"s_sleeve": (Pos(xs, 0, 0) * cs.sleeve(), "white"), "s_servo": (Pos(xs, 0, 0) * cs.servo_dummy(), "dark"),
                "s_pin": (Pos(xs, 0, 0) * cs.torque_pin(), "steel"),
                "s_adapter": (Pos(xs, 0, s_ad + e * 0.3) * cs.horn_adapter(), "dark"),
                "s_balls": (Pos(xs, 0, s_ad + e * 0.3) * cs.adapter_balls(), "steel")})
    asm.update({f"s_{k}": v for k, v in socket(KX44, s_ad + e * 0.3 + cs.NECK + KX44.face_t, e * 0.8, x=xs).items()})
    for n, (p, _) in asm.items():
        export_stl(p, str(OUT / f"asm_{n}.stl"))
    (OUT / "asm_colors.txt").write_text("\n".join(f"asm_{n} {c}" for n, (_, c) in asm.items()))
    for f in OUT.glob("*.stl"):
        if f.stem.startswith("asm_") and f.stem not in {f"asm_{n}" for n in asm}:
            f.unlink()
    print("exported", len(printable), "printable parts and", len(asm), "assembly bodies to", OUT)


if __name__ == "__main__":
    main()
