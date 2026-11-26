"""Build design-review packets for the owner's own reviews (Fable, Astra, ...). Sends nothing anywhere.
python -m rnd.review_packet

Writes to docs/review/:
- PROMPT_review2.md            the reviewer prompt (paste first)
- packet_m1_arm.md             focused packet for the buy decision: the bench arm (≈ 150–250 kB)
- packet_full.md               the whole robot (≈ 400 kB; split it if the reviewer's context is smaller)
- images: docs/rnd/img/robot_v4.png (attach if the reviewer takes images)
"""
from __future__ import annotations

from pathlib import Path

R = Path(__file__).resolve().parent.parent
OUT = R / "docs" / "review"

INTRO = ("# Pip v4: design-review packet #2\n\n"
         "A general-purpose home robot: omni base, a lift on a mast, two 7-DOF arms (6 joints + gripper), a head with a face screen. "
         "3D-printed (PETG + ASA near motors) on a Bambu P2S (256 mm bed) around off-the-shelf actuators: SteadyWin GIM6010-8 (24 V), "
         "Damiao DM-J4310-2EC V1.1, Feetech STS3215 12 V. Everything is generated from rnd/params.py: build123d CAD, MuJoCo twins, "
         "FEA (scikit-fem, P1 and P2 tets), Monte Carlo checks. Numbers marked ESTIMATE are unmeasured.\n\n"
         "**Context:** review #1 (another model, 2026-11-26) found 18 issues; docs/review/triage_2026-11-26.md lists them and RND_PLAN §4 "
         "shows what loop 40 did about each. The owner is about to buy the **M1 one-arm bench kit** (docs/M1_BENCH_ARM.md) and wants an "
         "independent check first.\n")

M1_DOCS = ["docs/M1_READINESS.md", "docs/M1_BENCH_ARM.md", "docs/rnd/SCORECARD.md", "docs/RND_PLAN.md", "docs/review/triage_2026-11-26.md",
           "docs/rnd/spring.md", "docs/rnd/belts.md", "docs/rnd/kx.md", "docs/rnd/kx_mc.md", "docs/rnd/thermal.md", "docs/rnd/stiffness.md",
           "docs/rnd/fea.md", "docs/rnd/dexterity.md", "docs/rnd/swept.md", "docs/rnd/spacing.md", "docs/rnd/assembly_seq.md",
           "docs/rnd/integration.md", "docs/rnd/harness.md", "docs/rnd/balancer.md", "docs/rnd/electrical.md", "docs/rnd/accuracy.md",
           "docs/SOFTWARE_PLAN.md", "docs/sw/s1_tests.md", "docs/sw/train_bc.md", "docs/COUPONS.md"]
M1_CODE = ["rnd/params.py", "rnd/specs/actuators.yaml", "rnd/parts/roll.py", "rnd/parts/arm.py", "rnd/parts/hand.py", "rnd/parts/kx.py",
           "rnd/parts/belt_stage.py", "rnd/parts/shoulder.py", "rnd/parts/cartridge_m.py", "rnd/spring.py", "rnd/stiffness.py",
           "rnd/balancer.py", "sw/safety.py", "sw/drivers/mit.py"]
FULL_EXTRA_DOCS = ["docs/rnd/BOM_v4.md", "docs/rnd/tip.md", "docs/rnd/mc.md", "docs/rnd/threshold_mc.md", "docs/rnd/dock_sim.md", "docs/rnd/twin.md",
                   "docs/rnd/budget.md", "docs/BUILD_v4.md", "docs/rnd/print_plan.md", "docs/rnd/assembly.md"]

PROMPT = """You are a senior mechanical / mechatronics reviewer (robot arms, mobile manipulators, FDM-printed structures, motor drives).
The packet that follows describes a 3D-printed home robot. The owner is about to spend ≈ $1.5k on the one-arm bench kit (M1) and wants to know
whether to buy now, what to change first, and what could still make the purchase wrong.

Review critically and independently. Don't restate the packet; find what is wrong, missing, risky or over-optimistic. Check the loop-40 fixes
for review #1 (triage file + RND_PLAN §4) and say which ones you think don't actually solve the problem.

Please cover:
1. **Buy decision for M1:** is each line of docs/M1_BENCH_ARM.md the right part (actuator variants, gear ratios, bearings, belts, D13 pulleys and
   couplings, regen clamp, CAN adapters, bench supply)? Anything missing that would stall the bench bring-up?
2. **Actuator sizing:** whole-workspace torque margins (spring.md), thermal (thermal.md, printed-can limit), belt ratings (belts.md).
3. **Structure and stiffness:** the compliance chain (stiffness.md), FEA method and allowables (fea.md: P1 vs P2), the KX couplings (kx.md, kx_mc.md).
4. **Assemblability and spacing:** assembly_seq.md, spacing.md, swept.md, integration.md.
5. **Cable routing:** harness.md (clock-spring cassettes, D14 ribbon).
6. **Electrical safety on the bench:** electrical.md (category-0 stop, regen clamp, pre-charge, watchdog), sw/safety.py.
7. **Software plan:** SOFTWARE_PLAN.md, the synthetic-data and pipeline dry run.

For each finding, as a numbered list: title; severity (critical / major / minor); evidence (file, section, numbers); why it matters
physically; a concrete fix or test; your confidence (0–100 %). Then: a one-paragraph verdict on buying M1 now (yes / yes with changes / not yet),
the 5 assumptions most likely to be wrong, and the 5 strongest aspects. Be specific and quantitative."""


def _cat(files, code):
    parts = []
    for f in files:
        p = R / f
        if p.exists():
            parts += [f"\n\n=== FILE: {f} ===\n", p.read_text()]
    for f in code:
        p = R / f
        if p.exists():
            lang = "yaml" if f.endswith(".yaml") else "python"
            parts += [f"\n\n=== FILE: {f} ===\n```{lang}\n", p.read_text(), "\n```"]
    return "".join(parts)


def main():
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "PROMPT_review2.md").write_text(PROMPT + "\n")
    m1 = INTRO + _cat(M1_DOCS, M1_CODE)
    (OUT / "packet_m1_arm.md").write_text(m1)
    code_all = ["rnd/params.py", "rnd/specs/actuators.yaml"] + sorted(str(p.relative_to(R)) for p in (R / "rnd/parts").glob("*.py"))
    full = INTRO + _cat(M1_DOCS + FULL_EXTRA_DOCS, code_all + ["rnd/spring.py", "rnd/stiffness.py", "rnd/balancer.py", "sw/safety.py"])
    (OUT / "packet_full.md").write_text(full)
    print(f"packet_m1_arm.md {len(m1) / 1000:.0f} kB, packet_full.md {len(full) / 1000:.0f} kB, prompt in PROMPT_review2.md")


if __name__ == "__main__":
    main()
