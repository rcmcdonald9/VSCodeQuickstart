# Printing on the Bambu Lab P2S

All STLs are in [`cad/stl/`](../cad/stl). Each one is already in its print orientation (flat face down) and needs **no supports**.

## 1. Print a test-fit set first

Servos and horns vary between brands by a few tenths of a millimetre. Before you print the full set, print **one `hip_housing.stl`** and **one `upper_leg.stl`** (about 1.5 h) and check:

- [ ] The servo case slides through the window and the ears sit flat on the plate.
- [ ] The 4 servo-ear holes line up (they take heat-set inserts).
- [ ] The 4 horn holes line up with your aluminium disc horn.

If something is off, change the numbers in [`cad/params.scad`](../cad/params.scad): `servo_len`, `servo_wid`, `servo_hole_pitch`, `horn_bolt_r`, `fit`, and so on. Then regenerate the STLs:

```bash
cd cad && ./export_stls.sh        # needs OpenSCAD (free): https://openscad.org
```

## 2. Full print list

| File | Qty | Material | Notes |
|------|-----|----------|-------|
| `chassis.stl` | 1 | PETG | 133 × 120 × 32 mm, the biggest part |
| `lid.stl` | 1 | PETG | Prints upside down (as exported) |
| `head.stl` | 1 | PETG | Short bridged roof, fine without supports |
| `hip_housing.stl` | **2** | PETG | Front-right + rear-left |
| `hip_housing_mirrored.stl` | **2** | PETG | Front-left + rear-right |
| `upper_leg.stl` | 4 | PETG | |
| `lower_leg.stl` | 4 | PETG | |
| `foot.stl` | 4 | **TPU 95A** | |

Suggested plates (let Bambu Studio's **Arrange** pack them):

1. chassis + lid
2. 4 × upper_leg + head
3. 4 × lower_leg + 4 hip housings (2 normal, 2 mirrored)
4. 4 × foot (TPU)

## 3. Slicer settings (Bambu Studio)

Start from the **Generic PETG** (or Bambu PETG HF) profile, 0.4 mm nozzle, then change:

| Setting | Structural parts | Chassis / lid / head |
|---------|------------------|----------------------|
| Layer height | 0.2 mm | 0.2 mm |
| Wall loops | **4** | 3 |
| Top / bottom shells | 5 / 5 | 4 / 4 |
| Sparse infill | **40 % gyroid** | 20 % gyroid |
| Supports | off | off |
| Brim | auto | auto |

Structural parts are the hip housings, upper legs and lower legs. They carry all the servo torque, so don't skimp on walls.

**Plate:** Textured PEI works well for PETG. On a smooth plate, use glue stick as a release agent: PETG can bond too strongly to bare smooth PEI.

**TPU feet:** Use the Bambu TPU 95A profile. Feed TPU from an **external spool, not through the AMS**. Use 3 walls, 20 % infill, and print slowly (the profile handles it).

The whole set takes very roughly 12–16 hours of printing and about 400 g of PETG. Bambu Studio shows the exact time and weight for each plate.

## 4. After printing

- Press the **M3 heat-set inserts** in with a soldering iron at about 220 °C (PETG). Push them in from the side the servo ear sits on:
  - Chassis: 16 (roll servo ears, inside face of the front/rear walls) + 4 (lid posts, top)
  - Each hip housing: 4 (pitch servo ears, on the face *away* from the protruding servo case)
  - Each upper leg: 4 (knee servo ears, on the flat face)
  - Head: 4 (base)
- Ream the horn holes with a 3.3 mm drill if the screws bind.
