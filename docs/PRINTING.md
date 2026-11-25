# Printing on the Bambu Lab P2S (v2)

All STLs are in [`cad/stl/`](../cad/stl). Each is already in its print orientation and needs **no supports**. There are **no heat-set inserts**: every screw is an M3 × 8 that cuts its own thread in a 2.6 mm hole.

## 1. Test-fit first (about 1.5 h)

Print **one `hip_block.stl` + one `servo_cap.stl`** and check them against a real STS3215:

- [ ] The servo drops into the cup and sits flat, with no force and less than about 0.5 mm of wobble.
- [ ] The cap screws down over the case and the horn turns freely through the cap's hole.
- [ ] The horn's 4 holes line up with a link's holes. Print `lower_link.stl` too if you want to check this (another 30 min).

If something's off, change `fit`, `servo_len`/`servo_wid` or `horn_hole_sq` in [`cad/params.scad`](../cad/params.scad), then run `cad/export_stls.sh`.

## 2. Print list

Times and weights come from slicing with P2S-like speeds. Bambu Studio will report its own.

| File | Qty | Time each | g each | Material |
|---|---|---|---|---|
| `tray.stl` | 1 | 8 h 28 m | 303 | PETG |
| `deck.stl` | 1 | 1 h 25 m | 46 | PETG |
| `hip_block.stl` | 4 | 1 h 13 m | 40 | PETG |
| `upper_link.stl` | 4 | 1 h 32 m | 49 | PETG |
| `lower_link.stl` | 4 | 27 m | 12 | PETG |
| `servo_cap.stl` | 10 | 5 m | 2.5 | PETG |
| `pan_bracket.stl` | 1 | 1 h 21 m | 38 | PETG |
| `sleeve_cap.stl` | 1 | 8 m | 4 | PETG |
| `tilt_hub.stl` | 1 | 12 m | 3.5 | PETG, 100 % infill |
| `head.stl` | 1 | 1 h 6 m | 37 | PETG |
| `foot.stl` | 4 | 13 m | 3 | **TPU 95A** |
| **Robot total** | | **≈ 27 h** | **≈ 863 g PETG** | |
| `leg_jig.stl` (tool, print once) | 1 | 2 h 7 m | 67 | PETG |

**Plates** (let Bambu Studio's Arrange pack them):

1. tray (overnight)
2. 4 × upper_link + head
3. 4 × hip_block + 4 × lower_link + pan_bracket
4. deck + 10 × servo_cap + sleeve_cap + tilt_hub
5. 4 × foot (TPU, external spool, not through the AMS)
6. leg_jig

**Iteration tip:** the tray is 31 % of the print time. When you're iterating on legs or the head, you only reprint those parts. Every module is independent, which is the point of the design.

## 3. Slicer settings (Bambu Studio, 0.4 mm nozzle, start from Generic or Bambu PETG HF)

| | Hip block, links, pan bracket | Everything else |
|---|---|---|
| Layer height | 0.2 mm | 0.2 mm |
| Wall loops | **4** | 3 |
| Top / bottom shells | 5 / 5 | 4 / 4 |
| Sparse infill | **40 % gyroid** | 20 % gyroid |
| Supports | off | off |

The **tilt hub** carries the head's torque through a small key, so print it at 100 % infill (rectilinear).

**Plate:** textured PEI works best for PETG. On a smooth plate, use glue stick. **TPU feet:** Bambu TPU 95A profile, 3 walls, 20 % infill.

## 4. After printing

- **No inserts.** The first time you drive each M3 × 8 into a printed pilot hole it cuts a thread. Drive it slowly and don't over-tighten.
- Check that each part's **grip lug** (the small block with a V-notch) and the **lead-in chamfers** came out clean. They're what make assembly forgiving.
