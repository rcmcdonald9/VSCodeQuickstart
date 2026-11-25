// =====================================================================
//  Pip v2 — assembly-first robot dog. Shared parameters (mm).
//  Design rules: docs/ASSEMBLY_FIRST_DESIGN.md §3. Every part is built
//  by straight-down moves; the precision lives in the parts.
//  Keep in sync with software/config.yaml (tests/test_cad_sync.py checks).
// =====================================================================
$fn = 64;

// ---------- Feetech STS3215 bus servo (12 V, 30 kg·cm) ------------------
// Measured from the SO-ARM100 project's STS3215 model. Servo frame used
// below: origin = output axis on the case top face, +Z = shaft, +X = long
// axis from the shaft toward the far end of the case.
servo_len       = 45.4;
servo_wid       = 24.8;
servo_case_h    = 32.0;   // top face -> case bottom
servo_bottom    = 4.0;    // rear idler disc + connector pins below the case
servo_shaft_off = 10.2;   // shaft centre -> near end of the case
horn_d          = 20.0;   // aluminium disc horn
horn_t          = 1.6;
horn_hole_sq    = 9.9;    // 4 tapped M3 holes on a 9.9 mm square
horn_screw_clear_d = 7.0; // centre spline-screw head clearance
servo_mass_g    = 55;     // assumed (not measured)

// ---------- one fastener for everything: M3 x 8 socket head -----------
screw_len   = 8;
m3_clear    = 3.4;
m3_pilot    = 2.6;        // self-tapping an M3 machine screw into PETG
fit         = 0.3;        // clearance around servo cases & docks
lead_in     = 1.5;        // chamfer on every mating edge (capture range)

// ---------- servo cup (the one standard servo holder) ------------------
cup_side    = 3.5;
cup_end     = 8.0;        // end walls carry the cap screws
cup_floor   = 2.5;
cup_depth   = servo_case_h + servo_bottom + cup_floor;     // 38.5
cup_x0      = -servo_shaft_off - fit - cup_end;            // -18.5
cup_x1      = servo_len - servo_shaft_off + fit + cup_end; //  43.5
cup_hw      = servo_wid / 2 + fit + cup_side;              //  16.2
cap_t       = 1.2;        // thinner than the horn so links clear it
cap_screw_x = [cup_x0 + cup_end / 2, cup_x1 - cup_end / 2];
cap_screw_y = 9;

// link plates bolt straight onto the horn: plate + horn = screw length
link_t      = screw_len - horn_t;                          // 6.4

// ---------- grip lug (rule 7): flats 20 mm apart + 3 mm datum notch ----
lug_w = 20; lug_h = 8; datum = 3;

// ---------- body / leg layout (body frame: X fwd, Y left, Z up) -------
bx        = 95;    // hip axis X (front +, rear -)
rim_y     = 45;    // |Y| of the pitch servo top face = tray side face
L2        = 100;   // hip axis -> knee axis
L3        = 100;   // knee axis -> foot ball centre
foot_d    = 22;
// lateral stack, measured outward from the pitch-servo top face
upper_in  = rim_y + horn_t;            // 46.6
upper_out = upper_in + link_t;         // 53.0 = knee servo top face
lower_in  = upper_out + horn_t;        // 54.6
lower_out = lower_in + link_t;         // 61.0
foot_y    = (lower_in + lower_out) / 2;// 57.8  (software: geometry.foot_y)

// tray
tray_wall   = 3;
tray_floor  = 3;
tray_z0     = cup_x0 - tray_floor;     // -21.5 (hip block bottom = floor top)
tray_z1     = cup_x1;                  //  43.5 (hip block top = deck bottom)
sock_wall   = 3;
sock_hw     = cup_hw + fit;            // socket half width in X
tongue_w    = 5; tongue_d = 2.5;
tongue_y    = -(rim_y - cup_depth / 2);// tongue centre (Y, right side)
sock_in_y   = -(rim_y - cup_depth) + fit;   // socket inboard wall inner face
tray_hx     = bx + sock_hw + tongue_d + sock_wall + 0.5;
deck_t      = 3;

// neck: pan servo cup built into the tray, rim just above the deck
pan_x     = 53;
pan_rim_z = tray_z1 + deck_t + 0.1;    // 46.6
bracket_z0 = pan_rim_z + horn_t;       // pan bracket sits on the horn
bracket_z1 = bracket_z0 + link_t;
// tilt servo stands upright in a sleeve on the bracket, shaft -> -Y
tilt_top_y = -14;                      // tilt servo top face (Y)
tilt_rest  = 3.5;                      // servo rests on ribs clear of the pan-horn screw heads
tilt_z     = bracket_z1 + tilt_rest + fit + (servo_len - servo_shaft_off); // tilt axis Z
sleeve_wall = 6;                       // thick X walls take the cap screws

echo(str("foot_y=", foot_y, "  tray=", 2*tray_hx, " x ", 2*rim_y,
         "  tilt axis z=", tilt_z));
