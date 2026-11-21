// =====================================================================
//  "Pip" robot dog — shared parameters (all units mm)
//  Every printed part pulls its dimensions from here, so if your servos
//  or horns measure differently, change them HERE and re-export STLs.
// =====================================================================

$fn = 64;

// ---------- Servo: "standard" size 20-35 kg·cm digital servo -----------
// Defaults match DS3218 / DS3225 / most 40x20 mm metal-gear servos.
// MEASURE YOURS WITH CALIPERS before printing the full set.
servo_len        = 40.5;   // long axis of the case
servo_wid        = 20.2;   // short axis of the case
servo_hgt        = 40.5;   // case bottom -> case top face (excl. shaft)
servo_shaft_off  = 10.2;   // shaft centre -> nearest short end of case
servo_flange_len = 54.5;   // tip-to-tip length of the mounting ears
servo_flange_t   = 2.5;    // ear thickness
servo_flange_top = 10.5;   // case top face -> top surface of ears
servo_hole_pitch = 49.5;   // ear hole spacing along long axis
servo_hole_span  = 10.0;   // ear hole spacing across short axis
horn_stack       = 7.0;    // case top face -> outer face of metal disc horn

// Metal disc horn (25T) bolt pattern. Most "25T aluminium disc horns"
// have 4 x M3 holes on a ~14 mm circle — CHECK YOURS.
horn_bolt_r      = 7.0;
horn_bolt_d      = 3.3;    // M3 clearance
horn_hub_clear_d = 9.0;    // lets you reach the centre spline screw
horn_disc_d      = 24.0;

// ---------- Fasteners -------------------------------------------------
m3_clear   = 3.4;
m3_insert  = 4.0;          // hole for M3 brass heat-set insert (OD ~4.6)
m3_insert_depth = 5.5;
fit        = 0.4;          // clearance added around servo cases

// ---------- Body geometry ---------------------------------------------
bx   = 95;      // hip pitch axis X position (front +, rear -)
by   = 39;      // roll axis Y offset from body centre line (ears of L/R roll servos must not collide)
wall = 3;       // general wall thickness
plate_t = 6;    // thickness of the plate that clamps a servo ear

// derived: roll servo case top (faces +X at the front hips)
roll_case_top_x   = bx - 24;                      // see docs/ASSEMBLY.md
chassis_front_x   = roll_case_top_x - (servo_flange_top - plate_t); // = bx-28.5
chassis_half_len  = chassis_front_x;
chassis_half_wid  = by + (servo_flange_len - servo_len) / 2 + servo_shaft_off + wall + 1;
chassis_z_bot     = -16;
chassis_z_top     = 16;

// hip housing
hip_plate_x0   = roll_case_top_x + horn_stack;    // back face of horn plate
hip_plate_t    = 5;
pitch_case_top = 22;                              // outboard offset of pitch case top from roll axis

// leg segments
upper_len   = 100;     // L2: hip pitch axis -> knee axis
lower_len   = 100;     // L3: knee axis -> foot contact centre
leg_w       = 32;      // upper-leg plate width
leg_t       = 6;       // leg plate thickness
foot_d      = 22;      // TPU foot ball diameter

// lateral stack (outboard distances from roll axis, Y)
upper_in    = pitch_case_top + horn_stack;        // inboard face of upper leg plate
upper_out   = upper_in + leg_t;
knee_case_top = upper_in + servo_flange_top;      // knee case top (outboard)
lower_in    = knee_case_top + horn_stack;
lower_out   = lower_in + leg_t;
// L1 — abduction offset used by the kinematics (software/config.yaml)
L1 = (lower_in + lower_out) / 2;                  // = 49.5 with defaults

echo(str("L1 (abduction offset) = ", L1, " mm"));
echo(str("Body length (hip to hip) = ", 2 * bx, " mm"));
echo(str("Chassis = ", 2 * chassis_half_len, " x ", 2 * chassis_half_wid, " mm"));
