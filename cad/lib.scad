// =====================================================================
//  Pip robot dog — part library.
//  All parts are modelled in the BODY frame for the RIGHT-FRONT leg
//  (X forward, Y left, Z up, origin = body centre at roll-axis height).
//  Other corners are mirror images. parts/*.scad re-orient for printing.
// =====================================================================
include <params.scad>

// ---------------------------------------------------------------------
// Servo frame: origin = shaft centre on the case top face, shaft = +Z,
// long axis = +X (from the shaft towards the far end of the case).
// ---------------------------------------------------------------------
servo_cx = servo_len / 2 - servo_shaft_off;   // case centre along X

module servo_dummy() {      // for the assembly preview only
    color("#222") translate([-servo_shaft_off, -servo_wid/2, -servo_hgt])
        cube([servo_len, servo_wid, servo_hgt]);
    color("#333") translate([servo_cx - servo_flange_len/2, -servo_wid/2,
                             -servo_flange_top - servo_flange_t])
        cube([servo_flange_len, servo_wid, servo_flange_t]);
    color("silver") cylinder(d = 6, h = horn_stack - 3);
    color("silver") translate([0, 0, horn_stack - 3]) cylinder(d = horn_disc_d, h = 3);
}

// Plate that clamps the servo ears. Occupies z in [-flange_top, -flange_top+plate_t].
module servo_clamp_plate(margin = 6) {
    translate([servo_cx - servo_flange_len/2 - 3, -servo_wid/2 - margin, -servo_flange_top])
        cube([servo_flange_len + 6, servo_wid + 2*margin, plate_t]);
}

// Negative volume: case window + insert holes for the ear screws.
module servo_clamp_cut() {
    translate([-servo_shaft_off - fit/2, -servo_wid/2 - fit/2, -60])
        cube([servo_len + fit, servo_wid + fit, 120]);
    for (dx = [-1, 1], dy = [-1, 1])
        translate([servo_cx + dx*servo_hole_pitch/2, dy*servo_hole_span/2, -servo_flange_top - 0.01])
            cylinder(d = m3_insert, h = plate_t + 0.02);
}

// Horn bolt pattern, axis along +Z, centred on origin, through everything.
module horn_cut(h = 400) {
    translate([0, 0, -h/2]) cylinder(d = horn_hub_clear_d, h = h);
    for (a = [45:90:315]) rotate([0, 0, a])
        translate([horn_bolt_r, 0, -h/2]) cylinder(d = horn_bolt_d, h = h);
}

// ---------------------------------------------------------------------
// Frame transforms (right-front corner)
// ---------------------------------------------------------------------
// roll servo: shaft -> +X, long axis -> +Y (inboard), servo Y -> +Z
module roll_servo_frame() {
    multmatrix([[0,0,1, roll_case_top_x],
                [1,0,0, -by],
                [0,1,0, 0],
                [0,0,0, 1]]) children();
}
// pitch servo: shaft -> -Y (outboard), long axis -> -Z, servo Y -> +X
module pitch_servo_frame() {
    multmatrix([[0,1,0, bx],
                [0,0,-1, -by - pitch_case_top],
                [-1,0,0, 0],
                [0,0,0, 1]]) children();
}
// knee servo (leg straight down): shaft -> -Y, long axis -> +Z, servo Y -> -X
module knee_servo_frame() {
    multmatrix([[0,-1,0, bx],
                [0,0,-1, -by - knee_case_top],
                [1,0,0, -upper_len],
                [0,0,0, 1]]) children();
}
// Places children on a Y-axis through (x, z) - used for horn patterns.
module on_y_axis(x, z) { translate([x, 0, z]) rotate([90, 0, 0]) children(); }

// ---------------------------------------------------------------------
// CHASSIS (open-top tub that holds the 4 roll servos + electronics)
// ---------------------------------------------------------------------
lid_post_xy = [for (sx = [-1, 1], sy = [-1, 1]) [sx * 12, sy * (chassis_half_wid - wall - 4)]];

module chassis() {
    L = chassis_half_len; W = chassis_half_wid;
    difference() {
        union() {
            difference() {
                translate([-L, -W, chassis_z_bot]) cube([2*L, 2*W, chassis_z_top - chassis_z_bot]);
                // cavity: front/rear walls are plate_t thick, sides/floor are wall thick
                translate([-L + plate_t, -W + wall, chassis_z_bot + wall])
                    cube([2*(L - plate_t), 2*(W - wall), 100]);
            }
            // lid posts
            for (p = lid_post_xy) translate([p[0], p[1], chassis_z_bot])
                cylinder(d = 9, h = chassis_z_top - chassis_z_bot);
        }
        // roll servo windows + ear inserts, all four corners
        for (mx = [0, 1], my = [0, 1]) mirror([mx, 0, 0]) mirror([0, my, 0])
            roll_servo_frame() servo_clamp_cut();
        // lid post insert holes
        for (p = lid_post_xy) translate([p[0], p[1], chassis_z_top - m3_insert_depth])
            cylinder(d = m3_insert, h = 10);
        // battery strap slots (battery hangs under the floor)
        for (sx = [-1, 1], sy = [-1, 1]) translate([sx * 25 - 11, sy * 22 - 1.5, chassis_z_bot - 1])
            cube([22, 3, wall + 2]);
        // battery lead / XT60 pass-through
        translate([0, 0, chassis_z_bot - 1]) cylinder(d = 14, h = wall + 2);
        // servo cable exits in the side walls (pitch + knee cables from each leg)
        for (sx = [-1, 1], sy = [-1, 1]) translate([sx * 42, sy * W, 4]) rotate([90, 0, 0])
            cylinder(d = 11, h = 3 * wall, center = true);
        // weight-saving / airflow slots in the floor
        for (i = [-1, 0, 1]) translate([i * 14 - 4, -8, chassis_z_bot - 1]) cube([8, 16, wall + 2]);
    }
}

// ---------------------------------------------------------------------
// LID / ELECTRONICS DECK (Pi 5 on top, head at the front)
// ---------------------------------------------------------------------
lid_t = 3;
pi_holes = [for (dx = [0, 58], dy = [0, 49]) [dx, dy]];
pi_origin = [-50, -24.5];        // Pi 5 mounted near the rear, USB facing back
head_base = [[38, -20], [38, 20], [60, -20], [60, 20]];

module lid() {
    L = chassis_half_len; W = chassis_half_wid;
    difference() {
        union() {
            translate([-L, -W, chassis_z_top]) cube([2*L, 2*W, lid_t]);
            // locating lip that drops into the tub
            translate([-L + plate_t + 0.4, -W + wall + 0.4, chassis_z_top - 3])
                difference() {
                    cube([2*(L - plate_t - 0.4), 2*(W - wall - 0.4), 3]);
                    translate([2, 2, -1]) cube([2*(L - plate_t - 0.4) - 4, 2*(W - wall - 0.4) - 4, 5]);
                }
        }
        for (p = lid_post_xy) translate([p[0], p[1], chassis_z_top - 5]) cylinder(d = m3_clear, h = 20);
        for (p = lid_post_xy) translate([p[0], p[1], chassis_z_top + lid_t - 2]) cylinder(d = 6.5, h = 5);
        // post clearance in the locating lip
        for (p = lid_post_xy) translate([p[0], p[1], chassis_z_top - 4]) cylinder(d = 11, h = 4);
        // Pi 5 mounting holes: use M2.5 x 10 mm brass standoffs + nuts underneath
        for (h = pi_holes) translate([pi_origin[0] + h[0], pi_origin[1] + h[1], chassis_z_top - 5])
            cylinder(d = 2.8, h = 20);
        // wiring slots: under the Pi (power + I2C/I2S to the bay), camera ribbon under the head
        translate([-38, -14, chassis_z_top - 5]) cube([30, 28, 20]);
        translate([38, -12, chassis_z_top - 5]) cube([8, 24, 20]);
        // head screws (from below, into inserts in the head base)
        for (p = head_base) translate([p[0], p[1], chassis_z_top - 5]) cylinder(d = m3_clear, h = 20);
        // vent grid
        for (i = [0:3]) translate([4 + i * 6, -30, chassis_z_top - 1]) cube([3, 60, 10]);
    }
}

// ---------------------------------------------------------------------
// HEAD (Camera Module 3, VL53L1X ToF, speaker, mic)
// ---------------------------------------------------------------------
head_size = [34, 52, 44];
head_x0 = 32;           // rear face of head
module head() {
    hx = head_size[0]; hy = head_size[1]; hz = head_size[2];
    z0 = chassis_z_top + lid_t;
    t = 2.4;
    difference() {
        union() {
            translate([head_x0, -hy/2, z0]) cube([hx, hy, hz]);
            // insert bosses for the base screws
            for (p = head_base) translate([p[0], p[1], z0]) cylinder(d = 8, h = 7);
        }
        // hollow, open at the back
        translate([head_x0 - 1, -hy/2 + t, z0 + t]) cube([hx - t + 1, hy - 2*t, hz - 2*t]);
        // ribbon cable hole in the floor
        translate([head_x0 + 6, -10, z0 - 1]) cube([6, 20, t + 2]);
        // base inserts
        for (p = head_base) translate([p[0], p[1], z0 - 0.01]) cylinder(d = m3_insert, h = m3_insert_depth);
        fx = head_x0 + hx;
        // camera: lens window + 4 x M2 holes (Camera Module 3: 21 x 12.5 mm pattern)
        cam_z = z0 + 28;
        translate([fx - t - 1, -7, cam_z - 7]) cube([t + 2, 14, 14]);
        for (dy = [-10.5, 10.5], dz = [-4, 8.5]) translate([fx - t - 1, dy, cam_z + dz])
            rotate([0, 90, 0]) cylinder(d = 2.2, h = t + 2);
        // ToF window under the camera
        translate([fx - t - 1, -6, z0 + 9]) cube([t + 2, 12, 7]);
        // "eyes" (5 mm LEDs) — optional, purely cosmetic
        for (dy = [-17, 17]) translate([fx - t - 1, dy, z0 + 33]) rotate([0, 90, 0]) cylinder(d = 5.2, h = t + 2);
        // speaker grille on the left cheek (40 mm speaker), mic hole on top
        for (r = [0, 5, 10], a = [0:60:359]) translate([head_x0 + hx/2 + (r == 0 ? 0 : r*cos(a + r*6)),
                                                    hy/2 - t - 1,
                                                    z0 + hz/2 + (r == 0 ? 0 : r*sin(a + r*6))])
            rotate([-90, 0, 0]) cylinder(d = 3, h = t + 2);
        translate([head_x0 + hx/2, 0, z0 + hz - t - 1]) cylinder(d = 2.5, h = t + 2);
    }
}

// ---------------------------------------------------------------------
// HIP HOUSING (bolts to the roll horn, holds the pitch servo)
// ---------------------------------------------------------------------
module hip_housing() {
    y_out = -by - (pitch_case_top - servo_flange_top) - plate_t;  // outboard face of pitch plate
    y_in  = -by + 16;
    cap_z0 = 17.6; cap_t = 4;
    difference() {
        union() {
            pitch_servo_frame() servo_clamp_plate(margin = 6);               // pitch plate
            translate([hip_plate_x0, y_out, -20]) cube([hip_plate_t, y_in - y_out, cap_z0 + cap_t + 20]); // horn plate
            translate([hip_plate_x0, y_out, cap_z0])                       // top cap
                cube([bx + servo_wid/2 + 6 - hip_plate_x0, y_in - y_out, cap_t]);
            // gusset between horn plate and pitch plate (bottom)
            translate([hip_plate_x0, y_out, -40]) cube([hip_plate_t, plate_t, 20]);
        }
        pitch_servo_frame() servo_clamp_cut();
        // roll horn pattern on the horn plate (axis +X through (y=-by, z=0))
        translate([hip_plate_x0, -by, 0]) rotate([0, 90, 0]) horn_cut();
        // cable relief in the top cap
        translate([bx - 6, -by - 2, cap_z0 - 1]) cube([12, 12, cap_t + 2]);
    }
}

// ---------------------------------------------------------------------
// UPPER LEG (bolts to the pitch horn, holds the knee servo)
// ---------------------------------------------------------------------
module upper_leg() {
    y0 = -by - upper_out;
    z_bot = -upper_len - (servo_len - servo_shaft_off) + servo_cx - servo_flange_len/2 - 3; // bottom of ear plate
    difference() {
        union() {
            translate([0, y0, 0]) rotate([-90, 0, 0]) linear_extrude(leg_t) hull() {
                translate([bx, 0]) circle(r = leg_w/2);
                translate([bx - leg_w/2, upper_len + 22]) square([leg_w, 1]);
            }
            // outboard edge ribs (stiffness) along the upper part
            for (sx = [-1, 1]) translate([bx + sx * (leg_w/2 - 1.5) - 1.5, y0 - 4, -upper_len + 42])
                cube([3, 4, upper_len - 52]);
        }
        knee_servo_frame() servo_clamp_cut();
        on_y_axis(bx, 0) horn_cut();
        // lightening slot
        translate([bx - 6, y0 - 5, -upper_len + 46]) cube([12, leg_t + 10, upper_len - 66]);
    }
}

// ---------------------------------------------------------------------
// LOWER LEG (bolts to the knee horn; the TPU foot slips over the tip)
// ---------------------------------------------------------------------
module lower_leg() {
    y0 = -by - lower_out;
    zk = -upper_len;
    difference() {
        union() {
            translate([0, y0, 0]) rotate([-90, 0, 0]) linear_extrude(leg_t) hull() {
                translate([bx, -zk]) circle(r = 15);
                translate([bx, -zk + lower_len - 2]) circle(r = 5);
            }
            // centre rib on the outboard face
            translate([bx - 2, y0 - 4, zk - lower_len + 18]) cube([4, 4, lower_len - 34]);
        }
        on_y_axis(bx, zk) horn_cut();
        // foot cross-bolt hole
        on_y_axis(bx, zk - lower_len + 4) cylinder(d = m3_clear, h = 200, center = true);
    }
}

// TPU foot: a ball with a slot for the leg tip, cross-bolted with M3.
module foot() {
    difference() {
        intersection() {          // flat 2 mm patch on the bottom: prints without supports
            sphere(d = foot_d);
            translate([-foot_d, -foot_d, -foot_d/2 + 2]) cube(2 * foot_d);
        }
        translate([-leg_t/2 - 0.3, -15, -4]) cube([leg_t + 0.6, 30, 20]);  // slot from the top
        translate([-20, 0, 4]) rotate([0, 90, 0]) cylinder(d = m3_clear, h = 40); // cross bolt
        translate([-20, 0, 4]) rotate([0, 90, 0]) cylinder(d = 6.5, h = 10); // head recess
    }
}
