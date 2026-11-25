// =====================================================================
//  Pip v2 — part library. Parts are modelled in the BODY frame
//  (X forward, Y left, Z up, origin = between the hip axes) at the
//  neutral pose: legs straight down, head level and facing forward.
//  Leg parts are drawn for the front-right corner. All four leg modules
//  are IDENTICAL: right side as drawn, left side rotated 180° about Z.
// =====================================================================
include <params.scad>

// ---------------------------------------------------------------- frames
module frame(o, xs, ys, zs) {
    multmatrix([[xs[0], ys[0], zs[0], o[0]],
                [xs[1], ys[1], zs[1], o[1]],
                [xs[2], ys[2], zs[2], o[2]],
                [0, 0, 0, 1]]) children();
}
// pitch servo (FR): shaft -> -Y (outboard), case extends up (+Z)
module pitch_frame() frame([bx, -rim_y, 0], [0,0,1], [-1,0,0], [0,-1,0]) children();
// knee servo (FR, leg straight): shaft -> -Y, case extends up the upper link
module knee_frame()  frame([bx, -upper_out, -L2], [0,0,1], [-1,0,0], [0,-1,0]) children();
// pan servo: shaft up, case extends backward
module pan_frame()   frame([pan_x, 0, pan_rim_z], [-1,0,0], [0,-1,0], [0,0,1]) children();
// tilt servo: shaft -> -Y, case extends down (stands upright in the sleeve)
module tilt_frame()  frame([pan_x, tilt_top_y, tilt_z], [0,0,-1], [1,0,0], [0,-1,0]) children();

// ------------------------------------------------- servo-frame helpers
module servo_dummy() {
    color("#1d1d1d") translate([-servo_shaft_off, -servo_wid/2, -servo_case_h])
        cube([servo_len, servo_wid, servo_case_h]);
    color("#2a2a2a") translate([0, 0, -servo_case_h - 2.4]) cylinder(d = horn_d, h = 2.4);
    color("#3a3a3a") translate([12, -9, -servo_case_h - servo_bottom]) cube([4, 18, servo_bottom]);
    color("silver") cylinder(d = horn_d, h = horn_t);
    color("silver") translate([0, 0, horn_t]) cylinder(d = 5.4, h = 1.9);
}

module cup_solid() {
    translate([cup_x0, -cup_hw, -cup_depth]) cube([cup_x1 - cup_x0, 2 * cup_hw, cup_depth]);
}

// case pocket + rim lead-in + connector window in the floor + cap pilots
module cup_cut() {
    ix0 = -servo_shaft_off - fit; ix1 = servo_len - servo_shaft_off + fit; iw = servo_wid/2 + fit;
    translate([ix0, -iw, -(servo_case_h + servo_bottom)]) cube([ix1 - ix0, 2*iw, servo_case_h + servo_bottom + 1]);
    hull() {                                    // lead-in chamfer at the rim
        translate([ix0, -iw, -lead_in]) cube([ix1 - ix0, 2*iw, 0.01]);
        translate([ix0 - lead_in, -iw - lead_in, 0]) cube([ix1 - ix0 + 2*lead_in, 2*iw + 2*lead_in, 0.02]);
    }
    translate([7, -11, -cup_depth - 1]) cube([15, 22, cup_floor + 2]);   // connectors + cables
    for (x = cap_screw_x, y = [-cap_screw_y, cap_screw_y])
        translate([x, y, -12]) cylinder(d = m3_pilot, h = 13);
}

// The ONE servo cap: clamps any servo into any cup (10 identical caps).
module servo_cap() {
    difference() {
        translate([cup_x0, -cup_hw, 0]) cube([cup_x1 - cup_x0, 2 * cup_hw, cap_t]);
        translate([0, 0, -1]) cylinder(d = horn_d + 2, h = cap_t + 2);
        for (x = cap_screw_x, y = [-cap_screw_y, cap_screw_y])
            translate([x, y, -1]) cylinder(d = m3_clear, h = cap_t + 2);
    }
}

// horn bolt pattern + centre-screw clearance, axis = local Z, through everything
module horn_cut() {
    cylinder(d = horn_screw_clear_d, h = 400, center = true);
    for (sx = [-1, 1], sy = [-1, 1])
        translate([sx * horn_hole_sq/2, sy * horn_hole_sq/2, 0]) cylinder(d = m3_clear, h = 400, center = true);
}
module on_y_axis(x, z) { translate([x, 0, z]) rotate([90, 0, 0]) children(); }

// Grip lug (rule 7): two flats lug_w apart along local X, datum V-notch on top.
module grip_lug(depth = 8, h = lug_h) {
    difference() {
        translate([-lug_w/2, -depth/2, 0]) cube([lug_w, depth, h]);
        translate([0, 0, h]) rotate([90, 0, 0]) rotate([0, 0, 45])
            cube([datum, datum, depth + 2], center = true);
    }
}

// ------------------------------------------------------------ HIP BLOCK
// Leg-module base. Holds the pitch servo, drops straight down into a
// tray socket (tongues + lead-ins), and is trapped there by the deck.
module hip_block() {
    in_y = -(rim_y - cup_depth);                     // inboard face (-6.5)
    difference() {
        union() {
            pitch_frame() cup_solid();
            for (s = [-1, 1]) hull() {               // locating tongues, chamfered bottom
                translate([bx + s * (cup_hw + tongue_d/2) - tongue_d/2, tongue_y - tongue_w/2, cup_x0 + lead_in])
                    cube([tongue_d, tongue_w, cup_x1 - cup_x0 - lead_in]);
                translate([bx + s * cup_hw - (s < 0 ? 0.01 : 0), tongue_y - tongue_w/2, cup_x0])
                    cube([0.01, tongue_w, 1]);
            }
            translate([bx, tongue_y, tray_z1]) grip_lug();
        }
        pitch_frame() cup_cut();
        // dock contact pocket on the inboard face (pads for the tray's pogo pins)
        translate([bx - 8, in_y - 3, -8]) cube([16, 3.01, 6]);
        // bottom lead-in chamfers along X faces
        for (s = [-1, 1]) translate([bx + s * cup_hw, 0, cup_x0]) rotate([0, 45, 0])
            cube([lead_in * 1.42, 200, lead_in * 1.42], center = true);
    }
}

// ----------------------------------------------------------- UPPER LINK
module upper_link() {
    y0 = -upper_out;
    difference() {
        union() {
            translate([0, y0, 0]) rotate([-90, 0, 0]) linear_extrude(link_t) hull() {
                translate([bx, 0]) circle(r = cup_hw);
                translate([bx - cup_hw, L2 - cup_x0 - 1]) square([2 * cup_hw, 1]);
            }
            knee_frame() cup_solid();
            translate([bx, y0 + link_t/2, cup_hw - 4]) grip_lug(depth = link_t);
        }
        knee_frame() cup_cut();
        on_y_axis(bx, 0) horn_cut();
        translate([bx - 6, y0 - 1, -L2 + cup_x1 + 8]) cube([12, link_t + 2, L2 - cup_x1 - 8 - 22]);
    }
}

// ----------------------------------------------------------- LOWER LINK
module lower_link() {
    y0 = -lower_out; zk = -L2;
    difference() {
        union() {
            translate([0, y0, 0]) rotate([-90, 0, 0]) linear_extrude(link_t) hull() {
                translate([bx, -zk]) circle(r = cup_hw - 2);
                translate([bx, -zk + L3 - 2]) circle(r = 6);
            }
            translate([bx, y0 + link_t/2, zk + 10]) grip_lug(depth = link_t);
        }
        on_y_axis(bx, zk) horn_cut();
        on_y_axis(bx, zk - L3) cylinder(d = m3_pilot, h = 400, center = true);  // foot screw
        translate([bx - 4, y0 - 1, zk - L3 + 22]) cube([8, link_t + 2, L3 - 44]);
    }
}

// TPU foot: slides onto the link tip from outboard, one M3x8 from outboard.
module foot() {
    translate([bx, -foot_y, -L2 - L3]) difference() {
        intersection() {
            sphere(d = foot_d);
            translate([-foot_d, -foot_d/2 + 2, -foot_d]) cube(2 * foot_d);    // flat print face (outboard)
        }
        translate([-6 - fit, -link_t/2 - fit, -4]) cube([12 + 2*fit, foot_d, foot_d]);   // tip slot, open inboard + up
        rotate([90, 0, 0]) cylinder(d = m3_clear, h = foot_d);                          // screw
        translate([0, -link_t/2 - 2, 0]) rotate([90, 0, 0]) cylinder(d = 6, h = foot_d);   // head counterbore
    }
}

// ------------------------------------------------------------------ TRAY
sock_x_hw = sock_hw + tongue_d + sock_wall + 0.5;   // socket block half-width
deck_boss = [for (sx = [-1, 1], sy = [-1, 1]) [sx * 40, sy * (rim_y - tray_wall - 3.5)]];  // bosses overlap the wall
module socket_block() translate([bx - sock_x_hw, -rim_y, tray_z0])
    cube([2 * sock_x_hw, rim_y + sock_in_y + sock_wall, tray_z1 - tray_z0]);
module socket_cut() {
    translate([bx - sock_hw, -rim_y - 1, tray_z0 + tray_floor]) cube([2 * sock_hw, rim_y + 1 + sock_in_y, 200]);
    for (s = [-1, 1]) {                           // tongue grooves + top funnel
        translate([bx + s * sock_hw - (s < 0 ? tongue_d + fit : 0), tongue_y - tongue_w/2 - fit, tray_z0 + tray_floor])
            cube([tongue_d + fit, tongue_w + 2*fit, 200]);
    }
    hull() {                                      // funnel lead-in at the socket mouth
        translate([bx - sock_hw, -rim_y - 1, tray_z1 - lead_in]) cube([2*sock_hw, rim_y + 1 + sock_in_y, 0.01]);
        translate([bx - sock_hw - lead_in, -rim_y - 1, tray_z1]) cube([2*sock_hw + 2*lead_in, rim_y + 1 + sock_in_y + lead_in, 0.02]);
    }
    // pogo-pin block pocket in the inboard wall + wire tunnel into the centre channel
    translate([bx - 8, sock_in_y - 0.01, -8]) cube([16, sock_wall + 0.02, 6]);
}

module tray() {
    difference() {
        union() {
            difference() {
                translate([-tray_hx, -rim_y, tray_z0]) cube([2*tray_hx, 2*rim_y, tray_z1 - tray_z0]);
                translate([-tray_hx + tray_wall, -rim_y + tray_wall, tray_z0 + tray_floor]) cube([2*(tray_hx - tray_wall), 2*(rim_y - tray_wall), 200]);
            }
            for (mx = [0, 1], my = [0, 1]) mirror([mx, 0, 0]) mirror([0, my, 0]) socket_block();
            pan_frame() cup_solid();
            difference() {                         // hollow pedestal under the pan cup
                translate([pan_x - cup_x1, -cup_hw, tray_z0]) cube([cup_x1 - cup_x0, 2*cup_hw, pan_rim_z - cup_depth - tray_z0]);
                translate([pan_x - cup_x1 + 3, -cup_hw + 3, tray_z0 + tray_floor]) cube([cup_x1 - cup_x0 - 6, 2*cup_hw - 6, 200]);
            }
            for (p = deck_boss) translate([p[0], p[1], tray_z0]) cylinder(d = 8, h = tray_z1 - tray_z0);
            for (s = [-1, 1]) translate([s * tray_hx, 0, 0]) rotate([0, s * 90, 0]) rotate([0, 0, 90])
                grip_lug(depth = 10);
        }
        for (mx = [0, 1], my = [0, 1]) mirror([mx, 0, 0]) mirror([0, my, 0]) socket_cut();
        pan_frame() cup_cut();
        translate([pan_x - cup_x1 - 1, -8, tray_z0 + tray_floor]) cube([6, 16, 20]);   // pedestal cable door
        for (p = deck_boss) translate([p[0], p[1], tray_z1 - 10]) cylinder(d = m3_pilot, h = 11);
        for (x = [-55, -20], s = [-1, 1]) translate([x - 11, s * 30 - 1.5, tray_z0 - 1]) cube([22, 3, tray_floor + 2]); // battery strap
    }
}

// ------------------------------------------------------------------ DECK
pi_holes = [for (x = [-108.5, -50.5], y = [-24.5, 24.5]) [x, y]];
module deck() {
    z0 = tray_z1;
    difference() {
        union() {
            translate([-tray_hx, -rim_y, z0]) cube([2*tray_hx, 2*rim_y, deck_t]);
            for (h = pi_holes) translate([h[0], h[1], z0 + deck_t]) {   // Pi 5 drops onto pins
                cylinder(d = 6, h = 5);
                cylinder(d1 = 2.5, d2 = 2.0, h = 5 + 3.5);
            }
            translate([105, 0, z0 + deck_t]) rotate([0, 0, 90]) grip_lug(depth = 8);
        }
        for (mx = [0, 1], my = [0, 1]) mirror([mx, 0, 0]) mirror([0, my, 0])
            translate([bx - lug_w/2 - fit, tongue_y - 4 - fit, z0 - 1]) cube([lug_w + 2*fit, 8 + 2*fit, deck_t + 2]);
        translate([pan_x - cup_x1 - fit, -cup_hw - fit, z0 - 1]) cube([cup_x1 - cup_x0 + 2*fit, 2*cup_hw + 2*fit, deck_t + 2]);
        for (p = deck_boss) translate([p[0], p[1], z0 - 1]) cylinder(d = m3_clear, h = deck_t + 2);
        translate([-70, -14, z0 - 1]) cube([30, 28, deck_t + 2]);      // wires up to the Pi
        translate([pan_x - cup_x1 - 14, -10, z0 - 1]) cube([10, 20, deck_t + 2]);  // head cables
        for (i = [0:4]) translate([-20 + i * 7, -32, z0 - 1]) cube([3, 64, deck_t + 2]);   // vents
    }
}

// ----------------------------------------------------------- PAN BRACKET
sl_hw   = servo_wid/2 + fit + sleeve_wall;               // 18.7
sl_y0   = tilt_top_y - cap_t;                            // thin front wall (shaft side)
sl_y1   = tilt_top_y + servo_case_h + servo_bottom + fit;// floor wall inner face
sl_top  = tilt_z + servo_shaft_off + fit;                // sleeve top
sleeve_screws = [for (sx = [-1, 1], y = [tilt_top_y + 6, sl_y1 - 6]) [pan_x + sx * (sl_hw - sleeve_wall/2), y]];
module pan_bracket() {
    iw = servo_wid/2 + fit;
    difference() {
        union() {
            translate([pan_x - sl_hw, sl_y0, bracket_z0]) cube([2*sl_hw, sl_y1 + cup_floor - sl_y0, link_t]);
            for (s = [-1, 1]) translate([pan_x + s * (iw + sleeve_wall/2) - sleeve_wall/2, sl_y0, bracket_z1])
                cube([sleeve_wall, sl_y1 + cup_floor - sl_y0, sl_top - bracket_z1]);
            difference() {                                   // thin shaft-side wall with U-slot
                translate([pan_x - sl_hw, sl_y0, bracket_z1]) cube([2*sl_hw, cap_t, sl_top - bracket_z1]);
                hull() {
                    translate([pan_x, sl_y0 - 1, tilt_z]) rotate([-90, 0, 0]) cylinder(d = horn_d + 2, h = cap_t + 2);
                    translate([pan_x - (horn_d + 2)/2, sl_y0 - 1, sl_top]) cube([horn_d + 2, cap_t + 2, 1]);
                }
            }
            translate([pan_x - sl_hw, sl_y1, bracket_z1]) cube([2*sl_hw, cup_floor, sl_top - bracket_z1]);  // floor wall
            translate([pan_x, sl_y1 + cup_floor, tilt_z]) rotate([-90, 0, 0]) cylinder(d = 8, h = 6.2);     // idler pin
            translate([pan_x - sl_hw, (sl_y0 + sl_y1)/2, bracket_z0 + link_t/2]) rotate([0, -90, 0]) rotate([0, 0, 90])
                grip_lug(depth = link_t, h = 6);
            for (s = [-1, 1]) translate([pan_x + s * (iw - 1.85) - 1.85, sl_y0, bracket_z1])   // rest ribs
                cube([3.7, sl_y1 - sl_y0, tilt_rest]);
        }
        translate([pan_x, 0, 0]) horn_cut();
        // connector window in the floor wall
        translate([pan_x - 11, sl_y1 - 1, tilt_z - 22]) cube([22, cup_floor + 2, 15]);
        for (p = sleeve_screws) translate([p[0], p[1], sl_top - 10]) cylinder(d = m3_pilot, h = 11);
        // lead-in at the sleeve mouth
        hull() {
            translate([pan_x - iw, tilt_top_y, sl_top - lead_in]) cube([2*iw, sl_y1 - tilt_top_y, 0.01]);
            translate([pan_x - iw - lead_in, tilt_top_y, sl_top]) cube([2*iw + 2*lead_in, sl_y1 - tilt_top_y + lead_in, 0.02]);
        }
    }
}
module sleeve_cap() {
    difference() {
        translate([pan_x - sl_hw, sl_y0, sl_top]) cube([2*sl_hw, sl_y1 + cup_floor - sl_y0, 3]);
        for (p = sleeve_screws) translate([p[0], p[1], sl_top - 1]) cylinder(d = m3_clear, h = 5);
    }
}

// -------------------------------------------------------------- TILT HUB
// Pre-fitted to the tilt horn (shaft-up, in its jig). Its key drives the head.
key_z0 = 8.5; key_z1 = 18; key_hw = 5;
hub_y0 = tilt_top_y - horn_t; hub_y1 = hub_y0 - link_t; key_y1 = hub_y1 - 4;
module tilt_hub() {
    difference() {
        union() {
            translate([pan_x, hub_y0, tilt_z]) rotate([90, 0, 0]) cylinder(d = horn_d + 1.6, h = link_t);
            hull() {
                translate([pan_x - key_hw, key_y1, tilt_z + key_z0]) cube([2*key_hw, hub_y0 - key_y1, 1]);
                translate([pan_x - key_hw, key_y1, tilt_z + key_z1 - 1]) cube([2*key_hw, hub_y0 - key_y1, 1]);
            }
        }
        translate([pan_x, 0, tilt_z]) rotate([90, 0, 0]) horn_cut();
    }
}

// ------------------------------------------------------------------ HEAD
// Drops straight down: key into the right slot, idler pin into the left.
hd_x0 = -14; hd_x1 = 50; hd_z0 = -16; hd_z1 = 26; box_x0 = 30;
hd_yr = hub_y1 - 0.2;            // right plate inner face (key sits inside the plate)
hd_yl = sl_y1 + cup_floor + 0.4; // left plate inner face
hp_t = 4;
module head() {
    X = pan_x; Z = tilt_z;
    yr0 = hd_yr - hp_t; yl1 = hd_yl + hp_t;
    difference() {
        union() {
            translate([X + hd_x0, yr0, Z + hd_z0]) cube([box_x0 + 4 - hd_x0, hp_t, hd_z1 - hd_z0]);          // right plate
            translate([X + hd_x0, hd_yl, Z + hd_z0]) cube([box_x0 + 4 - hd_x0, hp_t, hd_z1 - hd_z0]);          // left plate
            translate([X + hd_x0, yr0, Z + hd_z1]) cube([hd_x1 - hd_x0, yl1 - yr0, 3]);                        // top bridge
            translate([X + box_x0, yr0, Z - 14]) cube([hd_x1 - box_x0, yl1 - yr0, hd_z1 + 14]);               // face box
            for (y = [yr0 + 8, yl1 - 11]) translate([X + 2, y, Z + hd_z1 + 3]) rotate([90, 0, 0])            // ears
                linear_extrude(3, center = true) polygon([[0, 0], [18, 0], [4, 17]]);
            translate([X + hd_x0, (yr0 + yl1)/2, Z + hd_z1 + 1.5]) rotate([0, -90, 0]) rotate([0, 0, 90])
                grip_lug(depth = 3, h = 6);
        }
        // hollow face box, open at the back
        translate([X + box_x0 - 1, yr0 + 2.4, Z - 14 + 2.4]) cube([hd_x1 - box_x0 - 1.4, yl1 - yr0 - 4.8, hd_z1 + 14 - 2.4]);
        // right plate: key slot (open at top) + hub/screw-head clearance
        translate([X - key_hw - fit, yr0 - 1, Z + key_z0 - fit]) cube([2*(key_hw + fit), hp_t + 2, 100]);
        translate([X, yr0 - 1, Z]) rotate([-90, 0, 0]) cylinder(d = horn_d + 4, h = hp_t + 2);
        // left plate: idler-pin slot (open at top)
        hull() {
            translate([X, hd_yl - 1, Z]) rotate([-90, 0, 0]) cylinder(d = 8 + 2*fit, h = hp_t + 2);
            translate([X - 4 - fit, hd_yl - 1, Z + 100]) cube([8 + 2*fit, hp_t + 2, 1]);
        }
        // bridge relief over the slots so key/pin can enter from above
        translate([X - key_hw - fit, yr0 - 1, Z + hd_z1 - 1]) cube([2*(key_hw + fit), hp_t + 1.01, 10]);
        translate([X - 4 - fit, hd_yl - 0.01, Z + hd_z1 - 1]) cube([8 + 2*fit, hp_t + 1.01, 10]);
        // Camera Module 3: 14 mm lens window + 4 x M2 (21 x 12.5 mm)
        cy = (yr0 + yl1) / 2;
        fx = X + hd_x1;
        translate([fx - 4, cy - 7, Z + 1]) cube([6, 14, 14]);
        for (dy = [-10.5, 10.5], dz = [2, 14.5]) translate([fx - 4, cy + dy, Z + dz]) rotate([0, 90, 0]) cylinder(d = 2.2, h = 6);
        translate([fx - 4, cy - 6, Z - 10]) cube([6, 12, 7]);                 // VL53L1X window
        for (dy = [-18, 18]) translate([fx - 4, cy + dy, Z + 12]) rotate([0, 90, 0]) cylinder(d = 5.2, h = 6);  // LED eyes
        for (i = [0:5], j = [0:3]) translate([X + 18 + i * 5, cy - 7.5 + j * 5, Z + hd_z1 - 1]) cylinder(d = 2.6, h = 5); // speaker grille
        translate([X + 46, cy, Z + hd_z1 - 1]) cylinder(d = 2.5, h = 5);    // mic
    }
}

// -------------------------------------------------------------- LEG JIG
// Assembly + calibration fixture. The leg module is built in it straight down
// (outboard side up), and while it sits here the leg is exactly straight:
// pitch = 0, knee = 0 -> `python -m robodog.calibrate --jig FR` reads the zeros.
// Modelled in the body frame around the FR module; printed flat (see parts/).
module leg_jig() {
    in_y = -(rim_y - cup_depth);               // hip block inboard face
    base_t = 5; wall = 3; h = 10;
    zt = cup_x1 + 4.5; zb = -L2 - L3 - 8;
    hx = cup_hw + tongue_d + fit;              // pocket half width incl. tongues
    difference() {
        union() {
            translate([bx - 30, in_y, zb]) cube([60, base_t, zt - zb]);                       // base
            translate([bx - hx - wall, in_y - h, cup_x0 - fit - wall])                        // hip pocket
                cube([2*(hx + wall), h + 1, cup_x1 - cup_x0 + 2*(fit + wall)]);
            translate([bx - cup_hw - fit - wall, -(upper_out - cup_depth) - 10, -L2 + cup_x0 - fit - wall]) // knee pad + pocket
                cube([2*(cup_hw + fit + wall), (upper_out - cup_depth) + 10 + in_y + 1, cup_x1 - cup_x0 + 2*(fit + wall)]);
            fz = 40; fw = 6 + (cup_hw - 2 - 6) * (fz + 7 - 2) / (L3 - 2) + 1;   // link half-width at post top + clearance
            for (s = [-1, 1]) translate([bx + s * (fw + 2) - 2, -lower_out - 1, -L2 - L3 + fz - 7])  // lower-link fence
                cube([4, lower_out + 2 + in_y, 14]);
        }
        translate([bx - hx, in_y - h - 1, cup_x0 - fit]) cube([2*hx, h + 1.01, cup_x1 - cup_x0 + 2*fit]);
        translate([bx - cup_hw - fit, -(upper_out - cup_depth) - 11, -L2 + cup_x0 - fit])
            cube([2*(cup_hw + fit), 11.01, cup_x1 - cup_x0 + 2*fit]);
        for (z = [-60, -150]) translate([bx - 12, in_y - 1, z]) cube([24, base_t + 2, 40]);   // save plastic
    }
}
