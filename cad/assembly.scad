// Pip v2 full-robot preview (not for printing). Joint angles in degrees use
// the software's conventions: +pitch swings the knee backward, knee < 0 bends
// it backward, +tilt = nose down, +pan = look left.
include <lib.scad>

pitch = 30; knee = -60; pan = 0; tilt = 0;
show_servos = true;
exploded = 0;          // e.g. 40 to see the straight-down assembly order

module leg_module(p, k) {
    translate([0, 0, exploded * 0.0]) color("#4c8bf5") hip_block();
    if (show_servos) pitch_frame() servo_dummy();
    color("#9ab8f0") pitch_frame() translate([0, 0, exploded * 0.3]) servo_cap();
    translate([bx, 0, 0]) rotate([0, p, 0]) translate([-bx, -exploded * 0.6, 0]) {
        color("#f5a623") upper_link();
        if (show_servos) knee_frame() servo_dummy();
        color("#f8c67a") knee_frame() servo_cap();
        translate([bx, 0, -L2]) rotate([0, k, 0]) translate([-bx, -exploded * 0.3, L2]) {
            color("#f5a623") lower_link();
            color("#333") foot();
        }
    }
}

module neck_and_head(pn, tl) {
    if (show_servos) pan_frame() servo_dummy();
    color("#9ab8f0") pan_frame() servo_cap();
    translate([pan_x, 0, 0]) rotate([0, 0, pn]) translate([-pan_x, 0, exploded * 0.5]) {
        color("#4c8bf5") pan_bracket();
        color("#9ab8f0") translate([0, 0, exploded * 0.6]) sleeve_cap();
        if (show_servos) translate([0, 0, exploded * 0.3]) tilt_frame() servo_dummy();
        translate([pan_x, 0, tilt_z]) rotate([0, tl, 0]) translate([-pan_x, 0, -tilt_z]) {
            color("#cfd8e6") translate([0, 0, exploded * 0.3]) tilt_hub();
            color("#4c8bf5") translate([0, 0, exploded * 1.2]) head();
        }
    }
}

color("#e8e8e8") tray();
color("#d0d0d0") translate([0, 0, exploded * 1.0]) deck();
// identical modules: right side as drawn, left side rotated 180° about Z
for (sx = [1, -1]) {
    translate([sx > 0 ? 0 : -2 * bx, 0, 0]) leg_module(pitch, knee);                    // right legs
    rotate([0, 0, 180]) translate([sx > 0 ? 0 : -2 * bx, 0, 0]) leg_module(-pitch, -knee); // left legs
}
neck_and_head(pan, tilt);
