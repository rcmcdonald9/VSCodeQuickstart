// Full-robot preview. Not for printing — open in OpenSCAD to check fit
// and to see how the parts go together. Joint angles match software/
// (roll, hip pitch, knee) in degrees; + pitch swings the knee backward.
include <lib.scad>

roll  = 0;
pitch = 30;
knee  = -60;
show_servos = true;

module leg_assembly(r, p, k) {
    translate([0, -by, 0]) rotate([r, 0, 0]) translate([0, by, 0]) {
        color("#4c8bf5") hip_housing();
        if (show_servos) pitch_servo_frame() servo_dummy();
        translate([bx, 0, 0]) rotate([0, p, 0]) translate([-bx, 0, 0]) {
            color("#f5a623") upper_leg();
            if (show_servos) knee_servo_frame() servo_dummy();
            translate([bx, 0, -upper_len]) rotate([0, k, 0]) translate([-bx, 0, upper_len]) {
                color("#f5a623") lower_leg();
                color("#333") translate([bx, -by - L1, -upper_len - lower_len]) rotate([0, 0, 90]) foot();
            }
        }
    }
}

color("#e8e8e8") chassis();
color("#cfcfcf") lid();
color("#4c8bf5") head();
if (show_servos) for (mx = [0, 1], my = [0, 1]) mirror([mx, 0, 0]) mirror([0, my, 0]) roll_servo_frame() servo_dummy();

// front legs: knee points backward with +pitch; rear legs are mirrored in X,
// so they use -pitch / -knee to also point their knees backward.
for (my = [0, 1]) mirror([0, my, 0]) {
    leg_assembly(roll, pitch, knee);
    mirror([1, 0, 0]) leg_assembly(roll, -pitch, -knee);
}
