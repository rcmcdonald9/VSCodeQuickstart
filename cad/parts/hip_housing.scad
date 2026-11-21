// Print 4x: 2 normal + 2 with MIRROR=true (front-right/rear-left vs front-left/rear-right).
// Lay the big pitch-servo plate flat on the build plate. No supports.
include <../lib.scad>
MIRROR = false;
mirror([MIRROR ? 1 : 0, 0, 0])
    translate([-bx, 0, by + pitch_case_top - servo_flange_top + plate_t]) rotate([90, 0, 0]) hip_housing();
