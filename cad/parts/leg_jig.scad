// Print 1x. Assembly + calibration fixture for the leg modules. Flat, no supports.
include <../lib.scad>
translate([-bx, 0, -((rim_y - cup_depth) - 5)]) rotate([-90, 0, 0]) leg_jig();
