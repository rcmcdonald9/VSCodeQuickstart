// Print 4x in TPU 95A, flat face down.
include <../lib.scad>
translate([-bx, 0, foot_y + foot_d/2 - 2]) rotate([90, 0, 0]) foot();
