// Print 4x (all identical), inboard face down. PETG, 4 walls, 40% infill. No supports.
include <../lib.scad>
translate([-bx, 0, -(rim_y - cup_depth)]) rotate([-90, 0, 0]) hip_block();
