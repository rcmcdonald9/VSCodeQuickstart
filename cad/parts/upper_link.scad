// Print 4x (identical), outboard face down (servo cup opens onto the plate). PETG, 4 walls, 40%.
include <../lib.scad>
translate([-bx, 0, upper_out]) rotate([90, 0, 0]) upper_link();
