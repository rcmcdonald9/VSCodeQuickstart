// Print 4x (all identical). Flat face down, ribs up. No supports.
include <../lib.scad>
translate([-bx, 0, -by - upper_in]) rotate([-90, 0, 0]) upper_leg();
