// Print 4x (all identical). Flat face down, rib up. No supports.
include <../lib.scad>
translate([-bx, 0, -by - lower_in]) rotate([-90, 0, 0]) lower_leg();
