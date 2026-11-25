// Print 1x, face down (camera face on the plate). PETG. No supports.
include <../lib.scad>
translate([0, 0, pan_x + hd_x1]) rotate([0, 90, 0]) translate([0, 0, -tilt_z]) head();
