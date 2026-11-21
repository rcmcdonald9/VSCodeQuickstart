// Print 1x, upside down (top face on the plate, locating lip up). No supports.
include <../lib.scad>
translate([0, 0, chassis_z_top + lid_t]) rotate([180, 0, 0]) lid();
