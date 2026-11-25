// Print 1x, horn face down (key up). PETG, 100% infill.
include <../lib.scad>
translate([-pan_x, 0, hub_y0]) rotate([-90, 0, 0]) translate([0, 0, -tilt_z]) tilt_hub();
