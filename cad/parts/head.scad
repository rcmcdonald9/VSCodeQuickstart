// Print 1x standing on its base. The roof is a short bridge; no supports needed on a P2S.
include <../lib.scad>
translate([-head_x0 - head_size[0]/2, 0, -chassis_z_top - lid_t]) head();
