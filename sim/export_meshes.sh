#!/usr/bin/env bash
# Exports every printed part in the BODY frame (neutral pose) for the simulator.
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p assets
for p in tray deck hip_block upper_link lower_link foot pan_bracket sleeve_cap tilt_hub head servo_cap; do
  printf 'include <../../cad/lib.scad>\n%s();\n' "$p" > "assets/_$p.scad"
  openscad -q -o "assets/$p.stl" "assets/_$p.scad"
  rm "assets/_$p.scad"
  echo "-> assets/$p.stl"
done
# derived dimensions the simulator needs (body frame, mm)
cat > assets/_dims.scad <<'SCAD'
include <../../cad/lib.scad>
echo(str("DIMS {\"bx\":", bx, ",\"foot_y\":", foot_y, ",\"L2\":", L2, ",\"L3\":", L3,
  ",\"rim_y\":", rim_y, ",\"upper_out\":", upper_out, ",\"pan_x\":", pan_x, ",\"pan_rim_z\":", pan_rim_z,
  ",\"tilt_top_y\":", tilt_top_y, ",\"tilt_z\":", tilt_z, ",\"foot_d\":", foot_d,
  ",\"servo_len\":", servo_len, ",\"servo_wid\":", servo_wid, ",\"servo_case_h\":", servo_case_h,
  ",\"servo_shaft_off\":", servo_shaft_off, ",\"tray_z0\":", tray_z0, ",\"tray_z1\":", tray_z1,
  ",\"tray_hx\":", tray_hx, ",\"deck_t\":", deck_t, "}"));
cube(1);
SCAD
openscad -o assets/_dims.stl assets/_dims.scad 2>&1 | sed -n 's/.*DIMS //p' | tr -d '"\\' | sed 's/\([a-zA-Z_0-9]*\):/"\1":/g' > assets/dims.json
rm -f assets/_dims.scad assets/_dims.stl
echo "-> assets/dims.json"; cat assets/dims.json
