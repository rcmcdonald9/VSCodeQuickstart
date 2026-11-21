#!/usr/bin/env bash
# Regenerates every printable STL into cad/stl/. Needs OpenSCAD (2021.01+).
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p stl
for p in chassis lid head upper_leg lower_leg foot hip_housing; do
  echo "-> $p"
  openscad -q -o "stl/$p.stl" "parts/$p.scad"
done
openscad -q -D MIRROR=true -o stl/hip_housing_mirrored.stl parts/hip_housing.scad
echo "STLs written to cad/stl/"
