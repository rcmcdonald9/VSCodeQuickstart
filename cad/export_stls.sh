#!/usr/bin/env bash
# Regenerates every printable STL into cad/stl/ (needs OpenSCAD 2021.01+).
set -euo pipefail
cd "$(dirname "$0")"
mkdir -p stl
for f in parts/*.scad; do
  p=$(basename "$f" .scad); echo "-> $p"
  openscad -q -o "stl/$p.stl" "$f"
done
echo "STLs written to cad/stl/"
