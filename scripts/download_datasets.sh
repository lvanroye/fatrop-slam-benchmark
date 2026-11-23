#!/usr/bin/env bash
# Download the public pose-graph datasets into datasets/.
# All files are taken from the SE-Sync repository (data/ folder).
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
URL=https://raw.githubusercontent.com/david-m-rosen/SE-Sync/master/data
mkdir -p "$ROOT/datasets"

for f in \
    intel manhattan MIT CSAIL kitti_00 city10000 ais2klinik \
    parking-garage sphere2500 sphere_bignoise_vertex3 cubicle torus3D grid3D rim
do
    out="$ROOT/datasets/$f.g2o"
    [[ -s "$out" ]] || curl -fsSL -o "$out" "$URL/$f.g2o"
    echo "$out"
done
