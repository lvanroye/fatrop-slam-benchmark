#!/usr/bin/env bash
# Run every solver on every dataset in datasets/ (or on the files given).
# Usage: scripts/run_benchmarks.sh [file.g2o ...]
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${BIN:-$ROOT/build/slam_benchmark}"
[[ -x "$BIN" ]] || { echo "build first: cmake -S $ROOT -B $ROOT/build && cmake --build $ROOT/build" >&2; exit 1; }

files=("$@")
[[ ${#files[@]} -gt 0 ]] || files=("$ROOT"/datasets/*.g2o)

for f in "${files[@]}"; do
    "$BIN" "$f" ${MAX_ITERATIONS:+--max-iterations "$MAX_ITERATIONS"} ${SOLVERS:+--solvers "$SOLVERS"}
    echo
done
