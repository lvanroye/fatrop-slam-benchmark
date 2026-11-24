#!/usr/bin/env bash
# Run every solver on every dataset in datasets/ (or on the files given),
# one process per solver.
# Usage: scripts/run_benchmarks.sh [file.g2o ...]
# Env: SOLVERS (default: fatrop,ceres,g2o,gtsam), MAX_ITERATIONS, TIMEOUT (seconds per run).
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="${BIN:-$ROOT/build/slam_benchmark}"
[[ -x "$BIN" ]] || { echo "build first: cmake -S $ROOT -B $ROOT/build && cmake --build $ROOT/build" >&2; exit 1; }

files=("$@")
[[ ${#files[@]} -gt 0 ]] || files=("$ROOT"/datasets/*.g2o)
IFS=, read -ra solvers <<< "${SOLVERS:-fatrop,ceres,g2o,gtsam}"

for f in "${files[@]}"; do
    "$BIN" "$f" --solvers none | head -3
    for s in "${solvers[@]}"; do
        out=$(${TIMEOUT:+timeout "$TIMEOUT"} "$BIN" "$f" --solvers "$s" \
              ${MAX_ITERATIONS:+--max-iterations "$MAX_ITERATIONS"})
        if [[ $? -eq 124 ]]; then
            printf '%-8s timeout (>%s s)\n' "$s" "$TIMEOUT"
        else
            tail -n +4 <<< "$out"
        fi
    done
    echo
done
