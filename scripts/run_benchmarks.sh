#!/usr/bin/env bash
# Run every solver on every dataset in datasets/ (or on the files given),
# one process per solver.
#
# Usage: scripts/run_benchmarks.sh [options] [file.g2o ...]
#   --solvers a,b         solvers to run (default: fatrop,ceres,g2o,gtsam)
#   --timeout SECONDS     stop a solver after this long and report a timeout
#   --max-iterations N    override every solver's iteration cap
#   --bin PATH            benchmark binary (default: build/slam_benchmark)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$ROOT/build/slam_benchmark"
SOLVERS="fatrop,ceres,g2o,gtsam"
TIMEOUT=""
MAX_ITERATIONS=""
files=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --solvers) SOLVERS="$2"; shift 2 ;;
        --timeout) TIMEOUT="$2"; shift 2 ;;
        --max-iterations) MAX_ITERATIONS="$2"; shift 2 ;;
        --bin) BIN="$2"; shift 2 ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) files+=("$1"); shift ;;
    esac
done

[[ -x "$BIN" ]] || { echo "build first: cmake -S $ROOT -B $ROOT/build && cmake --build $ROOT/build" >&2; exit 1; }
[[ ${#files[@]} -gt 0 ]] || files=("$ROOT"/datasets/*.g2o)
IFS=, read -ra solvers <<< "$SOLVERS"

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
