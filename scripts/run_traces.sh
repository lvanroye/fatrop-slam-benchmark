#!/usr/bin/env bash
# Time-to-target runs: record every iterate of every solver (trace mode).
# Per (dataset, solver), one process at a time:
#   probe:   run until grad_inf <= GRAD_TARGET, no more progress, or the time limit;
#            this only gives the iteration count N (the gradient check costs time);
#   measure: rerun capped at N, recording only the time and a copy of each iterate.
# Objective and gradient are evaluated after the solve. Iterates are appended to
# OUT (dataset,solver,iteration,time_s,objective,grad_inf); summarise them with
# scripts/time_to_target.py.
#
# Usage: scripts/run_traces.sh [options] [file.g2o ...]
#   --solvers a,b         solvers to run (default: fatrop,ceres,g2o,gtsam)
#   --grad-target G       probe target (default: 1e-10)
#   --time-limit SECONDS  per run (default: 600)
#   --out FILE            trace file (default: traces.csv)
#   --bin PATH            benchmark binary (default: build/slam_benchmark)
set -uo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BIN="$ROOT/build/slam_benchmark"
SOLVERS="fatrop,ceres,g2o,gtsam"
GRAD_TARGET=1e-10
TIME_LIMIT=600
OUT=traces.csv
files=()

while [[ $# -gt 0 ]]; do
    case "$1" in
        --solvers) SOLVERS="$2"; shift 2 ;;
        --grad-target) GRAD_TARGET="$2"; shift 2 ;;
        --time-limit) TIME_LIMIT="$2"; shift 2 ;;
        --out) OUT="$2"; shift 2 ;;
        --bin) BIN="$2"; shift 2 ;;
        -h|--help) sed -n '2,18p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) files+=("$1"); shift ;;
    esac
done

[[ -x "$BIN" ]] || { echo "build first: cmake -S $ROOT -B $ROOT/build && cmake --build $ROOT/build" >&2; exit 1; }
[[ ${#files[@]} -gt 0 ]] || files=("$ROOT"/datasets/*.g2o)
IFS=, read -ra solvers <<< "$SOLVERS"
KILL_AFTER=$(( ${TIME_LIMIT%.*} + 100 ))

for f in "${files[@]}"; do
    for s in "${solvers[@]}"; do
        probe=$(timeout "$KILL_AFTER" "$BIN" "$f" --solvers "$s" --trace /dev/null \
                --grad-target "$GRAD_TARGET" --time-limit "$TIME_LIMIT" | tail -1)
        n=$(awk '{print $2}' <<< "$probe")
        if ! [[ "$n" =~ ^[0-9]+$ ]]; then
            printf '%-28s %-16s probe failed\n' "$(basename "$f")" "$s"
            continue
        fi
        meas=$(timeout "$KILL_AFTER" "$BIN" "$f" --solvers "$s" --trace "$OUT" \
               --max-iterations "$n" --time-limit "$TIME_LIMIT" | tail -1)
        printf '%-28s %s\n' "$(basename "$f")" "$meas"
    done
done
