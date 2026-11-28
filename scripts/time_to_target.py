#!/usr/bin/env python3
"""Summarise a trace file from scripts/run_traces.sh.

Usage: scripts/time_to_target.py [traces.csv] [summary.csv]

Per dataset and solver: time and iteration of the first iterate with
grad_inf <= 1e-4, 1e-6, 1e-8, and of the first within 1e-6 (relative) of f*,
the lowest objective any solver reached on that dataset; plus the final
objective, its gap to f* and the final gradient. Empty: never reached.
"""
import csv
import sys
from collections import defaultdict

GRAD = [1e-4, 1e-6, 1e-8]
FREL = 1e-6
src = sys.argv[1] if len(sys.argv) > 1 else "traces.csv"
dst = sys.argv[2] if len(sys.argv) > 2 else "summary.csv"

tr = defaultdict(list)
for d, s, it, t, f, g in csv.reader(open(src)):
    tr[d, s].append((int(it), float(t), float(f), float(g)))
datasets = list(dict.fromkeys(d for d, _ in tr))
solvers = list(dict.fromkeys(s for _, s in tr))


def first(pts, ok):
    return next(((t, it) for it, t, f, g in pts if ok(f, g)), None)


with open(dst, "w", newline="") as fh:
    w = csv.writer(fh)
    w.writerow(["dataset", "solver"]
               + [f"{k}_{x}" for k in [f"grad_{g:.0e}" for g in GRAD] + [f"f_within_{FREL:.0e}"]
                  for x in ["time_s", "iter"]]
               + ["f_star", "final_objective", "final_rel_gap", "final_grad", "final_time_s"])
    for d in datasets:
        fstar = min(p[2] for s in solvers for p in tr.get((d, s), []))
        for s in solvers:
            pts = tr.get((d, s))
            if not pts:
                continue
            row = [d, s]
            for ok in [lambda f, g, gt=gt: g <= gt for gt in GRAD] + [lambda f, g: f <= fstar * (1 + FREL)]:
                r = first(pts, ok)
                row += [f"{r[0]:.6g}", r[1]] if r else ["", ""]
            it, t, f, g = pts[-1]
            row += [f"{fstar:.10g}", f"{f:.10g}", f"{f / fstar - 1:.3e}", f"{g:.3e}", f"{t:.6g}"]
            w.writerow(row)
print(f"wrote {dst}")
