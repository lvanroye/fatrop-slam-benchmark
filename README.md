# slam-benchmark

Pose-graph SLAM benchmark: [fatrop](https://github.com/meco-group/fatrop) vs
[Ceres](http://ceres-solver.org), [g2o](https://github.com/RainerKuemmerle/g2o) and
[GTSAM](https://github.com/borglab/gtsam) on public 2D (SE(2)) and 3D (SE(3))
`.g2o` datasets.

Each solver runs its out-of-the-box algorithm with its default settings. Only log output is
silenced; the iteration caps can be lifted from the command line. The problem
formulation (residual, Jacobians, retraction) is shared, so all four minimise the same objective
from the same initial guess.

## Layout

```
external/            solver sources (git submodules)
scripts/             build_deps.sh, download_datasets.sh, run_benchmarks.sh,
                     run_traces.sh, time_to_target.py
src/common/          SE(2)/SE(3) definitions, .g2o loader, objective and gradient, trace recorder
src/solvers/<name>/  one adapter per solver: fatrop, ceres, g2o, gtsam
src/main.cpp         runs every solver on one dataset and prints the results
```

## Solvers

| solver | version | algorithm (default) | linear solver | license |
|---|---|---|---|---|
| fatrop | graph-perf branch | interior point, filter line search | built-in block-sparse Cholesky (blasfeo) | BSD-2 / EPL-2.0 |
| Ceres | 2.2.0 | Levenberg–Marquardt | Eigen simplicial sparse Cholesky | BSD-3 |
| g2o | 20241228 | Levenberg–Marquardt | Eigen simplicial sparse Cholesky | BSD (core) |
| GTSAM | 4.3.0 | Levenberg–Marquardt | multifrontal Cholesky | BSD |

The default build uses only permissively licensed dependencies: Eigen is MPL-2.0 and
blasfeo is BSD-2. SuiteSparse/CHOLMOD is disabled there.

### Optional CHOLMOD variants

With `-DSLAM_CHOLMOD=ON`, Ceres and g2o get extra variants that use supernodal CHOLMOD as
their linear solver:

- `ceres-cholmod`: Ceres with `SUITE_SPARSE` and AMD ordering.
- `ceres-cholmod-nd`: the same with nested-dissection ordering (METIS).
- `g2o-cholmod`: g2o with `LinearSolverCholmod`.

The plain `ceres` variant pins `EIGEN_SPARSE`, since a SuiteSparse-enabled Ceres would
otherwise default to CHOLMOD.

CHOLMOD's supernodal module is GPL (the rest of CHOLMOD is LGPL), so these builds are GPL.
This repository contains no SuiteSparse code; it only calls the Ceres and g2o APIs.

The dependencies are not built by `build_deps.sh`; build them by hand:

- SuiteSparse (CHOLMOD with its supernodal module, plus SPQR, which Ceres requires),
  single-threaded OpenBLAS and METIS, installed into `build/deps-suitesparse`.
- Ceres and g2o configured against that prefix, installed into `build/deps-cholmod`.

CMake then picks up both prefixes.

## Problem

- **Objective:** the mean over all scalar residuals,
  `f(x) = (1/m) · 0.5 Σ_e r_e(x)ᵀ Ω_e r_e(x)`, with `m` = edges × residual size and the
  information matrices `Ω_e` taken from the file. The scaling doesn't change the minimiser;
  it only keeps the objective and gradient of the large datasets in a sane range.
- **SE(2) residual:** `r = (R(θ_a)ᵀ(t_b − t_a) − z_t, wrap(θ_b − θ_a − z_θ))`.
- **SE(3) residual:** `r = (R_aᵀ(p_b − p_a) − z_t, Log(z_q⁻¹ q_a⁻¹ q_b))`.
- **Retraction:** world-frame translation and body-frame rotation. Every solver uses
  the same residual and analytic Jacobians (`src/common/lie.hpp`).
- **Gauge:** the first vertex, or the `FIX` vertex, is held at its initial value.
  - Ceres: `SetParameterBlockConstant`
  - g2o: `setFixed`
  - GTSAM: a tight `PriorFactor`
  - fatrop: a prior with weight 1e6, since it cannot fix a variable block
- **Information matrices:** some datasets contain ones that are not positive definite
  (cubicle, rim, parking-garage). Their eigenvalues are clamped once at load time,
  so every solver sees the same convex edge costs. The count is printed as
  `repaired_info`.

## Datasets

`scripts/download_datasets.sh` fetches the 14 files from the `data/` folder of
[SE-Sync](https://github.com/david-m-rosen/SE-Sync/tree/master/data)
(D. M. Rosen et al., *SE-Sync: A certifiably correct algorithm for synchronization
over the special Euclidean group*, IJRR 2019). That folder collects the standard
pose-graph benchmarks; most were published earlier by
[Luca Carlone](https://lucacarlone.mit.edu/datasets/), and kitti_00 is built from
[KITTI](https://www.cvlibs.net/datasets/kitti/eval_odometry.php) odometry sequence 00.

| 2D (SE(2)) | 3D (SE(3)) |
|---|---|
| intel, manhattan, MIT, CSAIL, kitti_00, city10000, ais2klinik | parking-garage, sphere2500, sphere_bignoise_vertex3, cubicle, torus3D, grid3D, rim |

The same selection was used earlier in fatrop's `benchmark_pgo/` (branch `slam_tests`).
For files that only list edges, the loader seeds the initial guess by dead reckoning
along the odometry chain.

## Build and run

```bash
git submodule update --init
scripts/build_deps.sh            # builds external/* into build/deps
scripts/download_datasets.sh
cmake -S . -B build && cmake --build build -j4
scripts/run_benchmarks.sh        # or: build/slam_benchmark datasets/intel.g2o
```

Options of `run_benchmarks.sh` (files given after the options replace the default `datasets/*.g2o`):

- `--solvers fatrop,ceres`: run only some of the solvers. Each solver runs in its own process.
- `--timeout 600`: stop a solver after that many seconds and report a timeout.
- `--max-iterations N`: override every solver's iteration cap. By default each solver keeps its own;
  g2o has none, so it gets 100 like GTSAM.
- `--bin PATH`: the benchmark binary (default `build/slam_benchmark`).

`build_deps.sh` takes `--jobs N` (default 4) and `--prefix PATH` (default `build/deps`), followed by
the components to build (default: all).

## Output

For each dataset, one line per solver:

- `iter`: the iterations reported by the solver.
- `time_s`: wall time for building that solver's problem plus solving it, on a single
  thread.
- `objective`: `f` at the returned poses, evaluated with the shared code, so the
  values are comparable between solvers.
- `grad_inf`: the max-norm of the gradient of `f` at the returned poses, in the
  shared tangent space, with the gauge pose excluded. This is the optimality measure.
- `status`: whether the solver reported convergence before the iteration cap.

## Running the benchmark

For timings, pin each run to one core, lift the iteration caps and use a time limit instead:

```bash
taskset -c 2 scripts/run_benchmarks.sh --max-iterations 1000000 --timeout 600
```

Every other setting is each solver's default, including its convergence criterion, so the solvers
stop at different accuracies. fatrop stops at its tolerance (1e-8, or 1e-6 at an acceptable point);
g2o keeps iterating until it can no longer lower the cost; Ceres and GTSAM stop on relative cost
change. Compare `grad_inf` alongside `time_s`.

## Time to a common target

Each solver's default stopping rule differs, so the default run compares solvers at different
accuracies. Trace mode compares them at the same one: every solver records each iterate, and the
time to the first iterate below a shared gradient (or objective) target is read off afterwards.

```bash
taskset -c 2 scripts/run_traces.sh --out traces.csv
scripts/time_to_target.py traces.csv summary.csv
```

- Per dataset and solver, `run_traces.sh` runs two processes, one at a time:
  - Probe: runs until `grad_inf <= 1e-10`, until the solver can make no more progress, or until
    the time limit. This gives the iteration count N.
  - Measure: reruns capped at N iterations.
- During the measured solve, each iteration only reads the clock and copies the iterate, and that
  time is subtracted. Objective and gradient are evaluated after the solve.
- In trace mode each solver's own convergence test is disabled:
  - Ceres: all tolerances 0, plus an `IterationCallback`.
  - fatrop: tolerances 1e-14, plus the NLP callback.
  - g2o: a post-iteration action and the force-stop flag.
  - GTSAM: `iterate()` in a loop, stopping when a step no longer lowers the error.
- `traces.csv` has one row per iterate: `dataset,solver,iteration,time_s,objective,grad_inf`.
- `summary.csv` has the time and iteration count to reach `grad_inf` <= 1e-4, 1e-6 and 1e-8, and to
  get within 1e-6 of the lowest objective any solver reached. It also has the final objective and
  gradient.
- The binary takes the same options directly: `--trace FILE`, `--grad-target G`,
  `--time-limit S`.

