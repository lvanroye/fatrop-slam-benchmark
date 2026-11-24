# slam-benchmark

Pose-graph SLAM benchmark: [fatrop](https://github.com/meco-group/fatrop) vs
[Ceres](http://ceres-solver.org), [g2o](https://github.com/RainerKuemmerle/g2o) and
[GTSAM](https://github.com/borglab/gtsam) on public 2D (SE(2)) and 3D (SE(3))
`.g2o` datasets.

Each solver runs its out-of-the-box algorithm with all default settings,
including its default iteration cap. Only log output is silenced. The problem
formulation (residual, Jacobians, retraction) is shared, so all four minimise
the same objective from the same initial guess.

## Layout

```
external/            solver sources (git submodules)
scripts/             build_deps.sh, download_datasets.sh, run_benchmarks.sh
src/common/          SE(2)/SE(3) definitions, .g2o loader, objective and gradient
src/solvers/<name>/  one adapter per solver: fatrop, ceres, g2o, gtsam
src/main.cpp         runs every solver on one dataset and prints the results
```

## Solvers

| solver | version | algorithm (default) | linear solver | license |
|---|---|---|---|---|
| fatrop | graph-perf branch | interior point, filter line search | built-in block-sparse Cholesky (blasfeo) | BSD-2 / EPL-2.0 |
| Ceres | 2.2.0 | Levenberg–Marquardt | sparse normal Cholesky, Eigen backend | BSD-3 |
| g2o | 20241228 | Levenberg–Marquardt | block solver + Eigen sparse Cholesky | BSD (core) |
| GTSAM | 4.3.0 | Levenberg–Marquardt | multifrontal Cholesky | BSD |

All dependencies are permissively licensed: Eigen is MPL-2.0 and blasfeo is
BSD-2. SuiteSparse/CHOLMOD (LGPL/GPL) is disabled everywhere.

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
scripts/build_deps.sh            # builds external/* into build/deps (JOBS=4 by default)
scripts/download_datasets.sh
cmake -S . -B build && cmake --build build -j4
scripts/run_benchmarks.sh        # or: build/slam_benchmark datasets/intel.g2o
```

Environment overrides for `run_benchmarks.sh`:

- `SOLVERS=fatrop,ceres`: run only some of the solvers. Each solver runs in its own process.
- `TIMEOUT=300`: stop a solver after that many seconds and report a timeout.
- `MAX_ITERATIONS`: override every solver's iteration cap. By default each solver keeps its own;
  g2o has none, so it gets 100 like GTSAM.
- `BIN`: the benchmark binary (default `build/slam_benchmark`).

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

## Results

`TIMEOUT=300 taskset -c 2 scripts/run_benchmarks.sh` on an Intel Core i7-10850H, one pinned core per
run, with the submodule versions of this commit. Each cell is wall time / iterations.

| dataset | dim | poses | edges | fatrop | Ceres | g2o | GTSAM |
|---|---|--:|--:|--:|--:|--:|--:|
| intel | 2D | 1,728 | 2,512 | **0.025 s / 4** | 0.032 s / 6 | 0.072 s / 27 | 0.052 s / 3 |
| CSAIL | 2D | 1,045 | 1,172 | **0.016 s / 12** | 0.028 s / 15 | 0.039 s / 38 | 0.036 s / 5 |
| MIT | 2D | 808 | 827 | **0.036 s / 42 ⚠** | 0.054 s / 50 ✗ | 0.060 s / 100 ✗ | 0.640 s / 69 ⚠ |
| kitti_00 | 2D | 4,541 | 4,677 | **0.049 s / 5** | 0.098 s / 12 | 0.161 s / 33 | 0.226 s / 7 |
| manhattan | 2D | 3,500 | 5,453 | **0.071 s / 6** | 0.356 s / 34 | 0.582 s / 72 | 0.264 s / 7 |
| city10000 | 2D | 10,000 | 20,687 | **0.300 s / 7** | 0.694 s / 9 | 8.15 s / 100 ✗ | 1.06 s / 8 |
| ais2klinik | 2D | 15,115 | 16,727 | **0.249 s / 7** | 1.21 s / 50 ✗ | 2.16 s / 100 ✗ | 1.31 s / 9 |
| parking-garage | 3D | 1,661 | 6,275 | **0.058 s / 3** | 0.543 s / 19 | 1.09 s / 49 | 0.179 s / 5 ⚠ |
| sphere2500 | 3D | 2,500 | 4,949 | **0.190 s / 7** | 2.94 s / 13 | 6.59 s / 24 | 0.423 s / 7 |
| sphere_bignoise_vertex3 | 3D | 2,200 | 8,647 | **15.3 s / 410** | 11.7 s / 50 ✗ | 22.4 s / 100 ✗ | 8.05 s / 100 ✗ |
| cubicle | 3D | 5,750 | 16,869 | **0.603 s / 9** | 9.39 s / 31 | 13.9 s / 39 | 18.8 s / 68 |
| torus3D | 3D | 5,000 | 9,048 | 10.5 s / 212 | 26.2 s / 50 ✗ | 50.8 s / 100 ✗ | **8.12 s / 57** |
| rim | 3D | 10,195 | 29,743 | **2.84 s / 23 ⚠** | 40.7 s / 50 ✗ | 87.7 s / 76 | 55.6 s / 100 ✗ |
| grid3D | 3D | 8,000 | 22,236 | 181.0 s / 100 | timeout | timeout | **172.0 s / 52 ⚠** |

- **Bold**: fastest solver that converged on that dataset.
- ✗: did not converge. The solver hit its default iteration cap, or got stuck far from a minimum.
- ⚠: converged, but at an objective more than 1% above the best any solver reached on that dataset.
  Pose-graph optimisation is non-convex, so solvers can end in different local minima.
- Iterations are counted the way each solver reports them: GTSAM counts accepted steps only, a g2o
  iteration can include several rejected attempts, a Ceres iteration is one linear solve.
- Ceres stops on relative cost change and GTSAM on relative and absolute error change, so both often
  stop at a larger gradient than fatrop and g2o. The `grad_inf` column of the raw output shows this.
