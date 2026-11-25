# slam-benchmark

Pose-graph SLAM benchmark: [fatrop](https://github.com/meco-group/fatrop) vs
[Ceres](http://ceres-solver.org), [g2o](https://github.com/RainerKuemmerle/g2o) and
[GTSAM](https://github.com/borglab/gtsam) on public 2D (SE(2)) and 3D (SE(3))
`.g2o` datasets.

Each solver runs its out-of-the-box algorithm with its default settings. Only log output is
silenced, and the results below lift the iteration caps and use a time limit instead. The problem
formulation (residual, Jacobians, retraction) is shared, so all four minimise the same objective
from the same initial guess.

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

`MAX_ITERATIONS=1000000 TIMEOUT=600 taskset -c 2 scripts/run_benchmarks.sh` on an Intel Core
i7-10850H: no iteration cap, a 600 s time limit per run, one pinned core per run. Each cell is wall
time / iterations.

| dataset | dim | poses | edges | fatrop | Ceres | g2o | GTSAM |
|---|---|--:|--:|--:|--:|--:|--:|
| intel | 2D | 1,728 | 2,512 | **0.029 s / 4** | 0.034 s / 6 | 0.082 s / 27 | 0.059 s / 3 |
| CSAIL | 2D | 1,045 | 1,172 | **0.020 s / 12** | 0.032 s / 15 | 0.045 s / 38 | 0.044 s / 5 |
| MIT | 2D | 808 | 827 | **0.041 s / 42** | 0.498 s / 405 | 0.207 s / 312 | 0.687 s / 69 |
| kitti_00 | 2D | 4,541 | 4,677 | **0.056 s / 5** | 0.099 s / 12 | 0.172 s / 33 | 0.257 s / 7 |
| manhattan | 2D | 3,500 | 5,453 | **0.081 s / 6** | 0.364 s / 34 | 0.624 s / 72 | 0.263 s / 7 |
| city10000 | 2D | 10,000 | 20,687 | **0.305 s / 7** | 0.741 s / 9 | 14.7 s / 180 | 1.17 s / 8 |
| ais2klinik | 2D | 15,115 | 16,727 | **0.252 s / 7** | 3.15 s / 124 | 8.11 s / 360 | 1.26 s / 9 |
| parking-garage | 3D | 1,661 | 6,275 | **0.061 s / 3** | 0.529 s / 19 | 1.09 s / 49 | 0.178 s / 5 |
| sphere2500 | 3D | 2,500 | 4,949 | **0.197 s / 7** | 2.92 s / 13 | 6.82 s / 24 | 0.416 s / 7 |
| sphere_bignoise_vertex3 | 3D | 2,200 | 8,647 | 15.2 s / 410 | 39.1 s / 159 | 203.9 s / 867 | **9.88 s / 106** |
| cubicle | 3D | 5,750 | 16,869 | **0.689 s / 9** | 9.95 s / 31 | 16.1 s / 39 | 22.2 s / 68 |
| torus3D | 3D | 5,000 | 9,048 | 12.7 s / 212 | 59.1 s / 95 | 203.9 s / 377 | **8.07 s / 57** |
| rim | 3D | 10,195 | 29,743 | **2.82 s / 23** | 94.3 s / 116 | 86.6 s / 76 | 117.7 s / 215 |
| grid3D | 3D | 8,000 | 22,236 | 181.0 s / 100 | x | x | **171.7 s / 52** |

**Bold**: fastest. x: did not converge within the time limit.

Accuracy: final objective and, in brackets, the max-norm of its gradient at the returned poses
(`grad_inf`), same run. **Bold**: lowest objective.

| dataset | fatrop | Ceres | g2o | GTSAM |
|---|--:|--:|--:|--:|
| intel | **2.94704140e-3 (2e-9)** | 2.94704356e-3 (5e-8) | **2.94704140e-3 (6e-15)** | 2.94718934e-3 (6e-7) |
| CSAIL | **8.69496204e-3 (5e-9)** | 8.69496403e-3 (7e-3) | **8.69496204e-3 (1e-10)** | 8.69497621e-3 (1e-4) |
| MIT | 1.55120352e-1 (8e-9) | 1.55120616e-1 (4e-4) | **1.27821902e-1 (4e-10)** | 2.13858247e-1 (2e-1) |
| kitti_00 | **3.50162789e-3 (1e-9)** | 3.50162826e-3 (1e-7) | **3.50162789e-3 (2e-14)** | 3.50162863e-3 (7e-7) |
| manhattan | **1.07997388e-1 (2e-9)** | 1.07997397e-1 (2e-3) | **1.07997388e-1 (2e-11)** | **1.07997388e-1 (9e-7)** |
| city10000 | **4.12485429e-3 (1e-9)** | 4.12485664e-3 (8e-8) | 1.19615031e-2 (1e-11) | **4.12485429e-3 (2e-8)** |
| ais2klinik | **1.72031489e-3 (5e-10)** | 1.72032234e-3 (5e-7) | 1.88272793e-3 (1e-15) | 1.73435986e-3 (3e-6) |
| parking-garage | **1.68444144e-5 (2e-9)** | 1.68444255e-5 (6e-9) | **1.68444144e-5 (2e-17)** | 1.70569540e-5 (1e-6) |
| sphere2500 | **2.27548044e-2 (4e-9)** | 2.27548101e-2 (4e-7) | **2.27548044e-2 (5e-12)** | 2.27548057e-2 (1e-6) |
| sphere_bignoise_vertex3 | **5.71209643e+1 (7e-7)** | 6.73060153e+1 (2e-3) | 7.05031624e+1 (1e-7) | 7.01245293e+1 (7e-3) |
| cubicle | **1.34852682e-2 (4e-10)** | 1.34852790e-2 (7e-5) | **1.34852682e-2 (1e-12)** | **1.34852682e-2 (6e-6)** |
| torus3D | 5.51453932e-1 (6e-7) | 5.51461589e-1 (3e-5) | **5.51453929e-1 (1e-9)** | 5.51582681e-1 (1e-4) |
| rim | 4.50767700e-1 (5e-9) | 4.43719960e-1 (4e-5) | **4.43719386e-1 (2e-11)** | 4.43949884e-1 (4e-2) |
| grid3D | **1.77293263e+0 (2e-7)** | x | x | 1.88651205e+0 (8e-5) |

fatrop stops at its tolerance (1e-8, or 1e-6 at an acceptable point); g2o keeps iterating until it can no
longer lower the cost; Ceres and GTSAM stop on relative cost change.

## Notes on fairness

- Without SuiteSparse/CHOLMOD, Ceres and g2o use Eigen's simplicial sparse Cholesky. With CHOLMOD
  they would be faster, mostly on the 3D datasets.
- The 1/m objective scaling does not change any minimiser, but GTSAM's Levenberg–Marquardt damping
  and error tolerance are absolute, so it makes them relatively stronger.
- Iterations are counted as each solver reports them: GTSAM counts accepted steps only, and a g2o
  iteration can include several rejected attempts.
- Every run is single-threaded; Ceres and GTSAM could use more threads.
- fatrop's graph linear algebra (`graph-perf` branch) was tuned while building this benchmark. Its
  algorithm and settings were not changed.
- Pose-graph optimisation is non-convex: all solvers start from the same initial guess and can end in
  different local minima. None of the results is certified to be the global minimum.
