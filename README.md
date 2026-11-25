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
| Ceres | 2.2.0 | Levenberg–Marquardt | Eigen simplicial sparse Cholesky | BSD-3 |
| g2o | 20241228 | Levenberg–Marquardt | Eigen simplicial sparse Cholesky | BSD (core) |
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

## Results

`taskset -c 2 scripts/run_benchmarks.sh --max-iterations 1000000 --timeout 600` on an Intel Core
i7-10850H: no iteration cap, a 600 s time limit per run, one pinned core per run. Every other
setting is each solver's default, including its convergence criterion, so the solvers stop at
different accuracies (see the accuracy table). Each cell is wall time / iterations.

| dataset | dim | poses | edges | fatrop | Ceres | g2o | GTSAM |
|---|---|--:|--:|--:|--:|--:|--:|
| intel | 2D | 1,728 | 2,512 | **0.021 s / 4** | 0.029 s / 6 | 0.071 s / 27 | 0.047 s / 3 |
| CSAIL | 2D | 1,045 | 1,172 | **0.016 s / 12** | 0.027 s / 15 | 0.037 s / 38 | 0.036 s / 5 |
| MIT | 2D | 808 | 827 | **0.034 s / 42** | 0.414 s / 405 | 0.175 s / 312 | 0.621 s / 69 |
| kitti_00 | 2D | 4,541 | 4,677 | **0.047 s / 5** | 0.088 s / 12 | 0.156 s / 33 | 0.221 s / 7 |
| manhattan | 2D | 3,500 | 5,453 | **0.062 s / 6** | 0.320 s / 34 | 0.549 s / 72 | 0.237 s / 7 |
| city10000 | 2D | 10,000 | 20,687 | **0.287 s / 7** | 0.672 s / 9 | 13.0 s / 180 | 1.02 s / 8 |
| ais2klinik | 2D | 15,115 | 16,727 | **0.241 s / 7** | 2.93 s / 124 | 7.14 s / 360 | 1.34 s / 9 |
| parking-garage | 3D | 1,661 | 6,275 | **0.061 s / 3** | 0.535 s / 19 | 1.07 s / 49 | 0.170 s / 5 |
| sphere2500 | 3D | 2,500 | 4,949 | **0.201 s / 7** | 2.88 s / 13 | 6.44 s / 24 | 0.407 s / 7 |
| sphere_bignoise_vertex3 | 3D | 2,200 | 8,647 | 15.2 s / 410 | 36.9 s / 159 | 229.8 s / 867 | **12.7 s / 106** |
| cubicle | 3D | 5,750 | 16,869 | **1.09 s / 9** | 12.6 s / 31 | 16.9 s / 39 | 20.2 s / 68 |
| torus3D | 3D | 5,000 | 9,048 | 11.5 s / 212 | 58.8 s / 95 | 231.2 s / 377 | **9.12 s / 57** |
| rim | 3D | 10,195 | 29,743 | **3.14 s / 23** | 106.8 s / 116 | 95.3 s / 76 | 139.6 s / 215 |
| grid3D | 3D | 8,000 | 22,236 | **247.9 s / 100** | x | x | 328.3 s / 52 |

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
