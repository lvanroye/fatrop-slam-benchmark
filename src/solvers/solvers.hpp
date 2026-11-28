// Every solver in src/solvers/<name>/ implements this pair of functions.
//
// Contract:
//   - minimise slam::objective() using the residual/Jacobians/retraction of
//     the pose space S (src/common/lie.hpp), started from problem.poses;
//   - hold problem.fixed at its initial value (gauge), using the solver's
//     usual mechanism;
//   - use the solver's out-of-the-box algorithm and default settings; only
//     silence its log output, and set the iteration cap only if
//     Options::max_iterations > 0;
//   - time everything from building the solver's problem to the end of the
//     solve (Result::time_s), single-threaded;
//   - return the final poses in S::Pose storage order.
#pragma once

#include "../common/problem.hpp"

#define SLAM_DECLARE_SOLVER(NAME)                                                     \
    namespace slam                                                                    \
    {                                                                                 \
    Result solve_##NAME(const Problem<SE2> &problem, const Options &options);         \
    Result solve_##NAME(const Problem<SE3> &problem, const Options &options);         \
    }

#ifdef SLAM_WITH_FATROP
SLAM_DECLARE_SOLVER(fatrop)
#endif
#ifdef SLAM_WITH_CERES
SLAM_DECLARE_SOLVER(ceres)
#endif
#ifdef SLAM_WITH_G2O
SLAM_DECLARE_SOLVER(g2o)
#endif
#ifdef SLAM_CERES_CHOLMOD
SLAM_DECLARE_SOLVER(ceres_cholmod)
SLAM_DECLARE_SOLVER(ceres_cholmod_nd)
#endif
#ifdef SLAM_G2O_CHOLMOD
SLAM_DECLARE_SOLVER(g2o_cholmod)
#endif
#ifdef SLAM_WITH_GTSAM
SLAM_DECLARE_SOLVER(gtsam)
#endif
