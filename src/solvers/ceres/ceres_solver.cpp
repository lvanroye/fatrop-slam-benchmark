// Ceres: one ambient parameter block per pose, a Manifold built from S's
// retraction, and one analytic cost function per edge.
#include "solvers/solvers.hpp"

#include <ceres/ceres.h>

#include <algorithm>
#include <chrono>
#include <limits>

namespace slam
{
namespace
{

template <class S>
using RowMajor = Eigen::Matrix<double, S::dim, S::nx, Eigen::RowMajor>;

// P^+ = (P^T P)^{-1} P^T, the left inverse of the plus Jacobian.
template <class S>
Eigen::Matrix<double, S::dim, S::nx> plus_jacobian_pinv(const typename S::Pose &x)
{
    const typename S::PlusJac P = S::plus_jacobian(x);
    return (P.transpose() * P).ldlt().solve(P.transpose());
}

template <class S>
class PoseManifold final : public ceres::Manifold
{
  public:
    using Pose = typename S::Pose;
    using Tangent = typename S::Tangent;

    int AmbientSize() const override { return S::nx; }
    int TangentSize() const override { return S::dim; }

    bool Plus(const double *x, const double *delta, double *x_plus_delta) const override
    {
        Eigen::Map<Pose> out(x_plus_delta);
        out = S::retract(Eigen::Map<const Pose>(x), Eigen::Map<const Tangent>(delta));
        return true;
    }

    bool PlusJacobian(const double *x, double *jacobian) const override
    {
        Eigen::Map<Eigen::Matrix<double, S::nx, S::dim, Eigen::RowMajor>> out(jacobian);
        out = S::plus_jacobian(Eigen::Map<const Pose>(x));
        return true;
    }

    bool Minus(const double *y, const double *x, double *y_minus_x) const override
    {
        Eigen::Map<Tangent> out(y_minus_x);
        out = S::local(Eigen::Map<const Pose>(x), Eigen::Map<const Pose>(y));
        return true;
    }

    bool MinusJacobian(const double *x, double *jacobian) const override
    {
        Eigen::Map<RowMajor<S>> out(jacobian);
        out = plus_jacobian_pinv<S>(Eigen::Map<const Pose>(x));
        return true;
    }
};

template <class S>
class EdgeCost final : public ceres::SizedCostFunction<S::dim, S::nx, S::nx>
{
  public:
    explicit EdgeCost(const Edge<S> &e) : z_(e.z), sqrt_info_(e.sqrt_info) {}

    bool Evaluate(const double *const *parameters, double *residuals, double **jacobians) const override
    {
        const Eigen::Map<const typename S::Pose> a(parameters[0]), b(parameters[1]);
        typename S::Tangent r;
        typename S::Jac Ja, Jb;
        const bool jac = jacobians && (jacobians[0] || jacobians[1]);
        S::residual(a, b, z_, r, jac ? &Ja : nullptr, jac ? &Jb : nullptr);
        Eigen::Map<typename S::Tangent> res(residuals);
        res = sqrt_info_ * r;
        if (jacobians && jacobians[0])
        {
            Eigen::Map<RowMajor<S>> J(jacobians[0]);
            J = sqrt_info_ * Ja * plus_jacobian_pinv<S>(a);
        }
        if (jacobians && jacobians[1])
        {
            Eigen::Map<RowMajor<S>> J(jacobians[1]);
            J = sqrt_info_ * Jb * plus_jacobian_pinv<S>(b);
        }
        return true;
    }

  private:
    const typename S::Pose z_;
    const typename S::Jac sqrt_info_;
};

template <class S>
class TraceCallback final : public ceres::IterationCallback
{
  public:
    TraceCallback(Tracer<S> &tracer, const std::vector<typename S::Pose> &x) : tracer_(tracer), x_(x) {}
    ceres::CallbackReturnType operator()(const ceres::IterationSummary &it) override
    {
        const bool stop = tracer_.record(it.iteration, [&](double *out) {
            for (const auto &xi : x_)
                out = std::copy(xi.data(), xi.data() + S::nx, out);
        });
        return stop ? ceres::SOLVER_TERMINATE_SUCCESSFULLY : ceres::SOLVER_CONTINUE;
    }

  private:
    Tracer<S> &tracer_;
    const std::vector<typename S::Pose> &x_;
};

template <class S>
Result solve(const Problem<S> &problem, const Options &options, bool cholmod = false,
             ceres::LinearSolverOrderingType ordering = ceres::AMD)
{
    Result res;
    res.solver = "ceres";
    std::vector<typename S::Pose> x(problem.poses.begin(), problem.poses.end());

    const auto t0 = std::chrono::steady_clock::now();
    Tracer<S> tracer(problem, options);
    ceres::Problem cp;
    PoseManifold<S> *manifold = new PoseManifold<S>;
    for (auto &xi : x)
        cp.AddParameterBlock(xi.data(), S::nx, manifold);
    for (const auto &e : problem.edges)
        cp.AddResidualBlock(new EdgeCost<S>(e), nullptr, x[e.a].data(), x[e.b].data());
    cp.SetParameterBlockConstant(x[problem.fixed].data());

    ceres::Solver::Options so;
    if (options.max_iterations > 0)
        so.max_num_iterations = options.max_iterations;
    so.logging_type = ceres::SILENT;
    // Pin the backend: a SuiteSparse-enabled Ceres defaults to CHOLMOD.
    so.linear_solver_type = ceres::SPARSE_NORMAL_CHOLESKY;
    so.sparse_linear_algebra_library_type = cholmod ? ceres::SUITE_SPARSE : ceres::EIGEN_SPARSE;
    so.linear_solver_ordering_type = ordering;
    TraceCallback<S> callback(tracer, x);
    if (options.trace)
    {
        so.function_tolerance = 0.0;
        so.gradient_tolerance = 0.0;
        so.parameter_tolerance = 0.0;
        so.max_num_iterations = options.max_iterations > 0 ? options.max_iterations : std::numeric_limits<int>::max();
        so.update_state_every_iteration = true;
        so.callbacks.push_back(&callback);
    }
    ceres::Solver::Summary summary;
    ceres::Solve(so, &cp, &summary);
    res.time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();

    res.converged = summary.termination_type == ceres::CONVERGENCE;
    res.iterations = static_cast<int>(summary.iterations.size()) - 1;
    if (!res.converged)
        res.status = summary.message;
    res.poses.assign(x.begin(), x.end());
    res.trace = tracer.take();
    return res;
}

} // namespace

Result solve_ceres(const Problem<SE2> &problem, const Options &options) { return solve<SE2>(problem, options); }
Result solve_ceres(const Problem<SE3> &problem, const Options &options) { return solve<SE3>(problem, options); }

#ifdef SLAM_CERES_CHOLMOD
Result solve_ceres_cholmod(const Problem<SE2> &p, const Options &o) { return solve<SE2>(p, o, true); }
Result solve_ceres_cholmod(const Problem<SE3> &p, const Options &o) { return solve<SE3>(p, o, true); }
Result solve_ceres_cholmod_nd(const Problem<SE2> &p, const Options &o) { return solve<SE2>(p, o, true, ceres::NESDIS); }
Result solve_ceres_cholmod_nd(const Problem<SE3> &p, const Options &o) { return solve<SE3>(p, o, true, ceres::NESDIS); }
#endif

} // namespace slam
