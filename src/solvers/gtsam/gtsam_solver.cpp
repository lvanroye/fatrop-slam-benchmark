// GTSAM: Pose2/Pose3 values, one custom factor per edge whose error is
// S::residual, Levenberg-Marquardt with default parameters. The gauge is
// a PriorFactor on problem.fixed with isotropic sigma 1e-3.
#include "solvers/solvers.hpp"

#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/PriorFactor.h>
#include <gtsam/nonlinear/Values.h>

#include <chrono>
#include <limits>

namespace slam
{
namespace
{

template <class S>
struct Traits;

// Pose2 tangent (v, w) in the body frame: t' = t + R v, th' = th + w.
template <>
struct Traits<SE2>
{
    using Pose = gtsam::Pose2;
    static Pose to_gtsam(const SE2::Pose &x) { return Pose(x[0], x[1], x[2]); }
    static SE2::Pose from_gtsam(const Pose &p) { return SE2::Pose(p.x(), p.y(), p.theta()); }
    static void map_jacobian(const SE2::Jac &J, const Pose &p, gtsam::Matrix *H)
    {
        SE2::Jac Jg;
        Jg.leftCols<2>() = J.leftCols<2>() * p.rotation().matrix();
        Jg.col(2) = J.col(2);
        *H = Jg;
    }
};

// Pose3 tangent (w, v) in the body frame: R' = R Exp(w), t' = t + R v.
template <>
struct Traits<SE3>
{
    using Pose = gtsam::Pose3;
    static Pose to_gtsam(const SE3::Pose &x)
    {
        return Pose(gtsam::Rot3::Quaternion(x[6], x[3], x[4], x[5]), gtsam::Point3(x[0], x[1], x[2]));
    }
    static SE3::Pose from_gtsam(const Pose &p)
    {
        return SE3::make(p.translation(), p.rotation().toQuaternion());
    }
    static void map_jacobian(const SE3::Jac &J, const Pose &p, gtsam::Matrix *H)
    {
        SE3::Jac Jg;
        Jg.leftCols<3>() = J.rightCols<3>();
        Jg.rightCols<3>() = J.leftCols<3>() * p.rotation().matrix();
        *H = Jg;
    }
};

template <class S>
class EdgeFactor final : public gtsam::NoiseModelFactorN<typename Traits<S>::Pose, typename Traits<S>::Pose>
{
    using Pose = typename Traits<S>::Pose;
    using Base = gtsam::NoiseModelFactorN<Pose, Pose>;
    typename S::Pose z_;

  public:
    using Base::evaluateError;

    EdgeFactor(gtsam::Key a, gtsam::Key b, const Edge<S> &e)
        : Base(gtsam::noiseModel::Gaussian::SqrtInformation(e.sqrt_info), a, b), z_(e.z)
    {
    }

    gtsam::Vector evaluateError(const Pose &pa, const Pose &pb, gtsam::Matrix *Ha,
                                gtsam::Matrix *Hb) const override
    {
        typename S::Tangent r;
        typename S::Jac Ja, Jb;
        const bool jac = Ha || Hb;
        S::residual(Traits<S>::from_gtsam(pa), Traits<S>::from_gtsam(pb), z_, r, jac ? &Ja : nullptr,
                    jac ? &Jb : nullptr);
        if (Ha)
            Traits<S>::map_jacobian(Ja, pa, Ha);
        if (Hb)
            Traits<S>::map_jacobian(Jb, pb, Hb);
        return r;
    }
};

template <class S>
Result solve(const Problem<S> &problem, const Options &options)
{
    using Pose = typename Traits<S>::Pose;
    const auto t0 = std::chrono::steady_clock::now();
    Tracer<S> tracer(problem, options);

    gtsam::NonlinearFactorGraph graph;
    gtsam::Values initial;
    for (std::size_t k = 0; k < problem.poses.size(); ++k)
        initial.insert(k, Traits<S>::to_gtsam(problem.poses[k]));
    for (const auto &e : problem.edges)
        graph.emplace_shared<EdgeFactor<S>>(e.a, e.b, e);
    graph.emplace_shared<gtsam::PriorFactor<Pose>>(problem.fixed, initial.at<Pose>(problem.fixed),
                                                   gtsam::noiseModel::Isotropic::Sigma(S::dim, 1e-3));

    gtsam::LevenbergMarquardtParams params;
    if (options.max_iterations > 0)
        params.maxIterations = options.max_iterations;
    params.verbosity = gtsam::NonlinearOptimizerParams::SILENT;
    params.verbosityLM = gtsam::LevenbergMarquardtParams::SILENT;
    gtsam::LevenbergMarquardtOptimizer optimizer(graph, initial, params);
    gtsam::Values values;
    if (!options.trace)
        values = optimizer.optimize();
    else
    {
        // LM steps until the trace target, or until a step no longer lowers the error.
        params.maxIterations = options.max_iterations > 0 ? options.max_iterations : std::numeric_limits<int>::max();
        auto record = [&] {
            return tracer.record(static_cast<int>(optimizer.iterations()), [&](double *out) {
                for (std::size_t k = 0; k < problem.poses.size(); ++k, out += S::nx)
                    Eigen::Map<typename S::Pose>{out} = Traits<S>::from_gtsam(optimizer.values().template at<Pose>(k));
            }) || optimizer.iterations() >= params.maxIterations;
        };
        bool stop = record();
        while (!stop)
        {
            const double before = optimizer.error();
            optimizer.iterate();
            stop = record() || !(optimizer.error() < before) || optimizer.lambda() >= params.lambdaUpperBound;
        }
        values = optimizer.values();
    }

    Result res;
    res.solver = "gtsam";
    res.time_s = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    res.iterations = static_cast<int>(optimizer.iterations());
    const bool capped = optimizer.iterations() >= params.maxIterations;
    const bool gave_up = optimizer.lambda() >= params.lambdaUpperBound;
    res.converged = !capped && !gave_up;
    res.status = capped ? "max iterations" : gave_up ? "lambda limit" : "error tolerance";
    for (std::size_t k = 0; k < problem.poses.size(); ++k)
        res.poses.emplace_back(Traits<S>::from_gtsam(values.at<Pose>(k)));
    res.trace = tracer.take();
    return res;
}

} // namespace

Result solve_gtsam(const Problem<SE2> &problem, const Options &options) { return solve(problem, options); }
Result solve_gtsam(const Problem<SE3> &problem, const Options &options) { return solve(problem, options); }

} // namespace slam
