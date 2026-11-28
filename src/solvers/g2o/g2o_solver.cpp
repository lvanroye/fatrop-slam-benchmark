#include "solvers/solvers.hpp"

#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/hyper_graph_action.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#ifdef SLAM_G2O_CHOLMOD
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#endif

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <vector>

namespace slam
{
namespace
{

template <class S>
class Vertex : public g2o::BaseVertex<S::dim, typename S::Pose>
{
public:
    void setToOriginImpl() override { this->_estimate = S::identity(); }
    void oplusImpl(const double *d) override
    {
        this->_estimate = S::retract(this->_estimate, Eigen::Map<const typename S::Tangent>(d));
    }
    bool read(std::istream &) override { return false; }
    bool write(std::ostream &) const override { return false; }
};

template <class S>
class Edge2 : public g2o::BaseBinaryEdge<S::dim, typename S::Pose, Vertex<S>, Vertex<S>>
{
public:
    const typename S::Pose &pose(int i) const
    {
        return static_cast<const Vertex<S> *>(this->_vertices[i])->estimate();
    }
    void computeError() override
    {
        typename S::Tangent r;
        S::residual(pose(0), pose(1), this->_measurement, r);
        this->_error = r;
    }
    void linearizeOplus() override
    {
        typename S::Tangent r;
        typename S::Jac Ja, Jb;
        S::residual(pose(0), pose(1), this->_measurement, r, &Ja, &Jb);
        this->_jacobianOplusXi = Ja;
        this->_jacobianOplusXj = Jb;
    }
    bool read(std::istream &) override { return false; }
    bool write(std::ostream &) const override { return false; }
};

template <class S>
class TraceAction final : public g2o::HyperGraphAction
{
public:
    TraceAction(Tracer<S> &tracer, const std::vector<Vertex<S> *> &v, bool &stop)
        : tracer_(tracer), v_(v), stop_(stop)
    {
    }
    g2o::HyperGraphAction *operator()(const g2o::HyperGraph *, Parameters *) override
    {
        stop_ = record(tracer_, ++iteration_, v_);
        return this;
    }
    static bool record(Tracer<S> &tracer, int iteration, const std::vector<Vertex<S> *> &v)
    {
        return tracer.record(iteration, [&](double *out) {
            for (const auto *vk : v)
                out = std::copy(vk->estimate().data(), vk->estimate().data() + S::nx, out);
        });
    }

private:
    Tracer<S> &tracer_;
    const std::vector<Vertex<S> *> &v_;
    bool &stop_;
    int iteration_ = 0;
};

template <class S, template <class> class LinearSolver = g2o::LinearSolverEigen>
Result solve(const Problem<S> &problem, const Options &options)
{
    using Block = g2o::BlockSolver<g2o::BlockSolverTraits<S::dim, S::dim>>;
    using Linear = LinearSolver<typename Block::PoseMatrixType>;

    const auto t0 = std::chrono::steady_clock::now();
    Tracer<S> tracer(problem, options);
    g2o::SparseOptimizer optimizer;
    optimizer.setVerbose(false);
    optimizer.setAlgorithm(new g2o::OptimizationAlgorithmLevenberg(
        std::make_unique<Block>(std::make_unique<Linear>())));

    std::vector<Vertex<S> *> vertices(problem.poses.size());
    for (std::size_t k = 0; k < problem.poses.size(); ++k)
    {
        auto *v = new Vertex<S>;
        v->setId(static_cast<int>(k));
        v->setEstimate(problem.poses[k]);
        v->setFixed(static_cast<int>(k) == problem.fixed);
        optimizer.addVertex(v);
        vertices[k] = v;
    }
    for (const auto &e : problem.edges)
    {
        auto *edge = new Edge2<S>;
        edge->setVertex(0, vertices[e.a]);
        edge->setVertex(1, vertices[e.b]);
        edge->setMeasurement(e.z);
        edge->setInformation(e.info);
        optimizer.addEdge(edge);
    }

    optimizer.initializeOptimization();
    // g2o has no default iteration count; use GTSAM's default of 100.
    int cap = options.max_iterations > 0 ? options.max_iterations : 100;
    bool stop = false;
    TraceAction<S> action(tracer, vertices, stop);
    if (options.trace)
    {
        cap = options.max_iterations > 0 ? options.max_iterations : std::numeric_limits<int>::max();
        optimizer.setForceStopFlag(&stop);
        optimizer.addPostIterationAction(&action);
        stop = TraceAction<S>::record(tracer, 0, vertices);
    }
    const int iterations = optimizer.optimize(cap);
    const auto t1 = std::chrono::steady_clock::now();

    Result res;
    res.solver = "g2o";
    res.iterations = iterations;
    res.converged = iterations > 0 && iterations < cap;
    res.time_s = std::chrono::duration<double>(t1 - t0).count();
    if (iterations <= 0)
        res.status = "failed";
    for (const auto *v : vertices)
        res.poses.emplace_back(v->estimate());
    if (options.trace)
        optimizer.removePostIterationAction(&action);
    res.trace = tracer.take();
    return res;
}

} // namespace

Result solve_g2o(const Problem<SE2> &problem, const Options &options) { return solve(problem, options); }
Result solve_g2o(const Problem<SE3> &problem, const Options &options) { return solve(problem, options); }

#ifdef SLAM_G2O_CHOLMOD
Result solve_g2o_cholmod(const Problem<SE2> &p, const Options &o) { return solve<SE2, g2o::LinearSolverCholmod>(p, o); }
Result solve_g2o_cholmod(const Problem<SE3> &p, const Options &o) { return solve<SE3, g2o::LinearSolverCholmod>(p, o); }
#endif

} // namespace slam
