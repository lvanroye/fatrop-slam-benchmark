#include "solvers/solvers.hpp"

#include <fatrop/fatrop.hpp>
#include <fatrop/graph/graph_abstract.hpp>
#include <fatrop/graph/hessian.hpp>
#include <fatrop/graph/jacobian.hpp>
#include <fatrop/graph/nlp_graph.hpp>
#include <fatrop/ip_algorithm/ip_alg_builder.hpp>
#include <fatrop/ip_algorithm/ip_algorithm.hpp>
#include <fatrop/ip_algorithm/ip_data.hpp>
#include <fatrop/linear_algebra/blasfeo_operations.hpp>

#include <algorithm>
#include <chrono>
#include <limits>
#include <memory>
#include <tuple>
#include <vector>

namespace slam
{
namespace
{

using fatrop::Index;
using fatrop::Scalar;

// Gauge prior weight on problem.fixed: 0.5 * w * ||local(x0, x)||^2.
constexpr double prior_weight = 1e6;

template <class S>
class PoseGraph : public fatrop::GraphAbstract
{
    using Pose = typename S::Pose;
    using Tangent = typename S::Tangent;
    using Jac = typename S::Jac;

public:
    explicit PoseGraph(const Problem<S> &p)
        : p_(p), n_(static_cast<Index>(p.poses.size())), edge_pair_(p.edges.size(), -1),
          diag_ws_(n_)
    {
        // Unique pairs (i > j); edge_pair_[k] = pair index of edge k (-1 for self-loops).
        std::vector<std::tuple<Index, Index, Index>> v;
        for (std::size_t k = 0; k < p.edges.size(); ++k)
            if (p.edges[k].a != p.edges[k].b)
                v.emplace_back(std::max(p.edges[k].a, p.edges[k].b),
                               std::min(p.edges[k].a, p.edges[k].b), static_cast<Index>(k));
        std::sort(v.begin(), v.end());
        for (const auto &[i, j, k] : v)
        {
            if (pairs_.empty() || pairs_.back() != std::make_pair(i, j))
                pairs_.emplace_back(i, j);
            edge_pair_[k] = static_cast<Index>(pairs_.size()) - 1;
        }
        off_ws_.resize(pairs_.size());
    }

    fatrop::GraphStructure get_structure() const override
    {
        fatrop::GraphStructure st;
        st.block_sizes.assign(n_, S::nx);
        st.block_sizes_tan.assign(n_, S::dim);
        st.off_diag_edges = pairs_;
        return st;
    }

    Index get_initial(Scalar *x) const override
    {
        for (Index k = 0; k < n_; ++k)
            pose(x, k) = p_.poses[k];
        return 0;
    }

    void apply_retraction(const Scalar *x, const Scalar *dx, const Scalar alpha, Scalar *x_next) override
    {
        for (Index k = 0; k < n_; ++k)
            pose(x_next, k) = S::retract(pose(x, k), alpha * Eigen::Map<const Tangent>(dx + k * S::dim));
    }

    Index eval_f(const Scalar *scale, const Scalar *x, Scalar *res) override
    {
        Tangent r;
        double f = 0.0;
        for (const auto &e : p_.edges)
        {
            S::residual(pose(x, e.a), pose(x, e.b), e.z, r);
            f += 0.5 * r.dot(e.info * r);
        }
        f += 0.5 * prior_weight * S::local(p_.poses[p_.fixed], pose(x, p_.fixed)).squaredNorm();
        *res = scale[0] * f;
        return 0;
    }

    Index eval_gradient(const Scalar *scale, const Scalar *x, Scalar *grad) override
    {
        accumulate<false>(scale[0], x, grad);
        return 0;
    }

    Index eval_hessian(const Scalar *scale, const Scalar *x, const Scalar *,
                       fatrop::GraphHessianView &H) override
    {
        for (auto &D : diag_ws_)
            D.setZero();
        for (auto &O : off_ws_)
            O.setZero();
        accumulate<true>(scale[0], x, H.grad());
        for (Index k = 0; k < n_; ++k)
        {
            if (k == p_.fixed)
                diag_ws_[k].diagonal().array() += prior_weight;
            diag_ws_[k] *= scale[0];
            fatrop::MatRealView Hk = H.diag(k);
            fatrop::pack_mat(S::dim, S::dim, diag_ws_[k].data(), S::dim, Hk, 0, 0);
        }
        for (std::size_t e = 0; e < pairs_.size(); ++e)
        {
            off_ws_[e] *= scale[0];
            fatrop::MatRealView He = H.off(static_cast<Index>(e));
            fatrop::pack_mat(S::dim, S::dim, off_ws_[e].data(), S::dim, He, 0, 0);
        }
        return 0;
    }

private:
    static Eigen::Map<Pose> pose(Scalar *x, Index k) { return Eigen::Map<Pose>(x + k * S::nx); }
    static Eigen::Map<const Pose> pose(const Scalar *x, Index k)
    {
        return Eigen::Map<const Pose>(x + k * S::nx);
    }
    static Eigen::Map<Tangent> seg(Scalar *g, Index k) { return Eigen::Map<Tangent>(g + k * S::dim); }

    // One residual + Jacobian evaluation per edge; gradient (and, with WithHess,
    // unscaled J^T Omega J into the workspaces).
    template <bool WithHess>
    void accumulate(Scalar scale, const Scalar *x, Scalar *grad)
    {
        Tangent r;
        Jac Ja, Jb;
        for (std::size_t k = 0; k < p_.edges.size(); ++k)
        {
            const auto &e = p_.edges[k];
            S::residual(pose(x, e.a), pose(x, e.b), e.z, r, &Ja, &Jb);
            const Tangent Wr = e.info * r;
            if (e.a == e.b)
            {
                const Jac J = Ja + Jb;
                seg(grad, e.a) += J.transpose() * Wr;
                if constexpr (WithHess)
                    diag_ws_[e.a] += J.transpose() * e.info * J;
                continue;
            }
            seg(grad, e.a) += Ja.transpose() * Wr;
            seg(grad, e.b) += Jb.transpose() * Wr;
            if constexpr (WithHess)
            {
                diag_ws_[e.a] += Ja.transpose() * e.info * Ja;
                diag_ws_[e.b] += Jb.transpose() * e.info * Jb;
                off_ws_[edge_pair_[k]] += e.a > e.b ? (Ja.transpose() * e.info * Jb).eval()
                                                    : (Jb.transpose() * e.info * Ja).eval();
            }
        }
        const Tangent d = S::local(p_.poses[p_.fixed], pose(x, p_.fixed));
        Tangent gd = prior_weight * d;
        if constexpr (S::nx == 7)
            gd.template tail<3>() = S::jr_inv(d.template tail<3>()).transpose() * gd.template tail<3>();
        seg(grad, p_.fixed) += gd;
        Eigen::Map<Eigen::VectorXd>(grad, n_ * S::dim) *= scale;
    }

    const Problem<S> &p_;
    const Index n_;
    std::vector<std::pair<Index, Index>> pairs_;
    std::vector<Index> edge_pair_;
    std::vector<Jac> diag_ws_; // unscaled diagonal blocks
    std::vector<Jac> off_ws_;  // unscaled off-diagonal blocks, one per pair
};

template <class S>
std::vector<Eigen::VectorXd> unpack(const fatrop::VecRealView &x, std::size_t n)
{
    Eigen::VectorXd flat(x.m());
    fatrop::unpack_vec(x.m(), x, 0, flat.data(), 1);
    std::vector<Eigen::VectorXd> poses;
    for (std::size_t k = 0; k < n; ++k)
        poses.push_back(flat.segment<S::nx>(k * S::nx));
    return poses;
}

// Records the iterate at the start of every fatrop iteration; throws to stop.
template <class S>
class TracedNlp final : public fatrop::NlpGraph
{
public:
    struct Stop
    {
    };
    TracedNlp(const std::shared_ptr<fatrop::GraphAbstract> &g, Tracer<S> &tracer)
        : fatrop::NlpGraph(g), tracer_(tracer)
    {
    }
    void callback(const fatrop::IpData<fatrop::GraphProblem> &d) override
    {
        if (record(d.iteration_number(), d.current_iterate().primal_x()))
            throw Stop{};
    }
    bool record(int iteration, const fatrop::VecRealView &x)
    {
        return tracer_.record(iteration, [&](double *out) { fatrop::unpack_vec(x.m(), x, 0, out, 1); });
    }

private:
    Tracer<S> &tracer_;
};

const char *flag_name(fatrop::IpSolverReturnFlag f)
{
    using F = fatrop::IpSolverReturnFlag;
    switch (f)
    {
    case F::Success: return "Success";
    case F::MaxIterExceeded: return "MaxIterExceeded";
    case F::StopAtAcceptablePoint: return "StopAtAcceptablePoint";
    case F::LocalInfeasibility: return "LocalInfeasibility";
    case F::LineSearchFailed: return "LineSearchFailed";
    case F::ErrorInStepComputation: return "ErrorInStepComputation";
    default: return "Unknown";
    }
}

template <class S>
Result solve(const Problem<S> &problem, const Options &options)
{
    using namespace fatrop;
    const auto t0 = std::chrono::steady_clock::now();
    Tracer<S> tracer(problem, options);
    auto graph = std::make_shared<PoseGraph<S>>(problem);
    auto nlp = std::make_shared<TracedNlp<S>>(graph, tracer);
    OptionRegistry opts;
    IpAlgBuilder<GraphProblem> builder(nlp);
    auto alg = builder.with_options_registry(&opts).build();
    if (options.max_iterations > 0)
        opts.set_option<Index>("max_iter", options.max_iterations);
    opts.set_option<Index>("print_level", 0);
    opts.set_option<bool>("suppress_banner", true);
    if (options.trace)
    {
        opts.set_option<Index>("max_iter", options.max_iterations > 0 ? options.max_iterations
                                                                      : std::numeric_limits<Index>::max());
        opts.set_option<Scalar>("tolerance", 1e-14);
        opts.set_option<Scalar>("tol_acceptable", 1e-14);
    }
    IpSolverReturnFlag flag = IpSolverReturnFlag::Unknown;
    bool stopped = false;
    try
    {
        flag = alg->optimize();
    }
    catch (const typename TracedNlp<S>::Stop &)
    {
        stopped = true;
    }
    const auto t1 = std::chrono::steady_clock::now();

    Result res;
    res.solver = "fatrop";
    res.time_s = std::chrono::duration<double>(t1 - t0).count();
    res.converged = flag == IpSolverReturnFlag::Success || flag == IpSolverReturnFlag::StopAtAcceptablePoint;
    res.status = stopped ? "trace stop" : flag_name(flag);
    auto ipdata = builder.get_ipdata();
    res.iterations = ipdata->iteration_number();
    const VecRealView &x = ipdata->current_iterate().primal_x();
    if (options.trace && !stopped)
        nlp->record(res.iterations, x);
    res.poses = unpack<S>(x, problem.poses.size());
    res.trace = tracer.take();
    return res;
}

} // namespace

Result solve_fatrop(const Problem<SE2> &problem, const Options &options) { return solve(problem, options); }
Result solve_fatrop(const Problem<SE3> &problem, const Options &options) { return solve(problem, options); }

} // namespace slam
