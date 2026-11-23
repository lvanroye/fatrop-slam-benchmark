#include "solvers/solvers.hpp"

#include <fatrop/fatrop.hpp>
#include <fatrop/graph/graph_abstract.hpp>
#include <fatrop/graph/hessian.hpp>
#include <fatrop/graph/jacobian.hpp>
#include <fatrop/graph/nlp_graph.hpp>
#include <fatrop/ip_algorithm/ip_alg_builder.hpp>
#include <fatrop/ip_algorithm/ip_algorithm.hpp>
#include <fatrop/ip_algorithm/ip_data.hpp>

#include <algorithm>
#include <chrono>
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

// Compressed rows: row r holds items[start[r] .. start[r + 1]).
struct Csr
{
    std::vector<Index> start, items;
    template <class Rows>
    explicit Csr(const Rows &rows)
    {
        start.push_back(0);
        for (const auto &r : rows)
        {
            items.insert(items.end(), r.begin(), r.end());
            start.push_back(static_cast<Index>(items.size()));
        }
    }
    const Index *begin(Index r) const { return items.data() + start[r]; }
    const Index *end(Index r) const { return items.data() + start[r + 1]; }
};

template <class S>
class PoseGraph : public fatrop::GraphAbstract
{
    using Pose = typename S::Pose;
    using Tangent = typename S::Tangent;
    using Jac = typename S::Jac;

public:
    explicit PoseGraph(const Problem<S> &p)
        : p_(p), n_(static_cast<Index>(p.poses.size())), incident_(incident_edges(p)),
          pairs_(build_pairs(p)), neighbors_(neighbor_rows()), pair_edges_(pair_edge_rows(p))
    {
    }

    Index get_num_blocks() const override { return n_; }
    Index get_block_size(Index) const override { return S::nx; }
    Index get_block_size_tan(Index) const override { return S::dim; }
    std::vector<std::pair<Index, Index>> get_off_diag_edges() const override { return pairs_; }

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

    Index eval_grad_k(Index k, const Scalar *scale, const Scalar *x, Scalar *res) override
    {
        Tangent r, g = Tangent::Zero();
        Jac Ja, Jb;
        for (const Index *it = incident_.begin(k); it != incident_.end(k); ++it)
        {
            const auto &e = p_.edges[*it];
            S::residual(pose(x, e.a), pose(x, e.b), e.z, r, &Ja, &Jb);
            g += own_jacobian(e, k, Ja, Jb).transpose() * (e.info * r);
        }
        if (k == p_.fixed)
        {
            const Tangent d = S::local(p_.poses[k], pose(x, k));
            Tangent gd = prior_weight * d;
            if constexpr (S::nx == 7)
                gd.template tail<3>() = S::jr_inv(d.template tail<3>()).transpose() * gd.template tail<3>();
            g += gd;
        }
        Eigen::Map<Tangent> out(res);
        out = scale[0] * g;
        return 0;
    }

    Index eval_Hk(Index i, Index j, const Scalar *scale, const Scalar *x, const Scalar *,
                  MAT *res) override
    {
        Tangent r;
        Jac Ja, Jb, H = Jac::Zero();
        if (i == j)
        {
            for (const Index *it = incident_.begin(i); it != incident_.end(i); ++it)
            {
                const auto &e = p_.edges[*it];
                S::residual(pose(x, e.a), pose(x, e.b), e.z, r, &Ja, &Jb);
                const Jac J = own_jacobian(e, i, Ja, Jb);
                H += J.transpose() * e.info * J;
            }
            if (i == p_.fixed)
                H.diagonal().array() += prior_weight;
        }
        else
        {
            const Index *nb = std::lower_bound(neighbors_.begin(j), neighbors_.end(j), i);
            const Index pair = static_cast<Index>(nb - neighbors_.items.data());
            for (const Index *it = pair_edges_.begin(pair); it != pair_edges_.end(pair); ++it)
            {
                const auto &e = p_.edges[*it];
                S::residual(pose(x, e.a), pose(x, e.b), e.z, r, &Ja, &Jb);
                H += e.a == i ? (Ja.transpose() * e.info * Jb).eval() : (Jb.transpose() * e.info * Ja).eval();
            }
        }
        H *= scale[0];
        fatrop::blasfeo_pack_mat_wrap(S::dim, S::dim, H.data(), S::dim, res, 0, 0);
        return 0;
    }

private:
    static Eigen::Map<Pose> pose(Scalar *x, Index k) { return Eigen::Map<Pose>(x + k * S::nx); }
    static Eigen::Map<const Pose> pose(const Scalar *x, Index k)
    {
        return Eigen::Map<const Pose>(x + k * S::nx);
    }

    static Jac own_jacobian(const Edge<S> &e, Index k, const Jac &Ja, const Jac &Jb)
    {
        return e.a == e.b ? (Ja + Jb).eval() : e.a == k ? Ja : Jb;
    }

    static Csr incident_edges(const Problem<S> &p)
    {
        std::vector<std::vector<Index>> rows(p.poses.size());
        for (std::size_t k = 0; k < p.edges.size(); ++k)
        {
            rows[p.edges[k].a].push_back(static_cast<Index>(k));
            if (p.edges[k].b != p.edges[k].a)
                rows[p.edges[k].b].push_back(static_cast<Index>(k));
        }
        return Csr(rows);
    }

    // Unordered pairs (i > j), sorted by (j, i), so that pair index = position in neighbors_.
    static std::vector<std::pair<Index, Index>> build_pairs(const Problem<S> &p)
    {
        std::vector<std::pair<Index, Index>> v;
        for (const auto &e : p.edges)
            if (e.a != e.b)
                v.emplace_back(std::max(e.a, e.b), std::min(e.a, e.b));
        std::sort(v.begin(), v.end(), [](auto &l, auto &r) {
            return std::tie(l.second, l.first) < std::tie(r.second, r.first);
        });
        v.erase(std::unique(v.begin(), v.end()), v.end());
        return v;
    }

    Csr neighbor_rows() const
    {
        std::vector<std::vector<Index>> rows(n_);
        for (const auto &[i, j] : pairs_)
            rows[j].push_back(i);
        return Csr(rows);
    }

    Csr pair_edge_rows(const Problem<S> &p) const
    {
        std::vector<std::vector<Index>> rows(pairs_.size());
        for (std::size_t k = 0; k < p.edges.size(); ++k)
        {
            const auto &e = p.edges[k];
            if (e.a == e.b)
                continue;
            const Index i = std::max(e.a, e.b), j = std::min(e.a, e.b);
            const Index *nb = std::lower_bound(neighbors_.begin(j), neighbors_.end(j), i);
            rows[nb - neighbors_.items.data()].push_back(static_cast<Index>(k));
        }
        return Csr(rows);
    }

    const Problem<S> &p_;
    const Index n_;
    const Csr incident_;
    const std::vector<std::pair<Index, Index>> pairs_;
    const Csr neighbors_;  // row j: sorted i > j paired with j; flat position = pair index
    const Csr pair_edges_; // row pair: edges contributing to that Hessian block
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
    auto graph = std::make_shared<PoseGraph<S>>(problem);
    auto nlp = std::make_shared<NlpGraph>(graph);
    OptionRegistry opts;
    IpAlgBuilder<GraphProblem> builder(nlp);
    auto alg = builder.with_options_registry(&opts).build();
    opts.set_option<Index>("max_iter", options.max_iterations);
    opts.set_option<Index>("print_level", 0);
    opts.set_option<bool>("suppress_banner", true);
    const IpSolverReturnFlag flag = alg->optimize();
    const auto t1 = std::chrono::steady_clock::now();

    Result res;
    res.solver = "fatrop";
    res.time_s = std::chrono::duration<double>(t1 - t0).count();
    res.converged = flag == IpSolverReturnFlag::Success || flag == IpSolverReturnFlag::StopAtAcceptablePoint;
    res.status = flag_name(flag);
    auto ipdata = builder.get_ipdata();
    res.iterations = ipdata->iteration_number();
    const VecRealView &x = ipdata->current_iterate().primal_x();
    Eigen::VectorXd flat(x.m());
    unpack_vec(x.m(), x, 0, flat.data(), 1);
    for (std::size_t k = 0; k < problem.poses.size(); ++k)
        res.poses.push_back(flat.segment<S::nx>(k * S::nx));
    return res;
}

} // namespace

Result solve_fatrop(const Problem<SE2> &problem, const Options &options) { return solve(problem, options); }
Result solve_fatrop(const Problem<SE3> &problem, const Options &options) { return solve(problem, options); }

} // namespace slam
