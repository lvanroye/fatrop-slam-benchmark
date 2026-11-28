// Solver-neutral pose-graph problem, .g2o loader, and the shared metrics.
#pragma once

#include "lie.hpp"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <fstream>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace slam
{

template <class S>
struct Edge
{
    int a, b;
    typename S::Pose z;                               // relative measurement a -> b
    Eigen::Matrix<double, S::dim, S::dim> info;       // Omega / m (see load_g2o)
    Eigen::Matrix<double, S::dim, S::dim> sqrt_info;  // upper-triangular U, U^T U = Omega
};

template <class S>
struct Problem
{
    std::string name;
    std::vector<typename S::Pose> poses; // initial guess
    std::vector<Edge<S>> edges;
    int fixed = 0;           // gauge: this pose is held at its initial value
    int n_repaired_info = 0; // edges whose Omega was not positive definite
};

struct Options
{
    int max_iterations = 0; // 0: keep the solver's default
    // Trace mode: record the time and a copy of every iterate, and replace
    // the solver's own stopping rule by the iteration cap, time_limit, or
    // (if grad_target > 0) shared gradient norm <= grad_target; the solver
    // still stops when it can make no more progress.
    bool trace = false;
    double grad_target = 0.0; // > 0: probe run, checks the gradient every iterate
    double time_limit = 600.0;
};

struct Iterate
{
    int iteration;
    double time_s;         // excludes the time spent recording
    std::vector<double> x; // poses, S::nx each
};

struct Result
{
    std::string solver;
    bool converged = false;
    int iterations = 0;
    double time_s = 0.0;
    std::string status;
    std::vector<Eigen::VectorXd> poses; // final poses, S::nx each
    std::vector<Iterate> trace;         // trace mode only
};

// Information matrices of a few public datasets are not positive definite.
// Clamp the eigenvalues so that every solver sees the same convex edge cost.
template <int D>
bool repair_info(Eigen::Matrix<double, D, D> &info)
{
    info = 0.5 * (info + info.transpose()).eval();
    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<double, D, D>> es(info);
    const double floor = 1e-9 * std::max(es.eigenvalues().maxCoeff(), 1.0);
    if (es.eigenvalues().minCoeff() >= floor)
        return false;
    const auto l = es.eigenvalues().cwiseMax(floor);
    info = es.eigenvectors() * l.asDiagonal() * es.eigenvectors().transpose();
    return true;
}

template <class S>
typename S::Pose read_pose(std::istringstream &ss)
{
    typename S::Pose x;
    for (int i = 0; i < S::nx; ++i)
        ss >> x[i];
    if constexpr (S::nx == 7)
        x.template tail<4>().normalize();
    return x;
}

// Reads VERTEX_SE2/EDGE_SE2 or VERTEX_SE3:QUAT/EDGE_SE3:QUAT files.
// Files without vertices get a dead-reckoning initial guess along the
// odometry chain (b == a + 1), which is what g2o itself would do.
template <class S>
Problem<S> load_g2o(const std::string &path)
{
    std::ifstream f(path);
    if (!f)
        throw std::runtime_error("cannot open " + path);
    const std::string vtag = S::nx == 3 ? "VERTEX_SE2" : "VERTEX_SE3:QUAT";
    const std::string etag = S::nx == 3 ? "EDGE_SE2" : "EDGE_SE3:QUAT";

    Problem<S> p;
    p.name = path.substr(path.find_last_of('/') + 1);
    std::map<int, typename S::Pose> verts;
    std::vector<std::pair<int, int>> ids;
    int fix_id = -1;
    std::string line, tag;
    while (std::getline(f, line))
    {
        std::istringstream ss(line);
        if (!(ss >> tag))
            continue;
        if (tag == vtag)
        {
            int id;
            ss >> id;
            verts[id] = read_pose<S>(ss);
        }
        else if (tag == etag)
        {
            Edge<S> e;
            int ia, ib;
            ss >> ia >> ib;
            e.z = read_pose<S>(ss);
            for (int i = 0; i < S::dim; ++i)
                for (int j = i; j < S::dim; ++j)
                {
                    ss >> e.info(i, j);
                    e.info(j, i) = e.info(i, j);
                }
            ids.emplace_back(ia, ib);
            p.edges.push_back(e);
        }
        else if (tag == "FIX")
            ss >> fix_id;
    }

    // Dead reckoning for vertices missing from the file.
    for (std::size_t k = 0; k < ids.size(); ++k)
    {
        const auto [ia, ib] = ids[k];
        if (ia == 0 && !verts.count(ia))
            verts[ia] = S::identity();
        if (ib == ia + 1 && verts.count(ia) && !verts.count(ib))
            verts[ib] = S::compose(verts[ia], p.edges[k].z);
    }

    std::map<int, int> index;
    for (const auto &[id, x] : verts)
    {
        index[id] = static_cast<int>(p.poses.size());
        p.poses.push_back(x);
    }
    // Mean over the scalar residuals: Omega / m with m = n_edges * dim.
    const double m = static_cast<double>(ids.size() * S::dim);
    for (std::size_t k = 0; k < ids.size(); ++k)
    {
        auto &e = p.edges[k];
        if (!index.count(ids[k].first) || !index.count(ids[k].second))
            throw std::runtime_error("edge references an unknown vertex in " + path);
        e.a = index[ids[k].first];
        e.b = index[ids[k].second];
        p.n_repaired_info += repair_info<S::dim>(e.info);
        e.info /= m;
        e.sqrt_info = e.info.llt().matrixU();
    }
    p.fixed = (fix_id >= 0 && index.count(fix_id)) ? index[fix_id] : 0;
    return p;
}

// 0 = not a pose graph, 2 = SE(2), 3 = SE(3).
inline int detect_dimension(const std::string &path)
{
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line))
    {
        if (line.rfind("VERTEX_SE2", 0) == 0 || line.rfind("EDGE_SE2", 0) == 0)
            return 2;
        if (line.rfind("VERTEX_SE3:QUAT", 0) == 0 || line.rfind("EDGE_SE3:QUAT", 0) == 0)
            return 3;
    }
    return 0;
}

template <class S>
typename S::Pose pose_of(const Eigen::VectorXd &x)
{
    return x.head<S::nx>();
}

// Objective 0.5 * sum_e r^T Omega r / m at the given poses.
template <class S>
double objective(const Problem<S> &p, const std::vector<Eigen::VectorXd> &x)
{
    double f = 0.0;
    typename S::Tangent r;
    for (const auto &e : p.edges)
    {
        S::residual(pose_of<S>(x[e.a]), pose_of<S>(x[e.b]), e.z, r);
        f += 0.5 * r.dot(e.info * r);
    }
    return f;
}

// Optimality measure: max-norm of the objective gradient in the tangent
// space, over all poses except the gauge-fixed one.
template <class S>
double gradient_norm(const Problem<S> &p, const std::vector<Eigen::VectorXd> &x)
{
    std::vector<typename S::Tangent> g(p.poses.size(), S::Tangent::Zero());
    typename S::Tangent r;
    typename S::Jac Ja, Jb;
    for (const auto &e : p.edges)
    {
        S::residual(pose_of<S>(x[e.a]), pose_of<S>(x[e.b]), e.z, r, &Ja, &Jb);
        const typename S::Tangent w = e.info * r;
        g[e.a] += Ja.transpose() * w;
        g[e.b] += Jb.transpose() * w;
    }
    double n = 0.0;
    for (std::size_t k = 0; k < g.size(); ++k)
        if (static_cast<int>(k) != p.fixed)
            n = std::max(n, g[k].template lpNorm<Eigen::Infinity>());
    return n;
}

} // namespace slam
