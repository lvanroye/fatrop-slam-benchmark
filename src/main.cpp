// slam_benchmark <file.g2o> [--max-iterations N] [--solvers a,b,...]
//                [--trace out.csv] [--grad-target G] [--time-limit S]
//
// Runs every compiled-in solver on one pose graph and prints one line per
// solver: iterations, wall time, objective and gradient norm at the
// returned poses. With --trace, every solver runs in trace mode (see
// Options::trace) and appends its iterates to out.csv as
// dataset,solver,iteration,time_s,objective,grad_inf.
#include "solvers/solvers.hpp"

#include <cstdio>
#include <cstring>
#include <fstream>
#include <functional>
#include <iostream>
#include <string>
#include <vector>

using namespace slam;

template <class S>
using SolveFn = std::function<Result(const Problem<S> &, const Options &)>;

template <class S>
std::vector<std::pair<std::string, SolveFn<S>>> solvers()
{
    std::vector<std::pair<std::string, SolveFn<S>>> v;
#ifdef SLAM_WITH_FATROP
    v.emplace_back("fatrop", [](const Problem<S> &p, const Options &o) { return solve_fatrop(p, o); });
#endif
#ifdef SLAM_WITH_CERES
    v.emplace_back("ceres", [](const Problem<S> &p, const Options &o) { return solve_ceres(p, o); });
#endif
#ifdef SLAM_WITH_G2O
    v.emplace_back("g2o", [](const Problem<S> &p, const Options &o) { return solve_g2o(p, o); });
#endif
#ifdef SLAM_CERES_CHOLMOD
    v.emplace_back("ceres-cholmod", [](const Problem<S> &p, const Options &o) { return solve_ceres_cholmod(p, o); });
    v.emplace_back("ceres-cholmod-nd", [](const Problem<S> &p, const Options &o) { return solve_ceres_cholmod_nd(p, o); });
#endif
#ifdef SLAM_G2O_CHOLMOD
    v.emplace_back("g2o-cholmod", [](const Problem<S> &p, const Options &o) { return solve_g2o_cholmod(p, o); });
#endif
#ifdef SLAM_WITH_GTSAM
    v.emplace_back("gtsam", [](const Problem<S> &p, const Options &o) { return solve_gtsam(p, o); });
#endif
    return v;
}

template <class S>
int run(const std::string &path, const Options &opt, const std::string &only, const std::string &trace)
{
    const Problem<S> p = load_g2o<S>(path);
    std::vector<Eigen::VectorXd> x0(p.poses.begin(), p.poses.end());
    std::printf("dataset %s  %s  poses %zu  edges %zu  repaired_info %d\n", p.name.c_str(), S::name,
                p.poses.size(), p.edges.size(), p.n_repaired_info);
    std::printf("%-16s %5s %10s %16s %12s  %s\n", "solver", "iter", "time_s", "objective", "grad_inf",
                "status");
    std::printf("%-16s %5s %10s %16.8e %12.3e\n", "initial", "-", "-", objective(p, x0),
                gradient_norm(p, x0));
    for (const auto &[name, solve] : solvers<S>())
    {
        if (!only.empty() && ("," + only + ",").find("," + name + ",") == std::string::npos)
            continue;
        Result r;
        try
        {
            r = solve(p, opt);
        }
        catch (const std::exception &e)
        {
            std::printf("%-16s %5s %10s %16s %12s  error: %s\n", name.c_str(), "-", "-", "-", "-", e.what());
            continue;
        }
        std::printf("%-16s %5d %10.4f %16.8e %12.3e  %s%s\n", name.c_str(), r.iterations, r.time_s,
                    objective(p, r.poses), gradient_norm(p, r.poses), r.converged ? "converged" : "not converged",
                    r.status.empty() ? "" : (" (" + r.status + ")").c_str());
        std::fflush(stdout);
        if (!trace.empty())
        {
            std::ofstream out(trace, std::ios::app);
            out.precision(10);
            for (const auto &t : r.trace) // metrics evaluated after the solve, outside its timing
            {
                const auto x = poses_of<S>(t.x);
                out << p.name << ',' << name << ',' << t.iteration << ',' << t.time_s << ',' << objective(p, x)
                    << ',' << gradient_norm(p, x) << '\n';
            }
        }
    }
    return 0;
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "usage: " << argv[0] << " <file.g2o> [--max-iterations N] [--solvers a,b]\n";
        return 1;
    }
    Options opt;
    std::string only, trace;
    for (int i = 2; i + 1 < argc; i += 2)
    {
        if (!std::strcmp(argv[i], "--max-iterations"))
            opt.max_iterations = std::stoi(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--solvers"))
            only = argv[i + 1];
        else if (!std::strcmp(argv[i], "--trace"))
            trace = argv[i + 1], opt.trace = true;
        else if (!std::strcmp(argv[i], "--grad-target"))
            opt.grad_target = std::stod(argv[i + 1]);
        else if (!std::strcmp(argv[i], "--time-limit"))
            opt.time_limit = std::stod(argv[i + 1]);
    }
    switch (detect_dimension(argv[1]))
    {
    case 2: return run<SE2>(argv[1], opt, only, trace);
    case 3: return run<SE3>(argv[1], opt, only, trace);
    default: std::cerr << "not an SE2/SE3 pose graph: " << argv[1] << "\n"; return 1;
    }
}
