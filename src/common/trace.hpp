// Per-iterate recording for trace mode (Options::trace).
#pragma once

#include "problem.hpp"

#include <chrono>

namespace slam
{

template <class S>
std::vector<Eigen::VectorXd> poses_of(const std::vector<double> &x)
{
    std::vector<Eigen::VectorXd> v;
    for (std::size_t k = 0; k < x.size(); k += S::nx)
        v.emplace_back(Eigen::Map<const Eigen::VectorXd>(x.data() + k, S::nx));
    return v;
}

template <class S>
class Tracer
{
    using clock = std::chrono::steady_clock;

public:
    Tracer(const Problem<S> &p, const Options &o) : p_(p), o_(o), t0_(clock::now())
    {
        if (o.trace && o.max_iterations > 0)
            trace_.reserve(o.max_iterations + 2);
    }

    // Wall time since construction, minus the time spent in record().
    double elapsed() const { return seconds(clock::now() - t0_) - paused_; }

    // Store the time and the iterate written by fill(double *x); true when
    // the solver should stop. Only a probe run evaluates the gradient here.
    template <class F>
    bool record(int iteration, F &&fill)
    {
        const double t = elapsed();
        const auto r0 = clock::now();
        trace_.push_back({iteration, t, std::vector<double>(p_.poses.size() * S::nx)});
        fill(trace_.back().x.data());
        const bool stop = t >= o_.time_limit || (o_.max_iterations > 0 && iteration >= o_.max_iterations) ||
                          (o_.grad_target > 0 && gradient_norm(p_, poses_of<S>(trace_.back().x)) <= o_.grad_target);
        paused_ += seconds(clock::now() - r0);
        return stop;
    }

    std::vector<Iterate> take() { return std::move(trace_); }

private:
    static double seconds(clock::duration d) { return std::chrono::duration<double>(d).count(); }

    const Problem<S> &p_;
    const Options &o_;
    const clock::time_point t0_;
    double paused_ = 0.0;
    std::vector<Iterate> trace_;
};

} // namespace slam
