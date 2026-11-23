// SE(2) and SE(3) pose spaces shared by every solver.
//
// Each space defines the pose storage, the retraction x (+) d, and the
// edge residual with its analytic Jacobians in that tangent space.
// Every solver uses exactly these, so they all minimise the same objective
//     f(x) = 0.5 * sum_e r_e(x)^T Omega_e r_e(x).
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cmath>

namespace slam
{

inline double wrap_angle(double a) { return std::atan2(std::sin(a), std::cos(a)); }

// Pose: (x, y, theta). Tangent: (dx, dy, dtheta), additive.
// Residual: r_t = R(th_a)^T (t_b - t_a) - z_t,  r_th = wrap(th_b - th_a - z_th).
struct SE2
{
    static constexpr int nx = 3;  // storage size
    static constexpr int dim = 3; // tangent size
    static constexpr const char *name = "SE2";
    using Pose = Eigen::Matrix<double, nx, 1>;
    using Tangent = Eigen::Matrix<double, dim, 1>;
    using Jac = Eigen::Matrix<double, dim, dim>;
    using PlusJac = Eigen::Matrix<double, nx, dim>;

    static Pose identity() { return Pose::Zero(); }

    static Pose retract(const Pose &x, const Tangent &d) { return x + d; }

    // d(x (+) d)/dd at d = 0.
    static PlusJac plus_jacobian(const Pose &) { return PlusJac::Identity(); }

    // Local coordinates of x w.r.t. x0 (inverse of retract).
    static Tangent local(const Pose &x0, const Pose &x)
    {
        Tangent d = x - x0;
        d[2] = wrap_angle(d[2]);
        return d;
    }

    // Pose b expressed from a and the relative measurement z (dead reckoning).
    static Pose compose(const Pose &a, const Pose &z)
    {
        const double c = std::cos(a[2]), s = std::sin(a[2]);
        return Pose(a[0] + c * z[0] - s * z[1], a[1] + s * z[0] + c * z[1], wrap_angle(a[2] + z[2]));
    }

    static void residual(const Pose &a, const Pose &b, const Pose &z, Tangent &r, Jac *Ja = nullptr,
                         Jac *Jb = nullptr)
    {
        const double c = std::cos(a[2]), s = std::sin(a[2]);
        const double dx = b[0] - a[0], dy = b[1] - a[1];
        r[0] = c * dx + s * dy - z[0];
        r[1] = -s * dx + c * dy - z[1];
        r[2] = wrap_angle(b[2] - a[2] - z[2]);
        if (Ja)
            *Ja << -c, -s, -s * dx + c * dy,
                    s, -c, -c * dx - s * dy,
                    0, 0, -1;
        if (Jb)
            *Jb << c, s, 0,
                  -s, c, 0,
                   0, 0, 1;
    }
};

// Pose: (px, py, pz, qx, qy, qz, qw), the .g2o and Eigen coefficient order.
// Tangent: (dp, dphi) with world-frame translation and body-frame rotation:
//     p (+) d = p + dp,   q (+) d = q * Exp(dphi).
// Residual: r_t = R_a^T (p_b - p_a) - z_t,  r_phi = Log(z_q^-1 q_a^-1 q_b).
struct SE3
{
    static constexpr int nx = 7;
    static constexpr int dim = 6;
    static constexpr const char *name = "SE3";
    using Pose = Eigen::Matrix<double, nx, 1>;
    using Tangent = Eigen::Matrix<double, dim, 1>;
    using Jac = Eigen::Matrix<double, dim, dim>;
    using PlusJac = Eigen::Matrix<double, nx, dim>;
    using Mat3 = Eigen::Matrix3d;
    using Vec3 = Eigen::Vector3d;
    using Quat = Eigen::Quaterniond;

    static Quat quat(const Pose &x) { return Quat(x[6], x[3], x[4], x[5]); }

    static Pose make(const Vec3 &p, const Quat &q)
    {
        Pose x;
        x << p, q.x(), q.y(), q.z(), q.w();
        return x;
    }

    static Mat3 hat(const Vec3 &v)
    {
        Mat3 S;
        S << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
        return S;
    }

    static Quat exp(const Vec3 &phi)
    {
        const double th = phi.norm();
        if (th < 1e-10)
            return Quat(1.0, 0.5 * phi.x(), 0.5 * phi.y(), 0.5 * phi.z()).normalized();
        const Vec3 v = std::sin(0.5 * th) / th * phi;
        return Quat(std::cos(0.5 * th), v.x(), v.y(), v.z());
    }

    // Shortest-path log, in [0, pi].
    static Vec3 log(Quat q)
    {
        if (q.w() < 0)
            q.coeffs() = -q.coeffs();
        const Vec3 v = q.vec();
        const double s = v.norm();
        if (s < 1e-10)
            return 2.0 * v / q.w();
        return 2.0 * std::atan2(s, q.w()) / s * v;
    }

    // Inverse right Jacobian of SO(3).
    static Mat3 jr_inv(const Vec3 &phi)
    {
        const double th2 = phi.squaredNorm();
        double c;
        if (th2 < 1e-8)
            c = 1.0 / 12.0 + th2 / 720.0;
        else
        {
            const double th = std::sqrt(th2);
            c = 1.0 / th2 - std::cos(0.5 * th) / (2.0 * th * std::sin(0.5 * th));
        }
        const Mat3 P = hat(phi);
        return Mat3::Identity() + 0.5 * P + c * P * P;
    }

    static Pose identity() { return make(Vec3::Zero(), Quat::Identity()); }

    static Pose retract(const Pose &x, const Tangent &d)
    {
        return make(x.head<3>() + d.head<3>(), (quat(x) * exp(d.tail<3>())).normalized());
    }

    // d(x (+) d)/dd at d = 0: identity on p, 0.5 * q * (0, dphi) on q.
    static PlusJac plus_jacobian(const Pose &x)
    {
        const double qx = x[3], qy = x[4], qz = x[5], qw = x[6];
        PlusJac P = PlusJac::Zero();
        P.topLeftCorner<3, 3>().setIdentity();
        P.bottomRightCorner<4, 3>() << qw, -qz, qy,
                                       qz, qw, -qx,
                                      -qy, qx, qw,
                                      -qx, -qy, -qz;
        P.bottomRightCorner<4, 3>() *= 0.5;
        return P;
    }

    static Tangent local(const Pose &x0, const Pose &x)
    {
        Tangent d;
        d << x.head<3>() - x0.head<3>(), log(quat(x0).conjugate() * quat(x));
        return d;
    }

    static Pose compose(const Pose &a, const Pose &z)
    {
        const Quat qa = quat(a);
        return make(a.head<3>() + qa * z.head<3>(), (qa * quat(z)).normalized());
    }

    static void residual(const Pose &a, const Pose &b, const Pose &z, Tangent &r, Jac *Ja = nullptr,
                         Jac *Jb = nullptr)
    {
        const Quat qa = quat(a), qb = quat(b), qz = quat(z);
        const Mat3 RaT = qa.toRotationMatrix().transpose();
        const Vec3 dp = RaT * (b.head<3>() - a.head<3>());
        const Vec3 phi = log(qz.conjugate() * qa.conjugate() * qb);
        r << dp - z.head<3>(), phi;
        if (!Ja && !Jb)
            return;
        const Mat3 Jri = jr_inv(phi);
        if (Ja)
        {
            Ja->setZero();
            Ja->topLeftCorner<3, 3>() = -RaT;
            Ja->topRightCorner<3, 3>() = hat(dp);
            Ja->bottomRightCorner<3, 3>() = -Jri.transpose() * qz.toRotationMatrix().transpose();
        }
        if (Jb)
        {
            Jb->setZero();
            Jb->topLeftCorner<3, 3>() = RaT;
            Jb->bottomRightCorner<3, 3>() = Jri;
        }
    }
};

} // namespace slam
