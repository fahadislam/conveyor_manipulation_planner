#include "profile.h"

// standard includes
#include <cmath>
#include <cstdlib>
#include <algorithm>
#include <iostream>
#include <utility>

// system includes
#include <Eigen/Dense>

namespace smpl {

auto ComputeCubicProfile(real_t qi, real_t qf, real_t vi, real_t vf, real_t tf)
    -> CubicProfile
{
    auto a0 = qi;
    auto a1 = vi;

    // (tf^3) * a3 + tf^2 * a2 = qf - a0 - a1 * tf
    // (3 * tf^2) * a3 + (2 * tf) * a2 = vf - a1
    //

    auto tf2 = tf * tf;
    auto tf3 = tf2 * tf;

    Eigen::Matrix2d A;
    Eigen::Vector2d b;
    A(0,0) = tf3;
    A(0,1) = tf2;
    A(1,0) = real_t(3) * tf2;
    A(1,1) = real_t(2) * tf;

    b(0) = qf - a0 - a1 * tf;
    b(1) = vf - a1;

    Eigen::Vector2d x;
    x = A.colPivHouseholderQr().solve(b);

    CubicProfile profile;
    profile.a[0] = x(0);
    profile.a[1] = x(1);
    profile.a[2] = a1;
    profile.a[3] = a0;
    return profile;
}

auto pos(CubicProfile profile, real_t x) -> real_t
{
    auto ret = real_t(profile.a[0]);
    ret *= x;
    ret += profile.a[1];
    ret *= x;
    ret += profile.a[2];
    ret *= x;
    ret += profile.a[3];
    return ret;
}

auto vel(CubicProfile profile, real_t x) -> real_t
{
    auto ret = real_t(3) * profile.a[0];
    ret *= x;
    ret += real_t(2) * profile.a[1];
    ret *= x;
    ret += profile.a[2];
    return ret;
}

auto acc(CubicProfile profile, real_t x) -> real_t
{
    auto ret = real_t(6) * profile.a[0];
    ret *= x;
    ret += real_t(2) * profile.a[1];
    return ret;
}

auto ComputeQuinticProfile(
    real_t qi, real_t qf,
    real_t vi, real_t vf,
    real_t ai, real_t af,
    real_t tf)
    -> QuinticProfile
{
    auto a2 = real_t(0.5) * ai;
    auto a1 = vi;
    auto a0 = qi;

    auto tf2 = tf * tf;
    auto tf3 = tf2 * tf;
    auto tf4 = tf3 * tf;
    auto tf5 = tf4 * tf;

    Eigen::Matrix3d A;
    A(0,0) = tf5;
    A(0,1) = tf4;
    A(0,2) = tf3;
    A(1,0) = real_t(5) * tf4;
    A(1,1) = real_t(4) * tf3;
    A(1,2) = real_t(3) * tf2;
    A(2,0) = real_t(20) * tf3;
    A(2,1) = real_t(12) * tf2;
    A(2,2) = real_t(6) * tf;

    Eigen::Vector3d b;
    b(0) = qf - real_t(0.5) * ai * tf2 - vi * tf - qi;
    b(1) = vf - ai * tf - vi;
    b(2) = af - ai;

    Eigen::Vector3d x;
    x = A.colPivHouseholderQr().solve(b);

    QuinticProfile profile;

    profile.a[0] = x(0);
    profile.a[1] = x(1);
    profile.a[2] = x(2);
    profile.a[3] = a2;
    profile.a[4] = a1;
    profile.a[5] = a0;

    return profile;
}

auto pos(QuinticProfile profile, real_t x) -> real_t
{
    auto ret = profile.a[0];
    ret *= x;
    ret += profile.a[1];
    ret *= x;
    ret += profile.a[2];
    ret *= x;
    ret += profile.a[3];
    ret *= x;
    ret += profile.a[4];
    ret *= x;
    ret += profile.a[5];
    return ret;
}

auto vel(QuinticProfile profile, real_t x) -> real_t
{
    auto ret = real_t(5) * profile.a[0];
    ret *= x;
    ret += real_t(4) * profile.a[1];
    ret *= x;
    ret += real_t(3) * profile.a[2];
    ret *= x;
    ret += real_t(2) * profile.a[3];
    ret *= x;
    ret += profile.a[4];
    return ret;
}

auto acc(QuinticProfile profile, real_t x) -> real_t
{
    auto ret = real_t(20) * profile.a[0];
    ret *= x;
    ret += real_t(12) * profile.a[1];
    ret *= x;
    ret += real_t(6) * profile.a[2];
    ret *= x;
    ret += profile.a[3];
    return ret;
}

template <class T>
T sign(T val) {
    if (val > T(0)) {
        return T(1);
    } else if (val < T(0)) {
        return T(-1);
    } else {
        return T(0);
    }
}

template <class T>
T sqrd(T val) { return val * val; }

auto GetTriangularProfileAcceleration(real_t qi, real_t qf, real_t tf) -> real_t
{
    return (real_t(4) * std::fabs(qf - qi)) / (tf * tf);
}

auto ComputeTrapezoidalProfileAcc(real_t qi, real_t qf, real_t ac, real_t tf)
    -> TrapezoidalProfile
{
    ac = std::copysign(ac, qf - qi);

    auto tmp = (tf * tf * ac - real_t(4) * (qf - qi)) / ac;
    if (tmp < 0.0) {
        fprintf(stderr, "Accelerate!\n");
        // TODO: acceleration not fast enough to make it there in time
        // TODO: Any issue here with using the result of
        // GetTriangularProfileAcceleration directly and ending up with
        // precision issues here?
    }

    auto tc = real_t(0.5) * tf - real_t(0.5) * std::sqrt(tmp);

    TrapezoidalProfile profile;
    profile.qi = qi;
    profile.qf = qf;
    profile.ac = ac;
    profile.tc = tc;
    profile.tf = tf;
    return profile;
}

auto ComputeTrapezoidalProfileVel(real_t qi, real_t qf, real_t vc, real_t tf)
    -> TrapezoidalProfile
{
    auto min_vc = std::fabs(qf - qi) / tf;
    auto max_vc = real_t(2) * min_vc;

    // TODO: errors
    if (std::fabs(vc) <= min_vc) {
        fprintf(stderr, "Speed up\n!");
    }
    if (std::fabs(vc) >= max_vc) {
        fprintf(stderr, "Slow down!\n");
    }

    auto tc = (qi - qf + vc * tf) / vc;

    auto ac = vc * vc / (qi - qf + vc * tf);

    TrapezoidalProfile profile;
    profile.qi = qi;
    profile.qf = qf;
    profile.ac = ac;
    profile.tc = tc;
    profile.tf = tf;
    return profile;
}

auto pos(TrapezoidalProfile profile, real_t x) -> real_t
{
    if (x <= profile.tc) {
        return profile.qi + real_t(0.5) * profile.ac * x * x;
    } else if (x >= profile.tf - profile.tc) {
        return profile.qf - real_t(0.5) * sqrd(profile.tf - x);
    } else {
        return profile.qi + profile.ac * profile.tc * (x - real_t(0.5) * profile.tc);
    }
}

auto vel(TrapezoidalProfile profile, real_t x) -> real_t
{
    if (x <= profile.tc) {
        return profile.ac * x;
    } else if (x >= profile.tf - profile.tc) {
        return profile.ac * profile.tc - (profile.ac * (x - (profile.tf - profile.tc)));
    } else {
        return profile.ac * profile.tc;
    }
}

auto acc(TrapezoidalProfile profile, real_t x) -> real_t
{
    if (x <= profile.tc) {
        return profile.ac;
    } else if (x >= profile.tf - profile.tc) {
        return -profile.ac;
    } else {
        return 0.0;
    }
}

auto ComputeCubicPathProfile(real_t* qs, real_t* vs, real_t* ts, int count)
    -> CubicPathProfile
{
    CubicPathProfile profile;

    profile.profiles.resize(count - 1);
    for (int i = 0; i < count - 1; ++i) {
        profile.profiles[i] = ComputeCubicProfile(
                qs[i], qs[i + 1], vs[i], vs[i + 1], ts[i + 1] - ts[i]);
    }

    profile.times.assign(ts, ts + count);

    return profile;
}

auto ComputeCubicPathProfile(real_t* qs, real_t* ts, int count)
    -> CubicPathProfile
{
    std::vector<real_t> vs(count);

    vs.front() = real_t(0);
    vs.back() = real_t(0);

    for (int i = 1; i < count - 1; ++i) {
        vs[i] = (qs[i] - qs[i - 1]) / (ts[i] - ts[i - 1]);
    }
    for (int i = 1; i < count - 1; ++i) {
        vs[i] = 0.5 * (vs[i] + vs[i + 1]);
    }

    return ComputeCubicPathProfile(qs, vs.data(), ts, count);
}

auto ComputeCubicPathProfileCont(real_t* qs, real_t* ts, int count)
    -> CubicPathProfile
{
    // inputs:
    // [ t1, t2, ..., tn ]
    // [ q1, q2, ..., qn ]
    // [ v1, v2, ..., vn ]

    // polynomial equations:
    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t^3, t^2, t^1, 1 ] = q(t), t1 <= t < t2
    // [ a3_2, a2_2, a1_2, a0_2 ] * [ t^3, t^2, t^1, 1 ] = q(t), t2 <= t < t3
    // ...
    // [ a3_(n-1), a2_(n-1), a1_(n-1), a0_(n-1) ] * [ t^3, t^2, t^1, 1 ] = q(t), t(n-1) <= t <= tn

    // first set of 4 constraints:
    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t2^3, t2^2, t2, 1 ] = q2
    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t2^3, t2^2, t2, 1 ] = [ a3_2, a2_2, a1_2, a0_1 ] * [ t2^3, t2^2, t2, 1 ]
    // [ 3 * a3_1, 2 * a2_1, a1_1 ] * [ t2^2, t2, 1 ] = [ 3 * a3_2, 2 * a2_2, a2_1 ] * [ t2^2, t2, 1 ]
    // [ 6 * a3_1, 2 * a2_1 ] * [ t2, 1 ] = [ 6 * a3_2, 2 * a2_2 ] * [ t2, 1 ]

    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t1^3, t1^2, t1, 1 ] = q1
    //
    // [ 3 * a3_1, 2 * a2_1, a1_1 ] * [ t1^2, t2, 1 ] = 0
    // [ 6 * a3_1, 2 * a2_1 ] * [ t2, 1 ] = 0
    printf("COUNT %d", count);
    auto N = 4 * (count - 1);
    printf("%d unknowns\n", N);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(N, N);
    A.resize(N, N);

    Eigen::VectorXd b;
    b.resize(N, 1);

    auto t1 = ts[0];
    auto t12 = t1 * t1;
    auto t13 = t1 * t12;

    auto tf = ts[count - 1];
    auto tf2 = tf * tf;
    auto tf3 = tf * tf2;

    auto ii = N - 4;

    // poly_1(t_1) = q_1
    A(ii, 0) = t13;
    A(ii, 1) = t12;
    A(ii, 2) = t1;
    A(ii, 3) = real_t(1);
    b(ii) = qs[0];

    // poly'_1(t_1) = 0
    A(ii + 1, 0) = real_t(3) * t12;
    A(ii + 1, 1) = real_t(2) * t1;
    A(ii + 1, 2) = real_t(1);
    b(ii + 1) = real_t(0);

    // poly_(n-1)(t_n) = q_f
    A(ii + 2, N - 4) = tf3;
    A(ii + 2, N - 3) = tf2;
    A(ii + 2, N - 2) = tf;
    A(ii + 2, N - 1) = real_t(1);
    b(ii + 2) = qs[count - 1];

    // poly'_(n-1)(t_n) = 0
    A(ii + 3, N - 4) = real_t(3) * tf2;
    A(ii + 3, N - 3) = real_t(2) * tf;
    A(ii + 3, N - 2) = real_t(1);
    b(ii + 3) = real_t(0);

    for (auto i = 1; i < count - 1; ++i) { // i in [1, n - 1] (from 2 to n)
        auto tk = ts[i];
        auto tk2 = tk * tk;
        auto tk3 = tk * tk2;

        auto q = qs[i];

        auto a3km1 = 4 * (i - 1); // index of a_3 for the (k - 1)'th polynomial
        auto a3k = 4 * i;         // index of a_3 for the k'th polynomial
        auto eq = 4 * (i - 1);

        // poly_(k-1)(t_k) = q_k
        A(eq + 0, a3km1 + 0) = tk3;
        A(eq + 0, a3km1 + 1) = tk2;
        A(eq + 0, a3km1 + 2) = tk;
        A(eq + 0, a3km1 + 3) = real_t(1);

        // poly_(k-1)(t_k) = poly_k(t_k)
        A(eq + 1, a3km1 + 0) = tk3;
        A(eq + 1, a3km1 + 1) = tk2;
        A(eq + 1, a3km1 + 2) = tk;
        A(eq + 1, a3km1 + 3) = real_t(1);
        A(eq + 1, a3k + 0) = -tk3;
        A(eq + 1, a3k + 1) = -tk2;
        A(eq + 1, a3k + 2) = -tk;
        A(eq + 1, a3k + 3) = real_t(-1);

        // poly'_(k-1)(t_k) = poly'_(t_k)
        A(eq + 2, a3km1 + 0) = real_t(3) * tk2;
        A(eq + 2, a3km1 + 1) = real_t(2) * tk;
        A(eq + 2, a3km1 + 2) = real_t(1);
        A(eq + 2, a3k + 0) = real_t(-3) * tk2;
        A(eq + 2, a3k + 1) = real_t(-2) * tk;
        A(eq + 2, a3k + 2) = real_t(-1);

        // poly''_(k-1)(t_k) = poly''_k(t_k)
        A(eq + 3, a3km1 + 0) = real_t(6) * tk;
        A(eq + 3, a3km1 + 1) = real_t(2);
        A(eq + 3, a3k + 0) = real_t(-6) * tk;
        A(eq + 3, a3k + 1) = real_t(-2);

        b(eq + 0) = q;
        b(eq + 1) = real_t(0);
        b(eq + 2) = real_t(0);
        b(eq + 3) = real_t(0);
    }

    std::cout << "A = \n" << A << std::endl;
    std::cout << "b = \n" << b << std::endl;

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    CubicPathProfile profile;
    profile.times.assign(ts, ts + count);
    for (int i = 0; i < count - 1; ++i) {
        CubicProfile prof;
        prof.a[0] = x(4 * i + 0);
        prof.a[1] = x(4 * i + 1);
        prof.a[2] = x(4 * i + 2);
        prof.a[3] = x(4 * i + 3);
        profile.profiles.push_back(prof);
    }

    printf("done!\n");
    return profile;
}

auto ComputeCubicPathProfileContZero(std::vector<real_t>& qs, std::vector<real_t>& ts, int N)
    -> CubicPathProfile
{
    // inputs:
    // [ t1, t2, ..., tn ]
    // [ q1, q2, ..., qn ]
    // [ v1, v2, ..., vn ]

    // polynomial equations:
    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t^3, t^2, t^1, 1 ] = q(t), t1 <= t < t2
    // [ a3_2, a2_2, a1_2, a0_2 ] * [ t^3, t^2, t^1, 1 ] = q(t), t2 <= t < t3
    // ...
    // [ a3_(n-1), a2_(n-1), a1_(n-1), a0_(n-1) ] * [ t^3, t^2, t^1, 1 ] = q(t), t(n-1) <= t <= tn

    // first set of 4 constraints:
    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t2^3, t2^2, t2, 1 ] = q2
    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t2^3, t2^2, t2, 1 ] = [ a3_2, a2_2, a1_2, a0_1 ] * [ t2^3, t2^2, t2, 1 ]
    // [ 3 * a3_1, 2 * a2_1, a1_1 ] * [ t2^2, t2, 1 ] = [ 3 * a3_2, 2 * a2_2, a2_1 ] * [ t2^2, t2, 1 ]
    // [ 6 * a3_1, 2 * a2_1 ] * [ t2, 1 ] = [ 6 * a3_2, 2 * a2_2 ] * [ t2, 1 ]

    // [ a3_1, a2_1, a1_1, a0_1 ] * [ t1^3, t1^2, t1, 1 ] = q1
    //
    // [ 3 * a3_1, 2 * a2_1, a1_1 ] * [ t1^2, t2, 1 ] = 0
    // [ 6 * a3_1, 2 * a2_1 ] * [ t2, 1 ] = 0

    auto EQ = 4 * (N + 1);
    printf("%d unknowns\n", EQ);
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(EQ, EQ);
    A.resize(EQ, EQ);

    Eigen::VectorXd b = Eigen::VectorXd::Zero(EQ,1);
    // b.resize(EQ, 1);

    auto ii = EQ - 12;

    auto t2 = 0.5 * (ts[0] + ts[1]);
    auto tnp1 = 0.5 * (ts[N - 2] + ts[N - 1]);

    {
        auto t1 = ts[0];
        auto t12 = t1 * t1;
        auto t13 = t1 * t12;

        // poly_1(t_1) = q_1
        A(ii, 0) = t13;
        A(ii, 1) = t12;
        A(ii, 2) = t1;
        A(ii, 3) = real_t(1);
        b(ii) = qs[0];

        // poly'_1(t_1) = 0
        A(ii + 1, 0) = real_t(3) * t12;
        A(ii + 1, 1) = real_t(2) * t1;
        A(ii + 1, 2) = real_t(1);
        b(ii + 1) = real_t(0);

        // poly''_1(t_1) = 0
        A(ii + 2, 0) = real_t(6) * t1;
        A(ii + 2, 1) = real_t(2);
        b(ii + 2) = real_t(0);
    }

    {
        auto tf = ts[N - 1];
        auto tf2 = tf * tf;
        auto tf3 = tf * tf2;

        // poly_(n-1)(t_n) = q_f
        A(ii + 3, EQ - 4) = tf3;
        A(ii + 3, EQ - 3) = tf2;
        A(ii + 3, EQ - 2) = tf;
        A(ii + 3, EQ - 1) = real_t(1);
        b(ii + 3) = qs[N - 1];

        // poly'_(n-1)(t_n) = 0
        A(ii + 4, EQ - 4) = real_t(3) * tf2;
        A(ii + 4, EQ - 3) = real_t(2) * tf;
        A(ii + 4, EQ - 2) = real_t(1);
        b(ii + 4) = real_t(0);

        // poly''_(n-1)(t_n) = 0
        A(ii + 5, EQ - 4) = real_t(6) * tf;
        A(ii + 5, EQ - 3) = real_t(2);
        b(ii + 5) = real_t(0);
    }

    {
        auto t2_2 = t2 * t2;
        auto t2_3 = t2 * t2_2;

        A(ii + 6, 0) = t2_3;
        A(ii + 6, 1) = t2_2;
        A(ii + 6, 2) = t2;
        A(ii + 6, 3) = real_t(1);
        A(ii + 6, 4) = -t2_3;
        A(ii + 6, 5) = -t2_2;
        A(ii + 6, 6) = -t2;
        A(ii + 6, 7) = real_t(-1);

        A(ii + 7, 0) = real_t(3) * t2_2;
        A(ii + 7, 1) = real_t(2) * t2;
        A(ii + 7, 2) = real_t(1);
        A(ii + 7, 4) = real_t(-3) * t2_2;
        A(ii + 7, 5) = real_t(-2) * t2;
        A(ii + 7, 6) = real_t(-1);

        A(ii + 8, 0) = real_t(6) * t2;
        A(ii + 8, 1) = real_t(2);
        A(ii + 8, 4) = real_t(-6) * t2;
        A(ii + 8, 5) = real_t(-2);

        auto tnp1_2 = tnp1 * tnp1;
        auto tnp1_3 = tnp1 * tnp1_2;

        A(ii + 9, EQ - 8) = tnp1_3;
        A(ii + 9, EQ - 7) = tnp1_2;
        A(ii + 9, EQ - 6) = tnp1;
        A(ii + 9, EQ - 5) = real_t(1);
        A(ii + 9, EQ - 4) = -tnp1_3;
        A(ii + 9, EQ - 3) = -tnp1_2;
        A(ii + 9, EQ - 2) = -tnp1;
        A(ii + 9, EQ - 1) = real_t(-1);

        A(ii + 10, EQ - 8) = real_t(3) * tnp1_2;
        A(ii + 10, EQ - 7) = real_t(2) * tnp1;
        A(ii + 10, EQ - 6) = real_t(1);
        A(ii + 10, EQ - 4) = real_t(-3) * tnp1_2;
        A(ii + 10, EQ - 3) = real_t(-2) * tnp1;
        A(ii + 10, EQ - 2) = real_t(-1);

        A(ii + 11, EQ - 8) = real_t(6) * tnp1;
        A(ii + 11, EQ - 7) = real_t(2);
        A(ii + 11, EQ - 4) = real_t(-6) * tnp1;
        A(ii + 11, EQ - 3) = real_t(-2);
    }

    // i in [1, N - 2] (k in [2, N - 1])
    // k = 3, ..., N, i = 2, ..., N - 1
    for (auto k = 3; k <= N; ++k) {
        auto i = k - 1;

        auto tk = ts[i - 1];
        auto tk2 = tk * tk;
        auto tk3 = tk * tk2;

        auto q = qs[i - 1];

        auto a3km1 = 4 * (i - 1); // index of a_3 for the (k - 1)'th polynomial
        auto a3k = 4 * i;         // index of a_3 for the k'th polynomial

        // offset to the (k - 2)'th set of 4 equations, 0 for k = 3
        auto eq = 4 * (i - 2);

        // poly_(k-1)(t_k) = q_k
        A(eq + 0, a3km1 + 0) = tk3;
        A(eq + 0, a3km1 + 1) = tk2;
        A(eq + 0, a3km1 + 2) = tk;
        A(eq + 0, a3km1 + 3) = real_t(1);

        // poly_(k-1)(t_k) = poly_k(t_k)
        A(eq + 1, a3km1 + 0) = tk3;
        A(eq + 1, a3km1 + 1) = tk2;
        A(eq + 1, a3km1 + 2) = tk;
        A(eq + 1, a3km1 + 3) = real_t(1);
        A(eq + 1, a3k + 0) = -tk3;
        A(eq + 1, a3k + 1) = -tk2;
        A(eq + 1, a3k + 2) = -tk;
        A(eq + 1, a3k + 3) = real_t(-1);

        // poly'_(k-1)(t_k) = poly'_(t_k)
        A(eq + 2, a3km1 + 0) = real_t(3) * tk2;
        A(eq + 2, a3km1 + 1) = real_t(2) * tk;
        A(eq + 2, a3km1 + 2) = real_t(1);
        A(eq + 2, a3k + 0) = real_t(-3) * tk2;
        A(eq + 2, a3k + 1) = real_t(-2) * tk;
        A(eq + 2, a3k + 2) = real_t(-1);

        // poly''_(k-1)(t_k) = poly''_k(t_k)
        A(eq + 3, a3km1 + 0) = real_t(6) * tk;
        A(eq + 3, a3km1 + 1) = real_t(2);
        A(eq + 3, a3k + 0) = real_t(-6) * tk;
        A(eq + 3, a3k + 1) = real_t(-2);

        b(eq + 0) = q;
        b(eq + 1) = real_t(0);
        b(eq + 2) = real_t(0);
        b(eq + 3) = real_t(0);
    }

    std::cout << "A = \n" << A << std::endl;
    std::cout << "b = \n" << b << std::endl;

    Eigen::VectorXd x = A.colPivHouseholderQr().solve(b);

    CubicPathProfile profile;
    profile.times.push_back(ts[0]);
    profile.times.push_back(t2);
    for (int i = 0; i < N - 2; ++i) {
        profile.times.push_back(ts[i + 1]);
    }
    profile.times.push_back(tnp1);
    profile.times.push_back(ts[N - 1]);

    printf("N %d\n", N);
    for (int i = 0; i < N + 1; ++i) {
        CubicProfile prof;
        prof.a[0] = x(4 * i + 0);
        prof.a[1] = x(4 * i + 1);
        prof.a[2] = x(4 * i + 2);
        prof.a[3] = x(4 * i + 3);
        printf("%f %f %f %f\n", prof.a[0], prof.a[1], prof.a[2], prof.a[3]);
        profile.profiles.push_back(prof);
    }

    printf("done!\n");
    return profile;
}

auto find_profile(const CubicPathProfile& profile, real_t x)
    -> std::pair<const int, real_t>
{
    assert(!profile.profiles.empty());
    assert(profile.times.size() >= 2);

    auto comp = [](real_t x, real_t y) { return x >= y; }; // >= ?
    auto it = std::lower_bound(profile.times.rbegin(), profile.times.rend(), x, comp);

    if (it == profile.times.rend()) {
        auto& first_profile = profile.profiles.front();
        auto ti = profile.times.front();
        return std::make_pair(0, ti);
    }

    auto index = std::distance(profile.times.rbegin(), it);
    // return std::make_pair(profile.profiles[profile.profiles.size() - index], *it);
    return std::make_pair(profile.profiles.size() - index, *it);
}

auto pos(const CubicPathProfile& profile, real_t x) -> real_t
{
    auto res = find_profile(profile, x);
    return pos(profile.profiles[res.first], x /*- res.second*/);
}

auto vel(const CubicPathProfile& profile, real_t x) -> real_t
{
    auto res = find_profile(profile, x);
    return vel(profile.profiles[res.first], x /*- res.second*/);
}

auto acc(const CubicPathProfile& profile, real_t x) -> real_t
{
    auto res = find_profile(profile, x);
    return acc(profile.profiles[res.first], x /*- res.second*/);
}

} // namespace smpl

