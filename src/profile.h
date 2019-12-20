#ifndef SMPL_PROFILE_H
#define SMPL_PROFILE_H

#include <vector>

namespace smpl {

using real_t = double;

template <class T>
struct pi
{
    static constexpr auto value = T(3.1415926535897932384626433832795028841);

    operator T() { return value; }
};

struct LinearProfile
{
};

// CubicProfile describes a cubic function that smoothly interpolates between
// two specified positions, with specific first derivatives at the endpoints.
// This function is capable of minimizing the total joint torque applied over
// time for a single joint (with the assumption that joint torque is not a
// function of joint configuration), but is incapable of specifying the second
// derivatives at the endpoints.
struct CubicProfile
{
    real_t a[4];
};

// QuinticProfile describes a quintic function that smoothly interpolates
// between two specified positions, with specific first and second derivatives
// at the endpoints. This function is not capable of minimizing the total
// joint torque applied over time for a single joint.
struct QuinticProfile
{
    real_t a[6];
};

// TrapezoidalProfile describes a piecewise polynomial consisting of an
// acceleration phase, a cruising velocity phase, and a deceleration phase.
// The profile is derived from either the maximum acceleration or the cruising
// velocity. This function is not capable of minimizing the total joint torque
// applied over time, but the performance is bounded from above by 12.5% with
// respect to optimal.
struct TrapezoidalProfile
{
    real_t qi;
    real_t qf;
    real_t ac;
    real_t tc;
    real_t tf;
};

struct CubicPathProfile
{
    std::vector<CubicProfile> profiles;
    std::vector<real_t> times;
};

// Return the coefficients of the cubic function satisfying the following
// conditions:
// (1) q(0) = q_i
// (2) q(t_f) = q_f
// (3) q'(0) = q'_i
// (4) q'(t_f) = q'_f
auto ComputeCubicProfile(real_t qi, real_t qf, real_t vi, real_t vf, real_t tf)
    -> CubicProfile;

// Sample a CubicProfile at a specific instant in time.
auto pos(CubicProfile profile, real_t x) -> real_t;
auto vel(CubicProfile profile, real_t x) -> real_t;
auto acc(CubicProfile profile, real_t x) -> real_t;

auto ComputeQuinticProfile(
    real_t qi, real_t qf,
    real_t vi, real_t vf,
    real_t ai, real_t af,
    real_t tf)
    -> QuinticProfile;

// Sample a QuinticProfile at a specific instant in time.
auto pos(QuinticProfile profile, real_t x) -> real_t;
auto vel(QuinticProfile profile, real_t x) -> real_t;
auto acc(QuinticProfile profile, real_t x) -> real_t;

auto ComputeTrapezoidalProfileVel(real_t qi, real_t qf, real_t vc, real_t tf)
    -> TrapezoidalProfile;

auto ComputeTrapezoidalProfileAcc(real_t qi, real_t qf, real_t ac, real_t tf)
    -> TrapezoidalProfile;

// Sample a TrapezoidalProfile at a specific instant in time.
auto pos(TrapezoidalProfile profile, real_t x) -> real_t;
auto vel(TrapezoidalProfile profile, real_t x) -> real_t;
auto acc(TrapezoidalProfile profile, real_t x) -> real_t;

// Describe cubic functions interpolating a sequence of points, with specific
// first derivatives, at specific timestamps. The profile exhibits
// discontinuities on the second derivative at each waypoint.
auto ComputeCubicPathProfile(real_t* qs, real_t* vs, real_t* ts, int count)
    -> CubicPathProfile;

// Describe cubic functions interpolating a sequence of points at specific
// timestamps by choosing reasonable values for first derivatives at each
// point. The endpoints of the first derivative are assigned to 0.  The
// profile exhibits discontinuities on the second derivative at each timestamp.
auto ComputeCubicPathProfile(real_t* qs, real_t* ts, int count)
    -> CubicPathProfile;

// Describe cubic functions interpolating a sequence of points at specific
// timestamps, choosing reasonable values for the first derivatives at each
// point. The profile is continuous on the second derivative, with the endpoints
// of the first derivative set to 0, and arbitrarily assigned endpoints for the
// second derivative.
auto ComputeCubicPathProfileCont(real_t* qs, real_t* ts, int count)
    -> CubicPathProfile;

// Describe cubic functions interpolating a sequence of points at specific
// timestamps, choosing reasonable values for the first derivatives at each
// point. The profile is continuous on the second derivative and the endpoints
// have first and second derivatives equal to 0.
auto ComputeCubicPathProfileContZero(std::vector<real_t>& qs, std::vector<real_t>& ts, int N)
    -> CubicPathProfile;

// Sample a CubicPathProfile at a specific instant in time.
auto pos(const CubicPathProfile& profile, real_t x) -> real_t;
auto vel(const CubicPathProfile& profile, real_t x) -> real_t;
auto acc(const CubicPathProfile& profile, real_t x) -> real_t;

auto ComputeTimeStamps(real_t* qs, int count) -> std::vector<real_t>;

} // namespace smpl

#endif
