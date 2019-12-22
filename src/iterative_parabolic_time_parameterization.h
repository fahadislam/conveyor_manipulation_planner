#ifndef MOVEIT_ITERATIVE_PARABOLIC_TIME_PARAMETERIZATION_H
#define MOVEIT_ITERATIVE_PARABOLIC_TIME_PARAMETERIZATION_H

#include <stddef.h>
#include <cstdint>

namespace genx {

struct RobotState;

// @brief maximum number of iterations to find solution
// @brief maximum allowed time change per iteration in seconds
bool ComputeTimeStamps(
    const RobotState* traj,
    int len,
    const RobotState* vel_limits,
    const RobotState* acc_limits,
    double times[], // time interval from previous waypoint
    RobotState* velocities = NULL,
    RobotState* accelerations = NULL,
    int max_iterations = 100,
    double max_time_change_per_it = 0.01,
//     double max_velocity_scaling_factor = 1.0,
//     double max_acceleration_scaling_factor = 1.0
    double max_velocity_scaling_factor = 1.0,  // HACK: just for simulation. Someone's getting in trouble for this one -Andrew
    double max_acceleration_scaling_factor = 1.0  // HACK: just for simulation
    );

bool ComputeTimeStamps2(
    const RobotState* traj,
    int len,
    const RobotState* vel_limits,
    const RobotState* acc_limits,
    double times[],
    RobotState* velocities = NULL,
    RobotState* accelerations = NULL,
    double max_velocity_scaling_factor = 1.0,
    double max_acceleration_scaling_factor = 1.0);

void fit_cubic_spline(
    const int n,
    const double dt[],
    const double x[],
    double x1[],
    double x2[]);

} // namespace genx

#endif

