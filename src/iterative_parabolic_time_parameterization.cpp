/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2011, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Ken Anderson */

#include "iterative_parabolic_time_parameterization.h"

// standard includes
#include <assert.h>
#include <cstdlib>
#include <cmath>

// system includes
#include <smpl/angles.h>
// #include <genx_motion_planner/joint_group.h>
#include <ros/console.h>

namespace genx {

static const char* LOG = "time_profiler";

// Initialize initial time stamps assuming infinite acceleration to a maximum
// velocity.

#if 0
static void ApplyVelocityConstraints(
    const RobotState traj[],
    int len,
    const RobotState* vel_limits,
    double time_diff[],
    double max_velocity_scaling_factor)
{
    auto scale = std::max(max_velocity_scaling_factor, 0.0);

    for (auto i = 0; i < len - 1; ++i) {
        auto& curr_waypoint = traj[i];
        auto& next_waypoint = traj[i + 1];

        time_diff[i] = 0.0;

        for (auto vidx = 0; vidx < Variable::VARIABLE_COUNT; ++vidx) {
            auto vlimit = (*vel_limits)[vidx];
            auto v_max = vlimit * scale;
            auto dq1 = curr_waypoint[vidx];
            auto dq2 = next_waypoint[vidx];
            auto t_min = std::fabs(dq2 - dq1) / v_max;
            if (t_min > time_diff[i]) {
                time_diff[i] = t_min;
            }
        }
    }
}

// Applies Acceleration constraints
void ApplyAccelerationConstraints(
    const RobotState traj[],
    int len,
    const RobotState* acc_limits,
    double time_diff[],
    double max_acceleration_scaling_factor,
    int max_iterations,
    double max_time_change_per_it)
{
    auto scale = std::max(0.0, max_acceleration_scaling_factor);

    auto iteration = 0;
    for (; iteration < max_iterations; ++iteration) {
        auto num_updates = 0;

        // In this case we iterate through the joints on the outer loop. This is
        // so that any time interval increases have a chance to get propogated
        // through the trajectory
        for (auto vidx = 0; vidx < Variable::VARIABLE_COUNT; ++vidx) {
            // Loop forwards, then backwards
            for (auto count = 0; count < 2; ++count) {
                auto backwards = (count == 1);
                for (auto i = 0; i < len - 1; ++i) {
                    auto index = backwards ? (len - 1) - i : i;

                    const RobotState* prev_waypoint = NULL;
                    const RobotState* curr_waypoint = NULL;
                    const RobotState* next_waypoint = NULL;

                    curr_waypoint = &traj[index];
                    if (index > 0) prev_waypoint = &traj[index - 1];
                    if (index < len - 1) next_waypoint = &traj[index + 1];

                    // Get acceleration limits
                    auto alimit = (*acc_limits)[vidx];
                    auto a_max = scale * alimit;

                    // positions and initial time deltas
                    double q1;
                    double q2;
                    double q3;
                    double dt1;
                    double dt2;

                    if (index == 0) {
                        // First point
                        q1 = (*next_waypoint)[vidx];
                        q2 = (*curr_waypoint)[vidx];
                        q3 = (*next_waypoint)[vidx];

                        dt1 = dt2 = time_diff[index];
                        assert(!backwards);
                    } else if (index < len - 1) {
                        // middle points
                        q1 = (*prev_waypoint)[vidx];
                        q2 = (*curr_waypoint)[vidx];
                        q3 = (*next_waypoint)[vidx];

                        dt1 = time_diff[index - 1];
                        dt2 = time_diff[index];
                    } else {
                        // last point - careful, there are only numpoints-1 time intervals
                        q1 = (*prev_waypoint)[vidx];
                        q2 = (*curr_waypoint)[vidx];
                        q3 = (*prev_waypoint)[vidx];

                        dt1 = dt2 = time_diff[index - 1];
                        assert(backwards);
                    }

                    auto dq1 = q2 - q1;
                    auto dq2 = q3 - q2;

                    // initial velocity and acceleration given current time deltas
                    double v1;
                    double v2;
                    double a;

                    if (dt1 == 0.0 || dt2 == 0.0) {
                        v1 = 0.0;
                        v2 = 0.0;
                        a = 0.0;
                    } else {
                        v1 = (q2 - q1) / dt1;
                        v2 = (q3 - q2) / dt2;
                        a = 2.0 * (v2 - v1) / (dt1 + dt2);
                    }

                    // expand time intervals to meet acceleration limits

                    auto ROUNDING_THRESHOLD = 0.01;
                    if (std::fabs(a) > a_max + ROUNDING_THRESHOLD)
                    {
                        if (!backwards) {
                            // Iteratively expand dt1 interval by a constant
                            // factor until within acceleration constraint In
                            // the future we may want to solve to quadratic
                            // equation to get the exact timing interval. To do
                            // this, use the CubicTrajectory::quadSolve()
                            // function in cubic_trajectory.h

                            auto max_dt2 = dt2 + max_time_change_per_it;

                            while (std::fabs(a) > a_max) {
                                auto mult_factor = 1.01;
                                auto new_dt2 = dt2 * mult_factor;
                                if (new_dt2 > max_dt2) {
                                    break;
                                }

                                // update time interval, velocity, and acceleration
                                dt2 = new_dt2;
                                v2 = dq2 / dt2;
                                a = 2.0 * (v2 - v1) / (dt1 + dt2);
                            }

                            time_diff[index] = dt2;
                        } else {
                            auto max_dt1 = dt1 + max_time_change_per_it;

                            while (std::fabs(a) > a_max) {
                                auto mult_factor = 1.01;
                                auto new_dt1 = dt1 * mult_factor;
                                if (new_dt1 > max_dt1) {
                                    break;
                                }

                                // update time interval, velocity, and acceleration
                                dt1 = new_dt1;
                                v1 = dq1 / dt1;
                                a = 2.0 * (v2 - v1) / (dt1 + dt2);
                            }

                            time_diff[index - 1] = dt1;
                        }
                        num_updates++;
                    }
                }
            }
        }

        if (num_updates == 0) {
            ROS_DEBUG_NAMED(LOG, "Acceleration profiling converged on iteration %d", iteration);
            break;
        }
    }

    if (iteration > max_iterations) {
        ROS_WARN_NAMED(LOG, "Acceleration profiling requires more than %d iterations. Solution may be outside limits", max_iterations);
    }
}

// Takes the time differences, and updates the timestamps, velocities and accelerations
// in the trajectory.
static
void UpdateTrajectory(
    const RobotState traj[],
    int len,
    double time_diff[],
    double times[],
    RobotState* velocities,
    RobotState* accelerations)
{
    // Error check
    if (len <= 1) return;

    times[0] = 0.0;

    // Times
    for (auto i = 1; i < len; ++i) {
        times[i] = time_diff[i - 1];
    }

    // Return if there is only one point in the trajectory!
    if (len <= 1) return;

    if (velocities == NULL && accelerations == NULL) return;

    for (auto i = 0; i < len; ++i) {
        const RobotState* prev_waypoint = NULL;
        const RobotState* curr_waypoint = NULL;
        const RobotState* next_waypoint = NULL;

        curr_waypoint = &traj[i];
        if (i > 0) prev_waypoint = &traj[i - 1];
        if (i < len - 1) next_waypoint = &traj[i + 1];

        for (auto vidx = 0; vidx < Variable::VARIABLE_COUNT; ++vidx) {
            double q1;
            double q2;
            double q3;
            double dt1;
            double dt2;

            if (i == 0) {
                // First point
                q1 = (*next_waypoint)[vidx];
                q2 = (*curr_waypoint)[vidx];
                q3 = q1;

                dt1 = dt2 = time_diff[i];
            } else if (i < len - 1) {
                // middle points
                q1 = (*prev_waypoint)[vidx];
                q2 = (*curr_waypoint)[vidx];
                q3 = (*next_waypoint)[vidx];

                dt1 = time_diff[i - 1];
                dt2 = time_diff[i];
            } else {
                // last point
                q1 = (*prev_waypoint)[vidx];
                q2 = (*curr_waypoint)[vidx];
                q3 = q1;

                dt1 = dt2 = time_diff[i - 1];
            }

            double v1;
            double v2;
            double a;

            if (dt1 == 0.0 || dt2 == 0.0) {
                v1 = 0.0;
                v2 = 0.0;
                a = 0.0;
            } else {
                v1 = (q2 - q1) / dt1;
                v2 = (q3 - q2) / dt2;
                a = 2.0 * (v2 - v1) / (dt1 + dt2);
            }

            if (velocities != NULL) velocities[i][vidx] = 0.5 * (v1 + v2);
            if (accelerations != NULL) accelerations[i][vidx] = a;
        }
    }
}

bool ComputeTimeStamps(
    const RobotState* traj,
    int len,
    const RobotState* vel_limits,
    const RobotState* acc_limits,
    double times[],
    RobotState* velocities,
    RobotState* accelerations,
    int max_iterations,
    double max_time_change_per_it,
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
{
    if (len == 0) return true;

    // this lib does not actually work properly when angles wrap around, so we
    // need to unwind the path first. In our case, the only variable that could
    // wrap is the orientation of the base, which we aren't currently planning
    // for, so we're ignoring this for now.
//    traj.unwind();

    // the time difference between adjacent points
    std::vector<double> time_diff(len - 1);

    ApplyVelocityConstraints(
            traj,
            len,
            vel_limits,
            time_diff.data(),
            max_velocity_scaling_factor);

    ApplyAccelerationConstraints(
            traj,
            len,
            acc_limits,
            time_diff.data(),
            max_acceleration_scaling_factor,
            max_iterations,
            max_time_change_per_it);

    UpdateTrajectory(traj, len, time_diff.data(), times, velocities, accelerations);
    return true;
}
#endif
// static void fit_cubic_spline(
//     const int n,
//     const double dt[],
//     const double x[],
//     double x1[],
//     double x2[]);

static void adjust_two_positions(
    const int n,
    const double dt[],
    double x[],
    double x1[],
    double x2[],
    const double x2_i,
    const double x2_f);

static void init_times(
    const int n,
    double dt[],
    const double x[],
    const double max_velocity,
    const double min_velocity);

static int fit_spline_and_adjust_times(
    const int n,
    double dt[],
    const double x[],
    double x1[],
    double x2[],
    const double max_velocity,
    const double min_velocity,
    const double max_acceleration,
    const double min_acceleration,
    const double tfactor);

static double global_adjustment_factor(
    const int n,
    double dt[],
    const double x[],
    double x1[],
    double x2[],
    const double max_velocity,
    const double min_velocity,
    const double max_acceleration,
    const double min_acceleration);

// The path of a single joint: positions, velocities, and accelerations
struct SingleJointTrajectory
{
  std::vector<double> positions;  // joint's position at time[x]
  std::vector<double> velocities;
  std::vector<double> accelerations;
  double initial_acceleration;
  double final_acceleration;
  double min_velocity;
  double max_velocity;
  double min_acceleration;
  double max_acceleration;
};

void globalAdjustment(
    std::vector<SingleJointTrajectory>& t2,
    int num_joints,
    const int num_points,
    std::vector<double>& time_diff);

static bool add_points_ = false;

#if 0
bool ComputeTimeStamps2(
    const RobotState* traj,
    int len,
    const RobotState* vel_limits,
    const RobotState* acc_limits,
    double times[],
    RobotState* velocities,
    RobotState* accelerations,
    double max_velocity_scaling_factor,
    double max_acceleration_scaling_factor)
{
    if (len == 0) return true;

    auto velocity_scaling_factor = 1.0;
    auto acceleration_scaling_factor = 1.0;
    auto num_points = len;
    auto num_joints = VARIABLE_COUNT;

  if (add_points_)
  {
    // Insert 2nd and 2nd-last points
    // (required to force acceleration to specified values at endpoints)
    if (len >= 2)
    {
      robot_state::RobotState point = trajectory.getWayPoint(1);
      robot_state::RobotStatePtr p0, p1;

      // 2nd point is 90% of p0, and 10% of p1
      p0 = trajectory.getWayPointPtr(0);
      p1 = trajectory.getWayPointPtr(1);
      for (unsigned int j = 0; j < num_joints; j++)
      {
        point.setVariablePosition(idx[j],
                                  (9 * p0->getVariablePosition(idx[j]) + 1 * p1->getVariablePosition(idx[j])) / 10);
      }
      trajectory.insertWayPoint(1, point, 0.0);
      num_points++;

      // 2nd-last point is 10% of p0, and 90% of p1
      p0 = trajectory.getWayPointPtr(num_points - 2);
      p1 = trajectory.getWayPointPtr(num_points - 1);
      for (unsigned int j = 0; j < num_joints; j++)
      {
        point.setVariablePosition(idx[j],
                                  (1 * p0->getVariablePosition(idx[j]) + 9 * p1->getVariablePosition(idx[j])) / 10);
      }
      trajectory.insertWayPoint(num_points - 1, point, 0.0);
      num_points++;
    }
  }

    // Error check
    if (num_points < 4) {
        ROS_ERROR_NAMED("trajectory_processing.iterative_spline_parameterization", "number of waypoints %d, needs to be greater than 3.\n", num_points);
        return false;
    }

    // JointTrajectory indexes in [point][joint] order.
    // We need [joint][point] order to solve efficiently,
    // so convert form here.

    std::vector<SingleJointTrajectory> t2(num_joints);

    for (auto j = 0; j < num_joints; j++) {
        // Copy positions
        t2[j].positions.resize(num_points, 0.0);
        for (auto i = 0; i < num_points; i++) {
            t2[j].positions[i] = traj[i][j];
        }

        // Initialize velocities
        t2[j].velocities.resize(num_points, 0.0);

#if 0
        // Copy initial/final velocities if specified
        if (trajectory.getWayPointPtr(0)->hasVelocities()) {
            t2[j].velocities[0] = trajectory.getWayPointPtr(0)->getVariableVelocity(idx[j]);
        }
        if (trajectory.getWayPointPtr(num_points - 1)->hasVelocities()) {
            t2[j].velocities[num_points - 1] = trajectory.getWayPointPtr(num_points - 1)->getVariableVelocity(idx[j]);
        }
#endif

        // Initialize accelerations
        t2[j].accelerations.resize(num_points, 0.0);
        t2[j].initial_acceleration = 0.0;
        t2[j].final_acceleration = 0.0;

#if 0
        // Copy initial/final accelerations if specified
        if (trajectory.getWayPointPtr(0)->hasAccelerations()) {
            t2[j].initial_acceleration = trajectory.getWayPointPtr(0)->getVariableAcceleration(idx[j]);
        }
        t2[j].accelerations[0] = t2[j].initial_acceleration;
        if (trajectory.getWayPointPtr(num_points - 1)->hasAccelerations()) {
            t2[j].final_acceleration = trajectory.getWayPointPtr(num_points - 1)->getVariableAcceleration(idx[j]);
        }
        t2[j].accelerations[num_points - 1] = t2[j].final_acceleration;
#endif

        // Set bounds based on model, or default limits
        t2[j].max_velocity = (*vel_limits)[j];
        t2[j].min_velocity = -(*vel_limits)[j];
        t2[j].max_velocity *= velocity_scaling_factor;
        t2[j].min_velocity *= velocity_scaling_factor;

        t2[j].max_acceleration = (*acc_limits)[j];
        t2[j].min_acceleration = -(*acc_limits)[j];
        t2[j].max_acceleration *= acceleration_scaling_factor;
        t2[j].min_acceleration *= acceleration_scaling_factor;
    }

    // Initialize times
    // start with valid velocities, then expand intervals
    // epsilon to prevent divide-by-zero
    std::vector<double> time_diff(len - 1, std::numeric_limits<double>::epsilon());
    for (auto j = 0; j < num_joints; j++) {
        init_times(num_points, &time_diff[0], &t2[j].positions[0], t2[j].max_velocity, t2[j].min_velocity);
    }

    // Stretch intervals until close to the bounds
    while (1) {
        int loop = 0;

        // Calculate the interval stretches due to acceleration
        std::vector<double> time_factor(num_points - 1, 1.00);
        for (unsigned j = 0; j < num_joints; j++) {
            // Move points to satisfy initial/final acceleration
            if (add_points_) {
                adjust_two_positions(num_points, &time_diff[0], &t2[j].positions[0], &t2[j].velocities[0], &t2[j].accelerations[0], t2[j].initial_acceleration, t2[j].final_acceleration);
            }

            fit_cubic_spline(num_points, &time_diff[0], &t2[j].positions[0], &t2[j].velocities[0], &t2[j].accelerations[0]);
            for (unsigned i = 0; i < num_points; i++) {
                const double acc = t2[j].accelerations[i];
                double atfactor = 1.0;
                if (acc > t2[j].max_acceleration) {
                    atfactor = sqrt(acc / t2[j].max_acceleration);
                }
                if (acc < t2[j].min_acceleration) {
                    atfactor = sqrt(acc / t2[j].min_acceleration);
                }
                if (atfactor > 1.01) { // within 1%
                    loop = 1;
                }
                atfactor = (atfactor - 1.0) / 16.0 + 1.0;  // 1/16th
                if (i > 0) {
                    time_factor[i - 1] = std::max(time_factor[i - 1], atfactor);
                }
                if (i < num_points - 1) {
                    time_factor[i] = std::max(time_factor[i], atfactor);
                }
            }
        }

        if (loop == 0) {
            // finished
            break;
        }

        // Stretch
        for (unsigned i = 0; i < num_points - 1; i++) {
            time_diff[i] *= time_factor[i];
        }
    }

    // Final adjustment forces the trajectory within bounds
    globalAdjustment(t2, num_joints, num_points, time_diff);

    // Convert back to JointTrajectory form
    times[0] = 0.0;
    for (unsigned int i = 1; i < num_points; i++) {
        times[i] = time_diff[i - 1];
    }
    for (auto i = 0; i < num_points; i++) {
        for (auto j = 0; j < num_joints; j++) {
            // TODO: adjusted positions here?
//            trajectory.getWayPointPtr(i)->setVariablePosition(idx[j], t2[j].positions[i]);
            velocities[i][j] = t2[j].velocities[i];
            accelerations[i][j] = t2[j].accelerations[i];
        }
    }

    return true;
}
#endif

//////// Internal functions //////////////

/*
  Fit a 'clamped' cubic spline over a series of points.
  A cubic spline ensures continuous function across positions,
  1st derivative (velocities), and 2nd derivative (accelerations).
  'Clamped' means the first derivative at the endpoints is specified.
  Fitting a cubic spline involves solving a series of linear equations.
  The general form for each segment is:
    (tj-t_(j-1))*x"_(j-1) + 2*(t_(j+1)-t_(j-1))*x"j + (t_(j+1)-tj)*x"_j+1) =
          (x_(j+1)-xj)/(t_(j+1)-tj) - (xj-x_(j-1))/(tj-t_(j-1))
  And the first and last segment equations are clamped to specified values: x1_i and x1_f.
  Represented in matrix form:
  [ 2*(t1-t0)   (t1-t0)                              0              ][x0"]       [(x1-x0)/(t1-t0) - t1_i           ]
  [ t1-t0       2*(t2-t0)   t2-t1                                   ][x1"]       [(x2-x1)/(t2-t1) - (x1-x0)/(t1-t0)]
  [             t2-t1       2*(t3-t1)   t3-t2                       ][x2"] = 6 * [(x3-x2)/(t3/t2) - (x2-x1)/(t2-t1)]
  [                       ...         ...         ...               ][...]       [...                              ]
  [ 0                                    tN-t_(N-1)  2*(tN-t_(N-1)) ][xN"]       [t1_f - (xN-x_(N-1))/(tN-t_(N-1)) ]
  This matrix is tridiagonal, which can be solved solved in O(N) time
  using the tridiagonal algorithm.
  There is a forward propogation pass followed by a backsubstitution pass.
  n is the number of points
  dt contains the time difference between each point (size=n-1)
  x  contains the positions                          (size=n)
  x1 contains the 1st derivative (velocities)        (size=n)
     x1[0] and x1[n-1] MUST be specified.
  x2 contains the 2nd derivative (accelerations)     (size=n)
  x1 and x2 are filled in by the algorithm.
*/

void fit_cubic_spline(const int n, const double dt[], const double x[], double x1[], double x2[])
{
  int i;
  const double x1_i = x1[0], x1_f = x1[n - 1];

  // Tridiagonal alg - forward sweep
  // x1 and x2 used to store the temporary coefficients c and d
  // (will get overwritten during backsubstitution)
  double *c = x1, *d = x2;
  c[0] = 0.5;
  d[0] = 3.0 * ((x[1] - x[0]) / dt[0] - x1_i) / dt[0];
  for (i = 1; i <= n - 2; i++)
  {
    const double dt2 = dt[i - 1] + dt[i];
    const double a = dt[i - 1] / dt2;
    const double denom = 2.0 - a * c[i - 1];
    c[i] = (1.0 - a) / denom;
    d[i] = 6.0 * ((x[i + 1] - x[i]) / dt[i] - (x[i] - x[i - 1]) / dt[i - 1]) / dt2;
    d[i] = (d[i] - a * d[i - 1]) / denom;
    
  }
  const double denom = dt[n - 2] * (2.0 - c[n - 2]);
  d[n - 1] = 6.0 * (x1_f - (x[n - 1] - x[n - 2]) / dt[n - 2]);
  d[n - 1] = (d[n - 1] - dt[n - 2] * d[n - 2]) / denom;

  // Tridiagonal alg - backsubstitution sweep
  // 2nd derivative
  x2[n - 1] = d[n - 1];
  for (i = n - 2; i >= 0; i--) {
    // printf("i %d d[i] %f c[i] %f x2[i + 1] %f : x2[i] %f\n", i, d[i], c[i], x2[i + 1], x2[i]);
    x2[i] = d[i] - c[i] * x2[i + 1];
}
    // getchar();

  // 1st derivative
  x1[0] = x1_i;
  for (i = 1; i < n - 1; i++)
    x1[i] = (x[i + 1] - x[i]) / dt[i] - (2 * x2[i] + x2[i + 1]) * dt[i] / 6.0;
  x1[n - 1] = x1_f;
}

/*
  Modify the value of x[1] and x[N-2]
  so that 2nd derivative starts and ends at specified value.
  This involves fitting the spline twice,
  then solving for the specified value.
  x2_i and x2_f are the (initial and final) 2nd derivative at 0 and N-1
*/

static void adjust_two_positions(const int n, const double dt[], double x[], double x1[], double x2[],
                                 const double x2_i, const double x2_f)
{
  x[1] = x[0];
  x[n - 2] = x[n - 3];
  fit_cubic_spline(n, dt, x, x1, x2);
  double a0 = x2[0];
  double b0 = x2[n - 1];

  x[1] = x[2];
  x[n - 2] = x[n - 1];
  fit_cubic_spline(n, dt, x, x1, x2);
  double a2 = x2[0];
  double b2 = x2[n - 1];

  // we can solve this with linear equation (use two-point form)
  if (a2 != a0)
    x[1] = x[0] + ((x[2] - x[0]) / (a2 - a0)) * (x2_i - a0);
  if (b2 != b0)
    x[n - 2] = x[n - 3] + ((x[n - 1] - x[n - 3]) / (b2 - b0)) * (x2_f - b0);
}

/*
  Find time required to go max velocity on each segment.
  Increase a segment's time interval if the current time isn't long enough.
*/

static void init_times(const int n, double dt[], const double x[], const double max_velocity, const double min_velocity)
{
  int i;
  for (i = 0; i < n - 1; i++)
  {
    double time;
    double dx = x[i + 1] - x[i];
    if (dx >= 0.0)
      time = (dx / max_velocity);
    else
      time = (dx / min_velocity);
    time += std::numeric_limits<double>::epsilon();  // prevent divide-by-zero

    if (dt[i] < time)
      dt[i] = time;
  }
}

/*
  Fit a spline, then check each interval to see if bounds are met.
  If all bounds met (no time adjustments made), return 0.
  If bounds not met (time adjustments made), slightly increase the
  surrounding time intervals and return 1.
  n is the number of points
  dt contains the time difference between each point (size=n-1)
  x  contains the positions                          (size=n)
  x1 contains the 1st derivative (velocities)        (size=n)
     x1[0] and x1[n-1] MUST be specified.
  x2 contains the 2nd derivative (accelerations)     (size=n)
  max_velocity is the max velocity for this joint.
  min_velocity is the min velocity for this joint.
  max_acceleration is the max acceleration for this joint.
  min_acceleration is the min acceleration for this joint.
  tfactor is the time adjustment (multiplication) factor.
  x1 and x2 are filled in by the algorithm.
*/

static int fit_spline_and_adjust_times(
    const int n,
    double dt[],
    const double x[],
    double x1[],
    double x2[],
    const double max_velocity,
    const double min_velocity,
    const double max_acceleration,
    const double min_acceleration,
    const double tfactor)
{
  int i, ret = 0;

  fit_cubic_spline(n, dt, x, x1, x2);

  // Instantaneous velocity is calculated at each point
  for (i = 0; i < n - 1; i++)
  {
    const double vel = x1[i];
    const double vel2 = x1[i + 1];
    if (vel > max_velocity || vel < min_velocity || vel2 > max_velocity || vel2 < min_velocity)
    {
      dt[i] *= tfactor;
      ret = 1;
    }
  }
  // Instantaneous acceleration is calculated at each point
  if (ret == 0)
  {
    for (i = 0; i < n - 1; i++)
    {
      const double acc = x2[i];
      const double acc2 = x2[i + 1];
      if (acc > max_acceleration || acc < min_acceleration || acc2 > max_acceleration || acc2 < min_acceleration)
      {
        dt[i] *= tfactor;
        ret = 1;
      }
    }
  }

  return ret;
}

// return global expansion multiplicative factor required
// to force within bounds.
// Assumes that the spline is already fit
// (fit_cubic_spline must have been called before this).
static double global_adjustment_factor(
    const int n,
    double dt[],
    const double x[],
    double x1[],
    double x2[],
    const double max_velocity,
    const double min_velocity,
    const double max_acceleration,
    const double min_acceleration)
{
  int i;
  double tfactor2 = 1.00;

  // fit_cubic_spline(n, dt, x, x1, x2);

  for (i = 0; i < n; i++)
  {
    double tfactor;
    tfactor = x1[i] / max_velocity;
    if (tfactor2 < tfactor)
      tfactor2 = tfactor;
    tfactor = x1[i] / min_velocity;
    if (tfactor2 < tfactor)
      tfactor2 = tfactor;

    if (x2[i] >= 0)
    {
      tfactor = sqrt(fabs(x2[i] / max_acceleration));
      if (tfactor2 < tfactor)
        tfactor2 = tfactor;
    }
    else
    {
      tfactor = sqrt(fabs(x2[i] / min_acceleration));
      if (tfactor2 < tfactor)
        tfactor2 = tfactor;
    }
  }

  return tfactor2;
}

// Expands the entire trajectory to fit exactly within bounds
void globalAdjustment(
    std::vector<SingleJointTrajectory>& t2,
    int num_joints,
    const int num_points,
    std::vector<double>& time_diff)
{
    double gtfactor = 1.0;
    for (int j = 0; j < num_joints; j++) {
        double tfactor;
        tfactor = global_adjustment_factor(
                num_points,
                &time_diff[0],
                &t2[j].positions[0],
                &t2[j].velocities[0],
                &t2[j].accelerations[0],
                t2[j].max_velocity,
                t2[j].min_velocity,
                t2[j].max_acceleration,
                t2[j].min_acceleration);
        if (tfactor > gtfactor) {
            gtfactor = tfactor;
        }
    }

    // printf("# Global adjustment: %0.4f%%\n", 100.0 * (gtfactor - 1.0));
    for (int i = 0; i < num_points - 1; i++) {
        time_diff[i] *= gtfactor;
    }

    for (int j = 0; j < num_joints; j++) {
        fit_cubic_spline(num_points, &time_diff[0], &t2[j].positions[0], &t2[j].velocities[0], &t2[j].accelerations[0]);
    }
}

} // namespace genx
