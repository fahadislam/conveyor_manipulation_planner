// standard includes
#include <math.h>
#include <assert.h>
#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <limits>
#include <utility>

#include <Eigen/Dense>
#include <eigen_conversions/eigen_msg.h>
#include <ros/ros.h>
#include <smpl/angles.h>
#include <smpl/spatial.h>
#include <smpl/types.h>

#include "conversions.h"

// static
// void ConvertRobotPathToRobotTrajectoryMsg(
//     const RobotPath* path,
//     moveit_msgs::RobotTrajectory* otraj)
// {
//     // metadata
//     otraj->joint_trajectory.joint_names = GetSingleDOFJointNames();
//     otraj->multi_dof_joint_trajectory.joint_names = GetMultiDOFJointNames();

//     for (auto i = 0; i < path->size(); ++i) {
//         auto& position = (*path)[i];

//         trajectory_msgs::JointTrajectoryPoint p;
//         p.positions = ExtractSingleDOFJointPositions(position);
//         otraj->joint_trajectory.points.push_back(p);

//         trajectory_msgs::MultiDOFJointTrajectoryPoint pp;
//         pp.transforms = ExtractMultiDOFJointTransforms(position);
//         otraj->multi_dof_joint_trajectory.points.push_back(pp);
//     }

//     assert(otraj->multi_dof_joint_trajectory.points.size() == path->size());
//     assert(otraj->joint_trajectory.points.size() == path->size());
// }

// static
// void ProfilePath(
//     const RobotPath* path,
//     const RobotState* vel_limits,
//     const RobotState* acc_limits,
//     moveit_msgs::RobotTrajectory* otraj)
// {
//     std::vector<double> times(path->size());
//     std::vector<RobotState> velocities(path->size());
//     std::vector<RobotState> accelerations(path->size());
//     ComputeTimeStamps(
//             path->data(), path->size(),   // input trajectory
//             vel_limits, acc_limits,         // vel/acc limits
//             times.data(),                   // output times
//             velocities.data(),              // output velocities
//             accelerations.data());          // output accelerations

//     auto time_from_start = 0.0;
//     for (auto i = 0; i < path->size(); ++i) {
//         time_from_start += times[i];
//         ROS_DEBUG_NAMED(LOG, "time from start = %f", time_from_start);

//         auto& velocity = velocities[i];
//         auto& acceleration = accelerations[i];

//         auto& jtp = otraj->joint_trajectory.points[i];
//         auto& mdjtp = otraj->multi_dof_joint_trajectory.points[i];

//         jtp.velocities = ExtractSingleDOFJointPositions(velocity);
//         jtp.accelerations = ExtractSingleDOFJointPositions(acceleration);
//         jtp.time_from_start = ros::Duration(time_from_start);

//         mdjtp.velocities = ExtractMultiDOFJointTwists(velocity);
//         mdjtp.accelerations = ExtractMultiDOFJointTwists(acceleration);
//         mdjtp.time_from_start = ros::Duration(time_from_start);
//     }
// }

// void ConvertRobotPathToRobotTrajectoryMsg(
//     const RobotPath* path,
//     const smpl::RobotState* vel_limits,
//     const smpl::RobotState* acc_limits,
//     moveit_msgs::RobotTrajectory* otraj)
// {
//     ConvertRobotPathToRobotTrajectoryMsg(path, otraj);
//     ProfilePath(path, vel_limits, acc_limits, otraj);
// }

static
auto interp(double src, double dst, double alpha) -> double
{
    return (1.0 - alpha) * src + alpha * dst;
}

static
auto get_nearest_planar_rotation(const geometry_msgs::Quaternion& q) -> double
{
    Eigen::Quaterniond qq;
    tf::quaternionMsgToEigen(q, qq);
    return smpl::get_nearest_planar_rotation(qq);
}

struct RobotTrajectoryPoint
{
    trajectory_msgs::JointTrajectoryPoint jtp;
    trajectory_msgs::MultiDOFJointTrajectoryPoint mdjtp;
};

static
auto interp(geometry_msgs::Transform a, geometry_msgs::Transform b, double alpha)
    -> geometry_msgs::Transform
{
    geometry_msgs::Transform c;
    c.translation.x = interp(a.translation.x, b.translation.x, alpha);
    c.translation.y = interp(a.translation.y, b.translation.y, alpha);
    c.translation.z = interp(a.translation.z, b.translation.z, alpha);

    auto qa = smpl::Quaternion(a.rotation.w, a.rotation.x, a.rotation.y, a.rotation.z);
    auto qb = smpl::Quaternion(b.rotation.w, b.rotation.x, b.rotation.y, b.rotation.z);
    auto qc = qa.slerp(alpha, qb);
    c.rotation.w = qc.w();
    c.rotation.x = qc.x();
    c.rotation.y = qc.y();
    c.rotation.z = qc.z();

    return c;
}

static
auto interp(geometry_msgs::Twist a, const geometry_msgs::Twist b, double alpha)
    -> geometry_msgs::Twist
{
    geometry_msgs::Twist c;

    c.linear.x = interp(a.linear.x, b.linear.x, alpha);
    c.linear.y = interp(a.linear.y, b.linear.y, alpha);
    c.linear.z = interp(a.linear.z, b.linear.z, alpha);

    c.angular.x = interp(a.angular.x, b.angular.x, alpha);
    c.angular.y = interp(a.angular.y, b.angular.y, alpha);
    c.angular.z = interp(a.angular.z, b.angular.z, alpha);

    return c;
}

#if 0
static
auto Sample(const moveit_msgs::RobotTrajectory* traj, double t)
    -> RobotTrajectoryPoint
{
    for (auto i = 1; i < traj->joint_trajectory.points.size(); ++i) {
        auto& curr = traj->joint_trajectory.points[i - 1];
        auto& next = traj->joint_trajectory.points[i];
        auto& mcurr = traj->multi_dof_joint_trajectory.points[i - 1];
        auto& mnext = traj->multi_dof_joint_trajectory.points[i];
        assert(mcurr.transforms.size() == 1 && mnext.transforms.size() == 1);

        double e = 0.001;
        if (curr.time_from_start.toSec() <= t && t <= next.time_from_start.toSec() + e) {
            auto jtp = trajectory_msgs::JointTrajectoryPoint();
            auto mjtp = trajectory_msgs::MultiDOFJointTrajectoryPoint();

            jtp.positions.resize(curr.positions.size());
            // jtp.velocities.resize(curr.velocities.size());
            // jtp.accelerations.resize(curr.accelerations.size());

            // mjtp.transforms.resize(mcurr.transforms.size());
            // mjtp.velocities.resize(mcurr.velocities.size());
            // mjtp.accelerations.resize(mcurr.accelerations.size());

            auto dt = next.time_from_start.toSec() - curr.time_from_start.toSec();
            assert(dt >= 0.0);
            auto alpha = dt >= 1e-6 ? (t - curr.time_from_start.toSec()) / dt : 0.0;

            // linearly interpolate single-dof joint positions
            for (auto j = 0; j < curr.positions.size(); ++j) {
                if (j < curr.positions.size() - 4) {
#if 1
                    jtp.positions[j] = interp(curr.positions[j], next.positions[j], alpha);
#else
                    auto tt = dt * alpha;
                    jtp.positions[j] = curr.positions[j] + curr.velocities[j] * tt + 0.5 * curr.accelerations[j] * tt * tt;
#endif
                } else {
                    jtp.positions[j] = curr.positions[j] + alpha * smpl::shortest_angle_diff(next.positions[j], curr.positions[j]);
                }
            }

            // linearly interpolate single-dof joint velocities
            // for (auto j = 0; j < curr.velocities.size(); ++j) {
            //     jtp.velocities[j] = interp(curr.velocities[j], next.velocities[j], alpha);
            // }

            // linearly interpolate single-dof joint accelerations
            // for (auto j = 0; j < curr.accelerations.size(); ++j) {
            //     jtp.accelerations[j] = interp(curr.accelerations[j], next.accelerations[j], alpha);
            // }
            // interpolate (x, y) position, ignore z (should be 0)
            // mjtp.transforms[0].translation.x = interp(
            //         mcurr.transforms[0].translation.x,
            //         mnext.transforms[0].translation.x,
            //         alpha);
            // mjtp.transforms[0].translation.y = interp(
            //         mcurr.transforms[0].translation.y,
            //         mnext.transforms[0].translation.y,
            //         alpha);

            // linearly interpolate the yaw only
            // auto src_yaw = get_nearest_planar_rotation(mcurr.transforms[0].rotation);
            // auto dst_yaw = get_nearest_planar_rotation(mnext.transforms[0].rotation);
            // auto mid_yaw = interp(src_yaw, dst_yaw, alpha);

            // Eigen::Quaterniond q(Eigen::AngleAxisd(mid_yaw, Eigen::Vector3d::UnitZ()));
            // tf::quaternionEigenToMsg(q, mjtp.transforms[0].rotation);

            // mjtp.transforms[0] = interp(mcurr.transforms[0], mnext.transforms[0], alpha);

            // interpolate multi-dof joint velocities and accelerations
            // if (!mjtp.velocities.empty()) {
            //     mjtp.velocities[0] = interp(mcurr.velocities[0], mnext.velocities[0], alpha);
            // }
            // if (!mjtp.accelerations.empty()) {
            //     mjtp.accelerations[0] = interp(mcurr.accelerations[0], mnext.accelerations[0], alpha);
            // }

            // interpolate timestamp
            jtp.time_from_start = curr.time_from_start + ros::Duration(alpha * dt);
            // mjtp.time_from_start = curr.time_from_start + ros::Duration(alpha * dt);
            return RobotTrajectoryPoint{ std::move(jtp), std::move(mjtp) };
        }
    }

    printf("last time %f, sample time %f\n",
            traj->joint_trajectory.points.back().time_from_start.toSec(), t);
    ROS_WARN("Trajectory segment not found");
    return RobotTrajectoryPoint {
            traj->joint_trajectory.points.back(),
            traj->multi_dof_joint_trajectory.points.back()
    };
}

auto MakeInterpolatedTrajectory(
    const moveit_msgs::RobotTrajectory* traj,
    double time_delta)
    -> moveit_msgs::RobotTrajectory
{
    ROS_DEBUG("Interpolate Trajectory of (%zu points)", traj->joint_trajectory.points.size());
    moveit_msgs::RobotTrajectory out;

    assert(traj->joint_trajectory.points.size() ==
            traj->multi_dof_joint_trajectory.points.size());

    if (traj->joint_trajectory.points.empty()) {
        ROS_INFO(" -> Trivial (empty) trajectory");
        return out;
    }

    assert(traj->multi_dof_joint_trajectory.points.back().time_from_start ==
            traj->joint_trajectory.points.back().time_from_start);

    if (traj->joint_trajectory.points.size() == 1) {
        ROS_INFO(" -> Trivial (len = 1) trajectory");
        return *traj;
    }

    auto duration = traj->joint_trajectory.points.back().time_from_start.toSec();

    auto samples = std::max(2, (int)std::round(duration / time_delta) + 1);

    // clear the trajectory
    out = moveit_msgs::RobotTrajectory{ };

    out.joint_trajectory.header = traj->joint_trajectory.header;
    out.joint_trajectory.joint_names = traj->joint_trajectory.joint_names;
    out.joint_trajectory.points.resize(samples);

    out.multi_dof_joint_trajectory.header = traj->multi_dof_joint_trajectory.header;
    out.multi_dof_joint_trajectory.joint_names = traj->multi_dof_joint_trajectory.joint_names;
    out.multi_dof_joint_trajectory.points.resize(samples);

    for (auto i = 0; i < samples; ++i) {
        auto t = duration * double(i) / double(samples - 1);
        auto p = Sample(traj, t);

        out.joint_trajectory.points[i] = std::move(p.jtp);
        out.multi_dof_joint_trajectory.points[i] = std::move(p.mdjtp);
    }

    ROS_DEBUG(" -> Interpolated trajectory has (%zu, %zu) waypoints", out.joint_trajectory.points.size(), out.multi_dof_joint_trajectory.points.size());
    return out;
}
#endif

static
auto Sample(const std::vector<smpl::RobotState>& path, double t)
    -> smpl::RobotState
{
    assert(path.size() > 1);
    smpl::RobotState state(path[0].size());
    int num_joints = state.size() - 1;
    for (auto i = 1; i < path.size(); ++i) {
        auto& curr = path[i - 1];
        auto& next = path[i];

        double e = 0.0001;
        if (curr.back() - e <= t && t <= next.back() + e) {
            // auto jtp = trajectory_msgs::JointTrajectoryPoint();

            // jtp.positions.resize(curr.positions.size());

            auto dt = next.back() - curr.back();
            // auto dt = next.time_from_start.toSec() - curr.time_from_start.toSec();
            assert(dt >= 0.0);
            auto alpha = dt >= 1e-6 ? (t - curr.back()) / dt : 0.0;

            // linearly interpolate single-dof joint positions
            for (auto j = 0; j < num_joints; ++j) {
                if (j < num_joints - 4) {
#if 1
                    state[j] = interp(curr[j], next[j], alpha);
#else
                    auto tt = dt * alpha;
                    jtp.positions[j] = curr.positions[j] + curr.velocities[j] * tt + 0.5 * curr.accelerations[j] * tt * tt;
#endif
                } else {
                    state[j] = curr[j] + alpha * smpl::shortest_angle_diff(next[j], curr[j]);
                }
            }

            // interpolate timestamp
            state.back() = curr.back() + ros::Duration(alpha * dt).toSec();
            return state;
        }
    }

    ROS_WARN("Trajectory segment not found");
    return path.back();
}


auto MakeInterpolatedTrajectory(
    const std::vector<smpl::RobotState>& path,
    double time_delta)
    -> std::vector<smpl::RobotState>
{
    std::vector<smpl::RobotState> interp_path;
    double duration = path.back().back() - path.front().back();
    auto samples = std::max(2, (int)std::round(duration / time_delta) + 1);
    for (auto i = 0; i < samples; ++i) {
        auto t = duration * double(i) / double(samples - 1);
        auto p = Sample(path, t);
        interp_path.push_back(p);
    }
    return interp_path;
}