////////////////////////////////////////////////////////////////////////
//
// Copyright 2018 Carnegie Mellon University.  All rights reserved.
//
// NREC Confidential and Proprietary
//
// This notice must appear in all copies of this file and its derivatives
//
////////////////////////////////////////////////////////////////////////

#ifndef CONVEYOR_MANIPULATION_PLANNER_CONVERSIONS_H
#define CONVEYOR_MANIPULATION_PLANNER_CONVERSIONS_H

// system includes
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

// project includes
// #include <genx_motion_planner/joint_group.h>


namespace smpl {
namespace urdf {
struct RobotState;
}
}
// using RobotPath = std::vector<RobotState>;

// namespace genx {

// auto CreateRobotStateFromState(const RobotState& state)
//     -> moveit_msgs::RobotState;

// auto CreateRobotStateFromRobotStateMsg(const moveit_msgs::RobotState& msg)
//     -> RobotState;

// auto CreateRobotStateFromTrajectoryWaypointMsg(
//     const trajectory_msgs::JointTrajectoryPoint& jtp,
//     const trajectory_msgs::MultiDOFJointTrajectoryPoint& mdjtp)
//     -> RobotState;

// auto MakeTrajectory(
//     const RobotPath* path,
//     const RobotState* vel_limits,
//     const RobotState* acc_limits)
    // -> RobotTrajectory;

// auto InterpolateTrajectory(const RobotTrajectory* traj, double dt)
//     -> RobotTrajectory;

// void ConvertRobotPathToRobotTrajectoryMsg(
//     const RobotState* itraj_start,
//     const RobotPath* itraj,
//     const RobotState* vel_limits,
//     const RobotState* acc_limits,
//     moveit_msgs::RobotTrajectory* otraj);

auto MakeInterpolatedTrajectory(
    const std::vector<smpl::RobotState>& path,
    double time_delta)
    -> std::vector<smpl::RobotState>;

// void ConvertRobotPathToRobotTrajectoryMsg(
//     const RobotPath* itraj,
//     const RobotState* vel_limits,
//     const RobotState* acc_limits,
//     moveit_msgs::RobotTrajectory* otraj);

// void ConvertRobotTrajectoryMsgToFollowJointTrajectoryGoal(
//     const moveit_msgs::RobotTrajectory* traj,
//     control_msgs::FollowJointTrajectoryActionGoal* goal);

// void SetRobotState(smpl::urdf::RobotState* urdf_state, const RobotState* state);
// void SetRobotState(RobotState* state, const smpl::urdf::RobotState* urdf_state);

// } // namespace genx

#endif

