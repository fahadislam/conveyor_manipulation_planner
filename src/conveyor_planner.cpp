////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Fahad Islam
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
//     1. Redistributions of source code must retain the above copyright notice
//        this list of conditions and the following disclaimer.
//     2. Redistributions in binary form must reproduce the above copyright
//        notice, this list of conditions and the following disclaimer in the
//        documentation and/or other materials provided with the distribution.
//     3. Neither the name of the copyright holder nor the names of its
//        contributors may be used to endorse or promote products derived from
//        this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
////////////////////////////////////////////////////////////////////////////////

/// \author Fahad Islam
// standard includes
#include <sched.h>
#include <omp.h>

#include "conveyor_planner.h"

#include "conveyor_kdl_robot_model.h"
#include "conveyor_manip_checker.h"
#include "conveyor_object_model.h"

#include <boost/filesystem.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/post_processing.h>
#include <smpl/ros/factories.h>
#include <smpl/stl/memory.h>
#include <smpl/time.h>
#include <smpl/angles.h>
#include <random>

using namespace std;
// #include "conversions.h"

const char* CP_LOGGER = "conveyor";

struct ConveyorManipLatticeActionSpaceParams
{
    std::string mprim_filename;
    bool use_multiple_ik_solutions = false;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    double xyz_snap_thresh;
    double rpy_snap_thresh;
    double xyzrpy_snap_thresh;
    double short_dist_mprims_thresh;
};

// Lookup parameters for ManipLatticeActionSpace, setting reasonable defaults
// for missing parameters. Return false if any required parameter is not found.
static
bool GetConveyorManipLatticeActionSpaceParams(
    ConveyorManipLatticeActionSpaceParams& params,
    const smpl::PlanningParams& pp)
{
    if (!pp.getParam("manip_mprim_filename", params.mprim_filename)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'mprim_filename' not found in planning params");
        return false;
    }

    pp.param("use_multiple_ik_solutions", params.use_multiple_ik_solutions, false);

    pp.param("use_xyz_snap_mprim", params.use_xyz_snap_mprim, false);
    pp.param("use_rpy_snap_mprim", params.use_rpy_snap_mprim, false);
    pp.param("use_xyzrpy_snap_mprim", params.use_xyzrpy_snap_mprim, false);
    pp.param("use_short_dist_mprims", params.use_short_dist_mprims, false);

    pp.param("xyz_snap_dist_thresh", params.xyz_snap_thresh, 0.0);
    pp.param("rpy_snap_dist_thresh", params.rpy_snap_thresh, 0.0);
    pp.param("xyzrpy_snap_dist_thresh", params.xyzrpy_snap_thresh, 0.0);
    pp.param("short_dist_mprims_thresh", params.short_dist_mprims_thresh, 0.0);
    return true;
}

template <class T>
static auto ParseMapFromString(const std::string& s)
    -> std::unordered_map<std::string, T>
{
    std::unordered_map<std::string, T> map;
    std::istringstream ss(s);
    std::string key;
    T value;
    while (ss >> key >> value) {
        map.insert(std::make_pair(key, value));
    }
    return map;
}

static
bool MakeConveyorManipLatticeEGraph(
	smpl::ConveyorManipLatticeEgraph* graph,
	smpl::ConveyorManipLatticeActionSpace* actions,
    ConveyorKDLRobotModel* robot,
    ConveyorManipChecker* checker,
    const smpl::PlanningParams* params,
    const smpl::OccupancyGrid* grid,
    const Eigen::Vector3d& object_velocity)
{
    ////////////////
    // Parameters //
    ////////////////

    std::vector<double> resolutions(robot->jointVariableCount() * 2 + 1);

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'discretization' not found in planning params");
        return false;
    }
    auto disc = ParseMapFromString<double>(disc_string);
    ROS_DEBUG_NAMED(CP_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == end(disc)) {
            ROS_ERROR_NAMED(CP_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return false;
        }
        resolutions[vidx] = dit->second;
    }

    // velocity
    auto dit_v = disc.find("velocity");
    for (size_t vidx = robot->jointVariableCount(); vidx < robot->jointVariableCount() * 2; ++vidx) {
        resolutions[vidx] = dit_v->second;
    }

    // time dimension
    auto dit = disc.find("time");
    if (dit == end(disc)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Discretization for variable 'time' not found in planning parameters");
        return false;
    }
    resolutions[robot->jointVariableCount() * 2] = dit->second;

    ConveyorManipLatticeActionSpaceParams action_params;
    if (!GetConveyorManipLatticeActionSpaceParams(action_params, *params)) {
        return false; // errors logged within
    }

    ////////////////////
    // Initialization //
    ////////////////////

    if (!graph->init(robot, checker, resolutions, actions)) {
        ROS_ERROR("Failed to initialize Conveyor Manip Lattice Egraph");
        return false;
    }

    if (!actions->init(graph)) {
        ROS_ERROR("Failed to initialize Conveyor Manip Lattice Action Space");
        return false;
    }

    // auto& actions = actions;
    actions->useMultipleIkSolutions(action_params.use_multiple_ik_solutions);
    actions->useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ, action_params.use_xyz_snap_mprim);
    actions->useAmp(smpl::MotionPrimitive::SNAP_TO_RPY, action_params.use_rpy_snap_mprim);
    actions->useAmp(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.use_xyzrpy_snap_mprim);
    actions->useAmp(smpl::MotionPrimitive::SHORT_DISTANCE, action_params.use_short_dist_mprims);
    actions->ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ, action_params.xyz_snap_thresh);
    actions->ampThresh(smpl::MotionPrimitive::SNAP_TO_RPY, action_params.rpy_snap_thresh);
    actions->ampThresh(smpl::MotionPrimitive::SNAP_TO_XYZ_RPY, action_params.xyzrpy_snap_thresh);
    actions->ampThresh(smpl::MotionPrimitive::SHORT_DISTANCE, action_params.short_dist_mprims_thresh);
    if (!actions->load(action_params.mprim_filename)) {
        ROS_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
        return false;
    }

    // set conveyor velocity
    graph->setObjectVelocity(object_velocity);

    return true;
}

static
void ConvertJointVariablePathToJointTrajectory(
    smpl::RobotModel* robot,
    const std::vector<smpl::RobotState>& path,
    const std::string& joint_state_frame,
    const std::string& multi_dof_joint_state_frame,
    const moveit_msgs::RobotState& start_state,
    moveit_msgs::RobotTrajectory& traj)
{
    // ROS_INFO("Convert Variable Path to Robot Trajectory");

    int num_joints = robot->jointVariableCount();
    std::vector<SingleJointTrajectory> joint_trajs(num_joints);
    for (size_t i = 0; i < num_joints; ++i) {
        joint_trajs[i].positions.resize(path.size(), 0.0);
        joint_trajs[i].velocities.resize(path.size(), 0.0);
        joint_trajs[i].accelerations.resize(path.size(), 0.0);
        for (size_t j = 0; j < path.size(); ++j) {
            joint_trajs[i].positions[j] = path[j][i];
        }
    }

    std::vector<double> time_diff(path.size() -1, std::numeric_limits<double>::epsilon());
    for (size_t i = 0; i < path.size() - 1; ++i) {
        time_diff[i] = path[i + 1].back() - path[i].back();
    }

    // heuristic profiling
    for (size_t i = 0; i < num_joints; ++i) {
        for (size_t j = 1; j < path.size() - 2; ++j) {
            double dp_prev = (path[j][i] - path[j - 1][i]) / time_diff[j - 1];
            double dp_next = (path[j + 1][i] - path[j][i]) / time_diff[j];
            if (dp_prev * dp_next < 0.0) {
                joint_trajs[i].velocities[j] = 0.0;
            }
            else {
                joint_trajs[i].velocities[j] = (dp_prev + dp_next) / 2;
            }
        }
        joint_trajs[i].velocities[0] = 0.0;
        joint_trajs[i].velocities.back() = 0.0;

        // make the velocity at the replan state continuous
        if (start_state.joint_state.velocity.size() != 0) {
            joint_trajs[i].velocities[0] = start_state.joint_state.velocity[i + 1];
        }
    }

    //--------------------------------------
    traj.joint_trajectory.header.frame_id = joint_state_frame;
    traj.multi_dof_joint_trajectory.header.frame_id = multi_dof_joint_state_frame;

    traj.joint_trajectory.joint_names.clear();
    traj.joint_trajectory.points.clear();
    traj.multi_dof_joint_trajectory.joint_names.clear();
    traj.multi_dof_joint_trajectory.points.clear();

    // fill joint names header for both single- and multi-dof joint trajectories
    auto& variable_names = robot->getPlanningJoints();
    for (auto& var_name : variable_names) {
        std::string joint_name;
        if (smpl::IsMultiDOFJointVariable(var_name, &joint_name)) {
            auto it = std::find(
                    begin(traj.multi_dof_joint_trajectory.joint_names),
                    end(traj.multi_dof_joint_trajectory.joint_names),
                    joint_name);
            if (it == end(traj.multi_dof_joint_trajectory.joint_names)) {
                // avoid duplicates
                traj.multi_dof_joint_trajectory.joint_names.push_back(joint_name);
            }
        } else {
            traj.joint_trajectory.joint_names.push_back(var_name);
        }
    }

    // ROS_INFO("  Path includes %zu single-dof joints and %zu multi-dof joints",
    //         traj.joint_trajectory.joint_names.size(),
    //         traj.multi_dof_joint_trajectory.joint_names.size());

    // empty or number of points in the path
    if (!traj.joint_trajectory.joint_names.empty()) {
        traj.joint_trajectory.points.resize(path.size());
    }
    // empty or number of points in the path
    if (!traj.multi_dof_joint_trajectory.joint_names.empty()) {
        traj.multi_dof_joint_trajectory.points.resize(path.size());
    }

    for (size_t pidx = 0; pidx < path.size(); ++pidx) {
        auto& point = path[pidx];

        for (size_t vidx = 0; vidx < 7; ++vidx) {
            auto& var_name = variable_names[vidx];

            std::string joint_name, local_name;
            if (smpl::IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto& p = traj.multi_dof_joint_trajectory.points[pidx];
                p.transforms.resize(traj.multi_dof_joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.multi_dof_joint_trajectory.joint_names),
                        end(traj.multi_dof_joint_trajectory.joint_names),
                        joint_name);
                if (it == end(traj.multi_dof_joint_trajectory.joint_names)) continue;

                auto tidx = std::distance(begin(traj.multi_dof_joint_trajectory.joint_names), it);

                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    p.transforms[tidx].translation.x = point[vidx];
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    p.transforms[tidx].translation.y = point[vidx];
                } else if (local_name == "trans_z") {
                    p.transforms[tidx].translation.z = point[vidx];
                } else if (local_name == "theta") {
                    Eigen::Quaterniond q(Eigen::AngleAxisd(point[vidx], Eigen::Vector3d::UnitZ()));
                    tf::quaternionEigenToMsg(q, p.transforms[tidx].rotation);
                } else if (local_name == "rot_w") {
                    p.transforms[tidx].rotation.w = point[vidx];
                } else if (local_name == "rot_x") {
                    p.transforms[tidx].rotation.x = point[vidx];
                } else if (local_name == "rot_y") {
                    p.transforms[tidx].rotation.y = point[vidx];
                } else if (local_name == "rot_z") {
                    p.transforms[tidx].rotation.z = point[vidx];
                } else {
                    ROS_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }
            } else {
                auto& p = traj.joint_trajectory.points[pidx];
                p.positions.resize(traj.joint_trajectory.joint_names.size());
                p.velocities.resize(traj.joint_trajectory.joint_names.size());
                p.accelerations.resize(traj.joint_trajectory.joint_names.size());

                auto it = std::find(
                        begin(traj.joint_trajectory.joint_names),
                        end(traj.joint_trajectory.joint_names),
                        var_name);
                if (it == end(traj.joint_trajectory.joint_names)) continue;

                auto posidx = std::distance(begin(traj.joint_trajectory.joint_names), it);

                p.positions[posidx] = point[vidx];
                p.velocities[posidx] = joint_trajs[vidx].velocities[pidx];
                // p.accelerations[posidx] = joint_trajs[vidx].accelerations[pidx];
                p.time_from_start = ros::Duration(point.back());
            }
        }
    }
}

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
        // printf("t %f curr %f next %f\n", t, curr.back(), next.back());
        double e = 1e-6;
        if (curr.back() - e <= t && t <= next.back() + e) {
            // auto jtp = trajectory_msgs::JointTrajectoryPoint();

            // jtp.positions.resize(curr.positions.size());

            auto dt = next.back() - curr.back();
            // auto dt = next.time_from_start.toSec() - curr.time_from_start.toSec();
            assert(dt >= 0.0);
            auto alpha = dt >= 1e-6 ? (t - curr.back()) / dt : 0.0;

            // linearly interpolate single-dof joint positions
            for (auto j = 0; j < num_joints; ++j) {
                state[j] = curr[j] + alpha * smpl::shortest_angle_diff(next[j], curr[j]);
                // state[j] = curr[j] + alpha * (next[j] - curr[j]);
            }

            // interpolate timestamp
            // state.back() = curr.back() + ros::Duration(alpha * dt).toSec();
            state.back() = t;
            return state;
        }
    }
    ROS_WARN("Trajectory segment not found");
    ROS_WARN("t sample: %f t last: %f\n", t, path.back().back());
    return path.back();
}

// static
// auto MakeInterpolatedTrajectory(
//     const std::vector<smpl::RobotState>& path,
//     double time_delta)
//     -> std::vector<smpl::RobotState>
// {
//     std::vector<smpl::RobotState> interp_path;
//     double duration = path.back().back() - path.front().back();
//     auto samples = std::max(2, (int)std::round(duration / time_delta) + 1);
//     for (auto i = 0; i < samples; ++i) {
//         auto t = duration * double(i) / double(samples - 1);
//         auto p = Sample(path, t);
//         interp_path.push_back(p);
//     }
//     return interp_path;
// }

static
auto MakeInterpolatedTrajectory(
    const std::vector<smpl::RobotState>& path,
    double time_delta,
    double t_max)
    -> std::vector<smpl::RobotState>
{
    std::vector<smpl::RobotState> interp_path;
    double duration = path.back().back() - path.front().back();
    auto samples = std::max(2, (int)std::round((t_max - path[0].back()) / time_delta));
    for (auto i = 0; i < samples; ++i) {
        // auto t = i * time_delta;
        auto t = path[0].back() + (i * time_delta);
        auto p = Sample(path, t);
        interp_path.push_back(p);
    }

    std::vector<smpl::RobotState> postreplan;
    for (const auto& wp : path) {
        if (wp.back() >= t_max) {
            postreplan.push_back(wp);
        }
    }
    // skip the first wp as it's part of prereplan also
    for (size_t i = 0; i < postreplan.size(); ++i) {
        interp_path.push_back(postreplan[i]);
    }
    return interp_path;
}

static
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
    for (i = 1; i < n - 1; i++) {
        x1[i] = (x[i + 1] - x[i]) / dt[i] - (2 * x2[i] + x2[i + 1]) * dt[i] / 6.0;
    }
    x1[n - 1] = x1_f;
}

static
bool WritePath(
    smpl::RobotModel* robot,
    const moveit_msgs::RobotState& ref,
    const moveit_msgs::RobotTrajectory& traj,
    const std::string& path,
    double intercept_time)
{
    boost::filesystem::path p(path);

    try {
        if (!boost::filesystem::exists(p)) {
            ROS_DEBUG("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            ROS_ERROR("Failed to log path. %s is not a directory", path.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        ROS_ERROR("Failed to create plan output directory %s", p.native().c_str());
        return false;
    }

    std::stringstream ss_filename;
    auto now = smpl::clock::now();
    ss_filename << "path_" << now.time_since_epoch().count();
    p /= ss_filename.str();

    std::ofstream ofs(p.native());
    if (!ofs.is_open()) {
        return false;
    }

    ROS_DEBUG("Log path to %s", p.native().c_str());

    // write header
    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& var_name = robot->getPlanningJoints()[vidx];
        ofs << var_name; // TODO: sanitize variable name for csv?
        if (vidx != robot->jointVariableCount() - 1) {
            ofs << ',';
        }
    }
    ofs << ',' << "time_from_start";
    ofs << '\n';

    auto wp_count = std::max(
            traj.joint_trajectory.points.size(),
            traj.multi_dof_joint_trajectory.points.size());
    for (size_t widx = 0; widx < wp_count; ++widx) {

        if (traj.joint_trajectory.points[widx].time_from_start.toSec() >= intercept_time + 1e-6) {
            continue;
        }
        // fill the complete robot state
        moveit_msgs::RobotState state = ref;

        if (widx < traj.joint_trajectory.points.size()) {
            auto& wp = traj.joint_trajectory.points[widx];
            auto joint_count = traj.joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                auto& joint_name = traj.joint_trajectory.joint_names[jidx];
                auto vp = wp.positions[jidx];
                auto it = std::find(
                        begin(state.joint_state.name),
                        end(state.joint_state.name),
                        joint_name);
                if (it != end(state.joint_state.name)) {
                    auto tvidx = std::distance(begin(state.joint_state.name), it);
                    state.joint_state.position[tvidx] = vp;
                }
            }
        }
        if (widx < traj.multi_dof_joint_trajectory.points.size()) {
            auto& wp = traj.multi_dof_joint_trajectory.points[widx];
            auto joint_count = traj.multi_dof_joint_trajectory.joint_names.size();
            for (size_t jidx = 0; jidx < joint_count; ++jidx) {
                auto& joint_name = traj.multi_dof_joint_trajectory.joint_names[jidx];
                auto& t = wp.transforms[jidx];
                auto it = std::find(
                        begin(state.multi_dof_joint_state.joint_names),
                        end(state.multi_dof_joint_state.joint_names),
                        joint_name);
                if (it != end(state.multi_dof_joint_state.joint_names)) {
                    size_t tvidx = std::distance(begin(state.multi_dof_joint_state.joint_names), it);
                    state.multi_dof_joint_state.transforms[tvidx] = t;
                }
            }
        }

        // write the planning variables out to file
        for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
            auto& var_name = robot->getPlanningJoints()[vidx];

            std::string joint_name, local_name;
            if (smpl::IsMultiDOFJointVariable(var_name, &joint_name, &local_name)) {
                auto it = std::find(
                        begin(state.multi_dof_joint_state.joint_names),
                        end(state.multi_dof_joint_state.joint_names),
                        joint_name);
                if (it == end(state.multi_dof_joint_state.joint_names)) continue;

                auto jidx = std::distance(begin(state.multi_dof_joint_state.joint_names), it);
                auto& transform = state.multi_dof_joint_state.transforms[jidx];
                double pos;
                if (local_name == "x" ||
                    local_name == "trans_x")
                {
                    pos = transform.translation.x;
                } else if (local_name == "y" ||
                    local_name == "trans_y")
                {
                    pos = transform.translation.y;
                } else if (local_name == "trans_z") {
                    pos = transform.translation.z;
                } else if (local_name == "theta") {
                    // this list just gets larger:
                    // from sbpl_collision_checking, MoveIt, Bullet, leatherman
                    double s_squared = 1.0 - transform.rotation.w * transform.rotation.w;
                    if (s_squared < 10.0 * std::numeric_limits<double>::epsilon()) {
                        pos = 0.0;
                    } else {
                        double s = 1.0 / sqrt(s_squared);
                        pos = (acos(transform.rotation.w) * 2.0) * transform.rotation.x * s;
                    }
                } else if (local_name == "rot_w") {
                    pos = transform.rotation.w;
                } else if (local_name == "rot_x") {
                    pos = transform.rotation.x;
                } else if (local_name == "rot_y") {
                    pos = transform.rotation.y;
                } else if (local_name == "rot_z") {
                    pos = transform.rotation.z;
                } else {
                    ROS_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
                    continue;
                }

                ofs << pos;
            } else {
                auto it = std::find(
                        begin(state.joint_state.name),
                        end(state.joint_state.name),
                        var_name);
                if (it == end(state.joint_state.name)) continue;

                auto tvidx = std::distance(begin(state.joint_state.name), it);
                auto vp = state.joint_state.position[tvidx];
                ofs << vp;
            }

            if (vidx != robot->jointVariableCount() - 1) {
                ofs << ',';
            }
        }
        ofs << ',' << traj.joint_trajectory.points[widx].time_from_start.toSec();
        // printf("widx %zu time %f\n", widx, traj.joint_trajectory.points[widx].time_from_start.toSec());
        ofs << '\n';
    }

    return true;
}

static
bool ReinitDijkstras(
	ConveyorPlanner* planner,
	const ObjectState& start_state)
{
    planner->hkey_dijkstra.force_planning_from_scratch();
    planner->hkey_dijkstra.set_search_mode(false);

    // const ObjectState start = planner->object_graph.getDiscreteCenter(
    //             { start_state[0], start_state[1], start_state[2] });

    const ObjectState start = { start_state[0], start_state[1], start_state[2] };

    // Will set the start and goal both as start state
    // We set he goal as start to get the heuristic value w.r.t. the start

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::JOINT_STATE_GOAL;
    goal.angles = start;
    goal.angle_tolerances = { 0.0, 0.0, 0.0 };  // irrelevant

    if (!planner->object_graph.setGoal(goal)) {
        ROS_ERROR("Failed to set object goal");
        return 1;
    }

    if (!planner->object_graph.setStart(start)) {
        ROS_ERROR("Failed to set object start");
        return 1;
    }

    int start_id = planner->object_graph.getStartStateID();
    if (start_id < 0) {
        ROS_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = planner->object_graph.getGoalStateID();
    if (goal_id < 0)  {
        ROS_ERROR("Goal state id is invalid");
        return 1;
    }

    if (planner->hkey_dijkstra.set_start(start_id) == 0) {
        ROS_ERROR("Failed to set planner start state");
        return 1;
    }

    if (planner->hkey_dijkstra.set_goal(goal_id) == 0) {
        ROS_ERROR("Failed to set planner goal state");
        return 1;
    }

    if (!planner->hkey_dijkstra.reinit_search()) {
        ROS_ERROR("Failed to initialize Dijsktras");
        return 1;
    }
}

bool Init(
	ConveyorPlanner* planner,
    ConveyorBoundsChecker* object_checker,
    ConveyorManipChecker* manip_checker,
    ConveyorObjectModel* object_model,
    ConveyorKDLRobotModel* robot_model,
    smpl::OccupancyGrid* object_grid,
    smpl::OccupancyGrid* manip_grid,
    const moveit_msgs::RobotState home_state,
    const Eigen::Vector3d& object_velocity,
    const smpl::PlanningParams* params,
    bool preprocessing)	// pass params
{
    // int cpu_num = sched_getcpu();
    // cpu_num = 3;
    // ROS_INFO("cpu_num: %d\n", cpu_num);
    // planner->main_dir_ = "/home/fislam/conveyor_data_" + std::to_string(cpu_num);
    planner->main_dir_ = "/home/fislam/conveyor_data_robot"; 
    if (!boost::filesystem::exists(planner->main_dir_)) {
        // ROS_INFO("Create plan output directory %s", start_dir.native().c_str());
        boost::filesystem::create_directory(planner->main_dir_);
    }

	planner->object_checker = object_checker;
	planner->manip_checker = manip_checker;
	planner->object_model = object_model;
	planner->robot_model = robot_model;
    planner->home_state_ = home_state;

    //////////////////////////////////
    //   Initialize Object Planner  //
    //////////////////////////////////

	// 1. Init graph
    double res_xy;     // 1cm
    double res_yaw;     // 5 degrees
    double origin_x;
    double origin_y;

    if (!params->getParam("resolution_xy", res_xy)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'resolution_xy' not found in planning params");
        return false;
    }
    if (!params->getParam("resolution_yaw", res_yaw)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'resolution_yaw' not found in planning params");
        return false;
    }
    if (!params->getParam("origin_x", origin_x)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'origin_x' not found in planning params");
        return false;
    }
    if (!params->getParam("origin_y", origin_y)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'origin_y' not found in planning params");
        return false;
    }

    if (!params->getParam("interp_resolution", planner->interp_resolution_)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'interp_resolution_' not found in planning params");
        return false;
    }

    if (!params->getParam("replan_resolution", planner->replan_resolution_)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'replan_resolution' not found in planning params");
        return false;
    }

    if (!params->getParam("time_bound", planner->time_bound_)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'time_bound' not found in planning params");
        return false;
    }

    if (!params->getParam("replan_cutoff", planner->replan_cutoff_)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'replan_cutoff' not found in planning params");
        return false;
    }

    std::vector<double> object_resolutions = { res_xy, res_xy, res_yaw };
    if (!planner->object_graph.init(
    		planner->object_model,
    		planner->object_checker,
    		object_resolutions,
    		&planner->object_actions)) {
        ROS_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    // 2. Init actions
    if (!planner->object_actions.init(&planner->object_graph)) {
        ROS_ERROR("Failed to initialize Manip Lattice Action Space");
        return 1;
    }
	std::string object_mprim_path;
    if (!params->getParam("object_mprim_filename", object_mprim_path)) {
        ROS_ERROR_NAMED(CP_LOGGER, "Parameter 'object_mprim_filename' not found in planning params");
        return false;
    }
    // load primitives from file, whose path is stored on the param server
    if (!planner->object_actions.load(object_mprim_path)) {
        return 1;
    }

    // 3. Init heuristic
    if (!planner->object_heuristic.init(&planner->object_graph)) {
        ROS_ERROR("Failed to initialize Joint Dist Heuristic");
        return false;
    }

    // 4. Init search done in constructor

    // 5. Create all object states
	ObjectState object_state = {origin_x, origin_y, 0.0};
	ReinitDijkstras(planner, object_state);
    ReplanParams search_params(10.0);
    std::vector<int> solution;
    int solcost;
	bool bret = planner->hkey_dijkstra.replan(&solution, search_params, &solcost);

    //////////////////////////////////
    //   Initialize Robot Planner   //
    //////////////////////////////////

    // 1 & 2. Init graph and actions
    MakeConveyorManipLatticeEGraph(
    		&planner->manip_graph,
    		&planner->manip_actions,
			planner->robot_model,
			planner->manip_checker,
			params,
			manip_grid,
			object_velocity);

    // 3. Init heuristic
    if (!planner->manip_heuristic.init(&planner->manip_graph)) {
        return false;
    }
    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
        ROS_ERROR("Failed to initialize Generic Egraph heuristic");
        return false;
    }
    double egw = 2.0;
    planner->egraph_manip_heuristic.setWeightEGraph(egw);

    // 4. Init search
    planner->egraph_planner = new smpl::ExperienceGraphPlanner(
						    			&planner->manip_graph,
						    			&planner->egraph_manip_heuristic);

    if (manip_grid) {
        planner->manip_graph.setVisualizationFrameId(manip_grid->getReferenceFrame());
    }

    // auto dir = "/home/fislam/conveyor_data/";

    if (!preprocessing) {
        planner->manip_graph.loadStartToMapIdMap(planner->main_dir_);
        planner->object_graph.loadStateToPathIdMap(planner->main_dir_);
    }
}

static
bool IsPathValid(smpl::CollisionChecker* checker, const std::vector<smpl::RobotState>& path)
{

    for (size_t i = 1; i < path.size(); ++i) {
        if (!checker->isStateToStateValid(path[i - 1], path[i])) {
            ROS_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
            return false;
        }
    }
    return true;
}

#if 0
static
void ShortcutPath(
    ConveyorPlanner* planner,
    double intercept_time,
    std::vector<smpl::RobotState>& path)
{
    // shortcut path piece-wise:
    //      start -> intercept -> grasp -> lift

    std::vector<smpl::RobotState> path_preintercept;
    std::vector<smpl::RobotState> path_postintercept;

    // printf("FULL PATH\n");
    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

    auto lift_state = path.back();
    path.pop_back();
    for (auto p : path) {
        if (p.back() < intercept_time) {
            path_preintercept.push_back(p);
        }
        else {
            path_postintercept.push_back(p);
        }
    }
    path_preintercept.push_back(path_postintercept[0]);

    path.clear();
    ShortcutPath(planner->robot_model,
            planner->manip_checker,
            path_preintercept,
            path,
            smpl::ShortcutType::JOINT_SPACE);
    path.pop_back();

    // printf("\nPREINTERCEPT\n");
    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

    // getchar();

    std::vector<smpl::RobotState> path_postintercept_sc;
    ShortcutPath(planner->robot_model,
            planner->manip_checker,
            path_postintercept,
            path_postintercept_sc,
            smpl::ShortcutType::JOINT_SPACE);

    for (auto p : path_postintercept_sc) {
        path.push_back(p);
    }

    // printf("\nPOSTINTERCEPT\n");
    // for (const auto& wp : path_postintercept_sc) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

    // printf("\nCOMBINED\n");
    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

    path.push_back(lift_state);

    // printf("AFTER LAST STATE\n");
    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }
}
#endif

static
void ShortcutPath(
	ConveyorPlanner* planner,
	double intercept_time,
	std::vector<smpl::RobotState>& path,
    bool shortcut_prerc = true)
{
    // shortcut path piece-wise:
    // 		start -> intercept -> grasp -> lift

    // printf("new path:\n");
    // for (const auto& wp : path) {
    //     if (wp.back() < 4.0) {
    //         ROS_INFO_STREAM("waypoint: " << wp);
    //     }
    // }

    std::vector<smpl::RobotState> path_prerc;
    std::vector<smpl::RobotState> path_postrc;
    if (!shortcut_prerc) {
        for (auto p : path) {
            if (p.back() < planner->replan_cutoff_) {
                path_prerc.push_back(p);
            }
            else {
                path_postrc.push_back(p);
            }
        }
        path_prerc.push_back(path_postrc[0]);
        path = path_postrc;
    }

    std::vector<smpl::RobotState> path_preintercept;
    std::vector<smpl::RobotState> path_postintercept;

    auto lift_state = path.back();
    path.pop_back();


    for (auto p : path) {
        if (p.back() < intercept_time) {
            path_preintercept.push_back(p);
        }
        else {
            path_postintercept.push_back(p);
        }
    }
    path_preintercept.push_back(path_postintercept[0]);

    path.clear();
    ShortcutPath(planner->robot_model,
            planner->manip_checker,
            path_preintercept,
            path,
            smpl::ShortcutType::JOINT_SPACE);
    path.pop_back();

    std::vector<smpl::RobotState> path_postintercept_sc;
    ShortcutPath(planner->robot_model,
            planner->manip_checker,
            path_postintercept,
            path_postintercept_sc,
            smpl::ShortcutType::JOINT_SPACE);

    for (auto p : path_postintercept_sc) {
        path.push_back(p);
    }

    if (!shortcut_prerc) {
        for (auto p : path) {
            path_prerc.push_back(p);
        }
        path = path_prerc;
    }

    path.push_back(lift_state);

}

static
void PostProcessPath(
    ConveyorPlanner* planner,
    std::vector<smpl::RobotState>& path,
    double resolution,
    double intercept_time,
    bool shortcut_prerc)
{
    ////////////////////
    // Shortcut Path  //
    ////////////////////

    ShortcutPath(planner, intercept_time, path, shortcut_prerc);
    /////////////////////////////////////////////////////////
    // Profile/Interpolate path and convert to trajectory  //
    /////////////////////////////////////////////////////////

    // Only need to interpolate until the time when we stop replanning

    // double last_replan_time = std::min(planner->replan_cutoff_, path.back().back());
    double interpt_until = path[path.size() - 2].back();
    path = MakeInterpolatedTrajectory(path, resolution, interpt_until);
    // printf("Size of interpolated path: %zu\n", path.size());
    // unwind path
    for (size_t i = 1; i < path.size(); ++i) {
        for (size_t j = 0; j < planner->robot_model->jointVariableCount(); ++j) {
            if (planner->robot_model->isContinuous(j)) {
                int n = std::round((path[i - 1][j] - path[i][j]) / (2 * M_PI));
                path[i][j] += n * 2 * M_PI;
            }
        }
    }
}

static
bool PlanRobotPath(
	ConveyorPlanner* planner,
	const moveit_msgs::RobotState& start_state,
    const Eigen::Affine3d& object_pose,
    std::vector<smpl::RobotState>& path,
    double& intercept_time,
    const PlanPathParams& params)
{
    //==============================================================================//
    // By default the collision object remains inflated                             //
    // We only deflate to find shortcut successors and to check final interpolation //
    //==============================================================================//

    ///////////////
    // Set Goal  //
    ///////////////
    bool b_ret = false;

    // Setting goal initially to be able to get intercept path for singleton path

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::XYZ_RPY_GOAL;
    goal.pose = object_pose;

    goal.xyz_tolerance[0] = 0.01; //0.015;
    goal.xyz_tolerance[1] = 0.005; //0.015;
    goal.xyz_tolerance[2] = 0.01; //0.015;
    goal.rpy_tolerance[0] = 0.0872665; //0.05;
    goal.rpy_tolerance[1] = 0.0872665; //0.05;
    goal.rpy_tolerance[2] = 0.0872665; //0.05;

    if (!planner->manip_graph.setGoal(goal)) {
        ROS_ERROR("Failed to set manip goal");
        return false;
    }

    std::vector<smpl::RobotState> new_path;
    if (!params.singleton_dir.empty()) {
    // if (false) {
		if (planner->manip_graph.loadPath(params.singleton_dir, path)) {
			b_ret = true;
            // printf("SINGLETON\n");
        }
	}
	else {
	    // planner->manip_checker->deflateCollisionObject();
	    planner->egraph_manip_heuristic.updateGoal(goal);
	    // planner->manip_checker->inflateCollisionObject();

	    if (params.rc_constrained) {
	        bool ret_cutoff = planner->egraph_manip_heuristic.isReplanCutoffBeforeShortcutNode(planner->replan_cutoff_);
	        if (!ret_cutoff) {
	            // ROS_WARN("Replan cutoff is after the shortcut node");
	            return false;
	        }
	    }

	    auto goal_id = planner->manip_graph.getGoalStateID();
	    if (goal_id == -1) {
	        ROS_ERROR("No goal state has been set");
	        return false;
	    }

	    if (planner->egraph_planner->set_goal(goal_id) == 0) {
	        ROS_ERROR("Failed to set planner goal state");
	        return false;
	    }

	    ////////////////
	    // Set Start  //
	    ////////////////

	    smpl::RobotState start;
	    std::vector<std::string> missing;
	    if (!leatherman::getJointPositions(
	            start_state.joint_state,
	            start_state.multi_dof_joint_state,
	            planner->robot_model->getPlanningJoints(),
	            start,
	            missing))
	    {	
	    	ROS_ERROR("Start state is missing planning joints");
	    	return false;
	    }

        // velocity
        // TODO: use from current state
        for (int i = 0; i < 7; ++i)
            start.push_back(0.0);

	    start.push_back(start_state.joint_state.position.back());	// time



	    if (!planner->manip_graph.setStart(start)) {
	        ROS_ERROR("Failed to set start state");
	        return false;
	    }

	    auto start_id = planner->manip_graph.getStartStateID();
	    if (start_id == -1) {
	        ROS_ERROR("No start state has been set");
	        return false;
	    }

	    if (planner->egraph_planner->set_start(start_id) == 0) {
	        ROS_ERROR("Failed to set start state");
	        return false;
	    }

	    ////////////////
	    // Plan Path  //
	    ////////////////

	    std::vector<int> solution_state_ids;
	    
	    int sol_cost;
	    planner->egraph_planner->m_allowed_expansions = params.allowed_time * 250;
	    // ROS_INFO("Expansions bound: %d\n", planner->egraph_planner->m_allowed_expansions);
        // ROS_INFO("BEFORE REPLAN");
	    b_ret = planner->egraph_planner->replan(params.allowed_time, &solution_state_ids, &sol_cost);
        // ROS_INFO("AFTER REPLAN");
	    if (params.only_check_success) {
	        return b_ret;
	    }

	    if (b_ret && (solution_state_ids.size() > 0)) {
	        ROS_DEBUG_NAMED(CP_LOGGER, "Planning succeeded");
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Num Expansions (Initial): %d", planner->egraph_planner->get_n_expands_init_solution());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Num Expansions (Final): %d", planner->egraph_planner->get_n_expands());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Epsilon (Initial): %0.3f", planner->egraph_planner->get_initial_eps());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Epsilon (Final): %0.3f", planner->egraph_planner->get_solution_eps());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Time (Initial): %0.3f", planner->egraph_planner->get_initial_eps_planning_time());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Time (Final): %0.3f", planner->egraph_planner->get_final_eps_planning_time());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
	        ROS_DEBUG_NAMED(CP_LOGGER, "  Solution Cost: %d", sol_cost);

	        path.clear();
	        if (!planner->manip_graph.extractPath(solution_state_ids, path)) {
	            ROS_ERROR("Failed to convert state id path to joint variable path");
	            return false;
	        }
	    }
	    else {
	        return b_ret;
	    }
    }

    intercept_time = planner->manip_graph.getInterceptTime(path);

    if (intercept_time < 0.1) {
        printf("ERRORRRRRRRRRRRRRRRRRRRR\n");
        // printf("new path:\n");
        // for (const auto& wp : path) {
        //     ROS_INFO_STREAM("waypoint: " << wp);
        // }
        // getchar();
    }

    //=========================================================
    // If replan request then merge new path with current path
    // If first plan request then use the new path as it is
    //=========================================================

    // ROS_INFO("BEFORE PREPROCESS");
    // PostProcessPath(planner, path, planner->interp_resolution_, intercept_time, params.shortcut_prerc);
    // ROS_INFO("AFTER PREPROCESS");
    // printf("new path:\n");
    // for (const auto& wp : new_path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

    //=============================================================================================
    // NOTES:
    // - No need to shortcut or interpolate the path before "last_replan_time" for const time
    //   plan and replan queries
    // - Only shortcut the entire path for preprocessing
    // - Only interpolate path until last_replan_time
    //=============================================================================================


    // printf("Final path:\n");
    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

#if 0
    auto to_check_path = path;
    to_check_path.pop_back();
    // planner->manip_checker->deflateCollisionObject();
    if (!IsPathValid(planner->manip_checker, to_check_path)) {
        ROS_WARN("Path is invalid after interp");
        // getchar();
    }
#endif
    // planner->manip_checker->inflateCollisionObject();

    // no need for joint_trajs anymore as we are not using cubic profile


    return b_ret;
}

static
auto ComputeObjectPose(
	const ObjectState& object_state,
	double height)
	-> Eigen::Affine3d
{
	Eigen::Affine3d object_pose(
	                Eigen::Translation3d(object_state[0], object_state[1], height) *
	                Eigen::AngleAxisd(object_state[2], Eigen::Vector3d::UnitZ()) *
	                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
	                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));

    return object_pose;
}

bool PlanPathUsingRootPath(
    ConveyorPlanner* planner,
    const std::string& egraph_dir,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    const ObjectState& object_state_grid,
    double height,
    std::vector<smpl::RobotState>& path,
    double& intercept_time,
    const PlanPathParams& params)
{
    ROS_INFO("#######    PlanPathUsingRootPath: Query object state: %.2f %.2f %f   #######", 
            object_state_grid[0], object_state_grid[1], object_state_grid[2]);

    auto object_pose = ComputeObjectPose(object_state_grid, height);
    auto goal_pose = object_pose * grasps[0].inverse();

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(object_pose);

    // clear all memory
    planner->manip_graph.eraseExperienceGraph();
    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

    // load experience graph
    if (!egraph_dir.empty()) {
        planner->manip_graph.loadExperienceGraph(egraph_dir);
    }
    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
        ROS_ERROR("Failed to initialize Generic Egraph heuristic");
        return false;
    }

    if (!PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params)) {
        ROS_INFO("Unable to plan to the center within time %f",
        planner->time_bound_);
        return false;
    }

    return true;
}

bool ComputeRootPaths(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const int start_id,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    std::vector<std::vector<smpl::RobotState>>& paths,
    std::vector<int>& G_UNCOV,
    std::vector<int>& G_COV)
{
    /////////////////////////////////////////////
    /// Compute paths for goal region G_UNCOV ///
    /////////////////////////////////////////////

    // Clears dirty/covered statuses, clears dirty list, sub regions
    planner->hkey_dijkstra.setUncoveredStates(G_UNCOV);

    int center_count = 0;
    std::string start_dir = planner->main_dir_ + "/start_" + std::to_string(start_id);
    if (!boost::filesystem::exists(start_dir)) {
        // ROS_INFO("Create plan output directory %s", start_dir.native().c_str());
        boost::filesystem::create_directory(start_dir);
    }

    std::string rc_egraph_dir = start_dir + "/paths/root/";
    while(true) {

        ///////////////////////////////////////////////
        // Sample a random object goal: { x, y yaw } //
        ///////////////////////////////////////////////

        auto state_id = planner->hkey_dijkstra.sampleObjectState(center_count);
        if (state_id == -1) {
            break;
        }

        // remove state from uncovered list
        auto center_state = planner->object_graph.extractState(state_id);

        printf("\n");
        ROS_INFO("*******************************************************************");
        ROS_INFO("********   Object State: %.2f, %.2f, %f \t id: %d   ********",
                            center_state[0], center_state[1], center_state[2], state_id);
        ROS_INFO("*******************************************************************");

        //////////////////////////////////////////////////////
        // Compute path to object goal and store experience //
        //////////////////////////////////////////////////////

        moveit_msgs::RobotTrajectory root_traj;
        double intercept_time;

        PlanPathParams params;
        params.allowed_time = 20;
        params.rc_constrained = false;
        params.shortcut_prerc = false;
        params.only_check_success = false;
        if (center_count == 0) {
            params.shortcut_prerc = true;
        }

        auto egraph_dir = rc_egraph_dir;
        if (center_count == 0) {
            egraph_dir = "";
        }

        std::vector<smpl::RobotState> path;
        if (!PlanPathUsingRootPath(
                planner,
                egraph_dir,
                start_state,
                grasps,
                center_state,
                height,
                path,
                intercept_time,
                params)){
            planner->hkey_dijkstra.markDirtyState(state_id);
            planner->hkey_dijkstra.removeStateFromUncoveredList(state_id);
            continue;           
        }

        ConvertJointVariablePathToJointTrajectory(
            planner->robot_model,
            path,
            start_state.joint_state.header.frame_id,
            start_state.multi_dof_joint_state.header.frame_id,
            start_state,
            root_traj);

        // write path to file
        boost::filesystem::create_directory(start_dir + "/paths/");
        egraph_dir = start_dir + "/paths/" + std::to_string(center_count);
        WritePath(planner->robot_model, start_state, root_traj, egraph_dir, intercept_time);

        if (center_count == 0) {
            WritePath(planner->robot_model, start_state, root_traj, rc_egraph_dir, planner->replan_cutoff_);
        }

        //////////////////////////////////
        //     Reachability Search      //
        //////////////////////////////////

        ReinitDijkstras(planner, center_state);
        int covered_count = 0;
        int iter = 0;
        while (true) {

            // Get next object state"

            //  - For the first iteration it should be the center state
            auto state_id = planner->hkey_dijkstra.getNextStateId();
            if (state_id == -1) {   // OPEN empty or all states covered
                break;
            }
            else if (state_id == -2) {      // State already covered
                continue;
            }
            auto object_state = planner->object_graph.extractState(state_id);

            // ROS_INFO("     *********************************************************");
            // ROS_INFO("     ********   Next State: %.2f, %.2f, %f \t id: %d   ********",
            //                     object_state[0], object_state[1], object_state[2], state_id);
            // ROS_INFO("     *********************************************************");

            ///////////////////////////////////////
            //     Compute path to next state    //
            ///////////////////////////////////////

            PlanPathParams params;
            params.allowed_time = planner->time_bound_;
            params.rc_constrained = true;
            params.only_check_success = true;
            if (iter == 0) {
                params.rc_constrained = false;
                params.only_check_success = false;
                params.shortcut_prerc = true;
            }

            double intercept_time;
            std::vector<smpl::RobotState> path;
            if (!PlanPathUsingRootPath(
                    planner,
                    egraph_dir,
                    start_state,
                    grasps,
                    object_state,
                    height,
                    path,
                    intercept_time,
                    params)){
                //=========================================================================//
                // dont need to mark state as dirty here, can just store the original path //
                //=========================================================================//
                // ROS_INFO("     Pose is NOT reachable");
                // ROS_INFO("-----------------------------------------------------------\n\n\n");
                if (iter == 0) {
                    // printf("singleton\n");
                    // store path as it is
                    // flag goal id as singleton goal

                    // planner->hkey_dijkstra.markDirtyState(state_id);
                    planner->hkey_dijkstra.removeStateFromDirtyList(state_id); // if dirty
                    planner->hkey_dijkstra.removeStateFromUncoveredList(state_id);
                    planner->hkey_dijkstra.addStateToSubregion(center_count, state_id);
                    auto sys_ret = system(("exec rm -r " + egraph_dir + "/*").c_str());
                    // Store full path and set path id as singleton

                    // intercept_time will be 0 here because planner failed
                    // if (intercept_time < 0.1) {
                    //     printf("second\n");
                    //     getchar();
                    // }
                    WritePath(planner->robot_model, start_state, root_traj, egraph_dir, 1e3);
                    planner->object_graph.setPathId(start_id, state_id, center_count, true);
                }
            }
            else {
                if (iter == 0) {
                    // remove previous path and write path to file (to resolve mismatch)
                    // issue because of interpolation)
                    // auto sys_ret = system(("exec rm -r " + egraph_dir + "/*").c_str());
                    // WritePath(planner->robot_model, start_state, traj, egraph_dir, intercept_time);
        //             printf("second\n"); getchar();
                }
                planner->hkey_dijkstra.removeStateFromDirtyList(state_id); // if dirty
                planner->hkey_dijkstra.removeStateFromUncoveredList(state_id);
                planner->hkey_dijkstra.addStateToSubregion(center_count, state_id);
                planner->object_graph.setPathId(start_id, state_id, center_count);
                // ROS_INFO("     Pose is reachable, path id: %d", center_count);
                // ROS_INFO("-----------------------------------------------------------\n\n\n");
                // covered_count++;
            }
            iter++;
        }
        // if (covered_count > 0) {
        center_count++;
        if (planner->home_query) {
            planner->home_center_states_.push_back(center_state);
        }
        // }
        // ROS_INFO("No. center states %d", center_count);

        // save subregion
        // getchar();
    }
    // ROS_INFO("Preprocessing Completed");
    // ROS_INFO("No. center states %d", center_count);

    // Get remaining states
    G_UNCOV = planner->hkey_dijkstra.getUncoveredStates();
    // ROS_INFO("Remaining states:");
    // for (auto id : G_UNCOV) {
    //     ROS_INFO(" %d",id);
    // }
    auto G_COV_ = planner->hkey_dijkstra.getCoveredStates();
    // ROS_INFO("Covered states:");
    // for (auto id : G_COV_) {
    //     ROS_INFO(" %d",id);
    // }
    planner->hkey_dijkstra.addStates(G_COV, G_COV_);
    // Keep the paths from home state to be used for latching
    // Happens for the first call to Preprocess only
    // planner->object_graph.saveStateToPathIdMap(planner->home_query, start_id, start_dir);
    planner->hkey_dijkstra.appendSubregions();

    // Start to map id map stuff
    smpl::RobotState start;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            start_state.joint_state,
            start_state.multi_dof_joint_state,
            planner->robot_model->getPlanningJoints(),
            start,
            missing))
    {   
        ROS_ERROR("Start state is missing planning joints");
        return false;
    }
    start.push_back(start_state.joint_state.position.back());
    paths.resize(center_count);
    planner->manip_graph.loadPaths(start_dir + "/paths/", paths);
    if (!paths.empty()) {
        planner->manip_graph.setMapId(start, start_id);
    }
    if (planner->home_query) {
        planner->home_paths_ = paths;
    }
    if (planner->home_paths_.size() != planner->home_center_states_.size()) {
        ROS_ERROR("Problem"); getchar();
    }
    planner->home_query = false;
    //=============================================
    // Compute Root Paths End
    //=============================================
}

static 
int GetStateIndexAtTime(const std::vector<smpl::RobotState>& path, double t)
{
    // Returns state right after t;
    for (size_t i = 0; i < path.size(); ++i) {
        // printf("%f %f\n", path[i].back(), t);
        if (fabs(path[i].back() - t) < 1e-3) {
            return i;
        }
    }
    return -1;
}

static 
int GetStateIndexAfterTime(const std::vector<smpl::RobotState>& path, double t)
{
    // Returns state right after t;
    for (size_t i = 0; i < path.size(); ++i) {
        if (path[i].back() >= t) {
            return i;
        }
    }
}

bool CheckSnap(
    ConveyorPlanner* planner,
    const smpl::RobotState from_state,
    const std::vector<smpl::RobotState> path,
    double buffer) 
{
    auto to_idx = GetStateIndexAtTime(path, from_state.back() + planner->replan_resolution_);
    auto to_state = path[to_idx];
    // for (const auto& wp : path) {
    //     if (fabs(wp.back() - from_state.back()) >= planner->replan_resolution_ - 1e-3) {
    //         to_state = wp;
    //         break;
    //     }
    // }

    // ROS_INFO("Check snap from time %f to %f", from_state.back(), to_state.back());

    // TODO: Use this common function whereever
    double max_time = 0.0;
    double diff_time = to_state.back() - from_state.back(); 
    for (size_t j = 0; j < planner->robot_model->jointVariableCount(); ++j) {
        auto from_pos = from_state[j];
        auto to_pos = to_state[j];
        auto vel = planner->robot_model->velLimit(j);
        if (vel <= 0.0) {
            continue;
        }
        auto t = 0.0;
        if (planner->robot_model->isContinuous(j)) {
            t = smpl::angles::shortest_angle_dist(from_pos, to_pos) / vel;
        } else {
            t = fabs(to_pos - from_pos) / vel;
        }
        max_time = std::max(max_time, t);
    }

    // if (from_state.back() + planner->replan_resolution_ > planner->replan_cutoff_) {
    //     max_time += 0.01;
    // }
    // ROS_WARN("Cannot snap in time. time diff: %f, min time %f", diff_time, max_time);
    bool ret = true;
    if (max_time > diff_time) {
        ROS_WARN("Cannot snap in time. time diff: %f, min time %f", diff_time, max_time);
        // getchar();
        ret = false;
    }

    if (!planner->manip_checker->isStateToStateValid(from_state, to_state)) {
        ROS_WARN("Snap motion is in collision");
        ret = false;
    }

    return ret;
}

static
void PrintRegion(std::vector<int> region, int indent)
{
    return;
    if (region.empty()) {
        return;
    }
    for (int i = 0; i < indent; ++i)
        printf("\t");
    for (auto id : region)
        printf(" %d",id);
    printf("\n");
}

bool PreprocessConveyorPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    // const int start_id,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    std::vector<int>& G_UNCOV,
    std::vector<int>& G_COV)
{
    double t_start = start_state.joint_state.position[8];

    printf("\n");
    ROS_INFO("###################    Start id: %d, Start time: %f    ###################", planner->start_id_, t_start);
    ROS_INFO("Uncovered states: %zu", G_UNCOV.size());
    PrintRegion(G_UNCOV, 1);

    ROS_INFO("Covered states: %zu", G_COV.size());
    PrintRegion(G_COV, 1);

    int start_id_work = planner->start_id_;
    std::vector<std::vector<smpl::RobotState>> paths;
    ROS_INFO("Computing Root Paths...");
    auto ret = ComputeRootPaths(
        planner,
        start_state,
        planner->start_id_,
        grasps,
        height,
        paths,
        G_UNCOV,
        G_COV);

    ROS_INFO("Number of paths: %zu", paths.size());
    ROS_INFO("Uncovered after ComputeRootPaths(): %zu", G_UNCOV.size());
    PrintRegion(G_UNCOV, 1);
    ROS_INFO("Covered after ComputeRootPaths(): %zu", G_COV.size());
    PrintRegion(G_COV, 1);

    if (t_start >= planner->replan_cutoff_) {
        ROS_INFO("Start state %d is at Replan cutoff\n", planner->start_id_);
        ROS_INFO("###################    Returning Preprocess for Start id: %d    ###################", planner->start_id_);
        return true;
    }

    // TODO: starts_count should have local scope
    for (size_t i = 0; i < paths.size(); ++i) {
        ROS_INFO("     Working Path: %d Start id %d", i, start_id_work);
        // Assuming replan cutoff respects discretization, stupid
        auto last_state_idx = GetStateIndexAfterTime(paths[i], planner->replan_cutoff_);
        double t = paths[i][last_state_idx].back();
        auto G_UNCOV = G_COV;
        auto G_i = planner->hkey_dijkstra.getSubregion(start_id_work, i);
        planner->hkey_dijkstra.subtractStates(G_UNCOV, G_i);
        auto G_cov = G_i;
        ROS_INFO("     Uncovered init: %zu", G_UNCOV.size());
        PrintRegion(G_UNCOV, 3);
        ROS_INFO("     Covered init: %zu", G_cov.size());
        PrintRegion(G_cov, 3);
        // getchar();

        while (ros::ok() && (t > t_start + 1e-3)) {
            auto state_idx = GetStateIndexAtTime(paths[i], t);
            auto start_new = paths[i][state_idx];
            assert(t == start_new.back());
            ROS_INFO("         Start id: %d Path id: %d Time: %f", start_id_work, i, t);
            for (size_t j = 0; j < planner->home_paths_.size(); ++j) {
                ROS_INFO("             Check snap to home path %d", j);
                // set object init pose in cc based on new goal
                auto object_state_grid = planner->object_graph.getDiscreteCenter(planner->home_center_states_[j]);
                auto object_pose = ComputeObjectPose(object_state_grid, height);
                auto goal_pose = object_pose * grasps[0].inverse();
                planner->manip_checker->setObjectInitialPose(object_pose);
                //

                if (CheckSnap(planner, start_new, planner->home_paths_[j], 1e-3)) {
                    ROS_INFO("             - Snap successful");
                    auto G_j = planner->hkey_dijkstra.getSubregion(0, j);
                    planner->hkey_dijkstra.subtractStates(G_UNCOV, G_j);
                    planner->hkey_dijkstra.addStates(G_cov, G_j);
                    ROS_INFO("             Uncovered after Snap: %zu", G_UNCOV.size());
                    PrintRegion(G_UNCOV, 3);
                    ROS_INFO("             Covered after Snap: %zu", G_cov.size());
                    PrintRegion(G_cov, 3);
                }
                else {
                    SMPL_WARN("             - Failed to snap");
                }
                if (G_UNCOV.empty()) {
                    break;
                }
            }

            break;
            // ROS_INFO("     Remaining states after latching: %zu", G_UNCOV.size());
            // for (auto id : G_UNCOV) {
            //     ROS_INFO("     %d",id);
            // }
            if (G_UNCOV.empty()) {
                ROS_INFO("         Snapping for Path %d covered everything at state %f", i, t);
                break;
            }

            // fill new start state
            moveit_msgs::RobotState start_state_next = start_state;
            for (size_t idx = 0; idx < start_new.size(); ++idx) {
                start_state_next.joint_state.position[idx + 1] = start_new[idx];
            }

            // start_id_next++;
            planner->start_id_++;
            size_t before_rem = G_UNCOV.size();
            auto ret = PreprocessConveyorPlanner(
                planner,
                start_state_next,
                // start_id_next,
                grasps,
                height,
                G_UNCOV,
                G_cov);
            size_t after_rem = G_UNCOV.size();

            if (after_rem == before_rem) {
                // start_id_next--;
                planner->start_id_--;
            }

            ROS_INFO("         Start id: %d Path id: %d Uncovered after Preprocess: %zu", start_id_work, i, G_UNCOV.size());
            PrintRegion(G_UNCOV, 2);
            ROS_INFO("         Start id: %d Path id: %d Covered after Preprocess: %zu", start_id_work, i, G_cov.size());
            PrintRegion(G_cov, 2);

            if (G_UNCOV.empty()) {
                ROS_INFO("         Preprocess for Path %d covered everything at state %f", i, t);
                break;
            }
            t -= planner->replan_resolution_;
            // ROS_INFO("         Start id: %d Path id: %d Decrement time: %f", planner->start_id_, i, t);
            // getchar();
        }
    }
    ROS_INFO("###################    End of Preprocess for Start id: %d    ###################", start_id_work);

	return true;
}

bool PreprocessConveyorPlannerMain(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& home_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height)
{
    ROS_INFO("Preprocessing Main Started!\n");
    // remove all stored data
    std::string cmd = "exec rm -r " + planner->main_dir_ + "/*";
    auto sys_ret = system(cmd.c_str());

    auto G_full = planner->hkey_dijkstra.getAllStates();
    std::vector<int> G_cov;
    // int start_id = 0;
    planner->start_id_ = 0;
    auto ret = PreprocessConveyorPlanner(
        planner,
        home_state,
        // start_id,
        grasps,
        height,
        G_full,
        G_cov);

    // auto dir = "/home/fislam/conveyor_data/";
    planner->manip_graph.saveStartToMapIdMap(planner->main_dir_);
    planner->object_graph.saveStateToPathIdMap(planner->main_dir_);
    ROS_INFO("Preprocessing Main Finished!\n");

    return true;
}

static
auto PreClipPath(
    const std::vector<smpl::RobotState>& path_in,
    int state_idx)
    -> std::vector<smpl::RobotState>
{
    std::vector<smpl::RobotState> path;
    for (int i = state_idx; i < path_in.size(); ++i) {
        path.push_back(path_in[i]);
    }

    return path;
}

static
auto MergePaths(
    const std::vector<smpl::RobotState>& from_path,
    const std::vector<smpl::RobotState>& to_path,
    int start_idx,
    int from_idx)
    -> std::vector<smpl::RobotState>
{
    std::vector<smpl::RobotState> path;
    for (int i = start_idx; i <= from_idx; ++i) {
        // printf("i %d\n", i);
        path.push_back(from_path[i]);
    }

    for (size_t i = 1; i < to_path.size(); ++i) {
        path.push_back(to_path[i]);
    }

    return path;
}

static
auto MergePathsWithSnap(
    const std::vector<smpl::RobotState>& from_path,
    const std::vector<smpl::RobotState>& to_path,
    int start_idx,
    int from_idx,
    int to_idx)
    -> std::vector<smpl::RobotState>
{
    std::vector<smpl::RobotState> path;
    for (int i = start_idx; i <= from_idx; ++i) {
        path.push_back(from_path[i]);
    }

    for (int i = to_idx + 1; i < to_path.size(); ++i) {
        path.push_back(to_path[i]);
    }
    // printf("\nCOMBINED\n");
    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }
    return path;
}

bool QueryConstTimePlanner(
	ConveyorPlanner* planner,
	const moveit_msgs::RobotState& start_state,
	const std::vector<Eigen::Affine3d>& grasps,
    const ObjectState& object_state,
    double height,
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time)
{
	auto object_state_grid = planner->object_graph.getDiscreteCenter(object_state);

    printf("\n");
    ROS_INFO("*******************************************************************");
    ROS_INFO("********   Object State: %.2f, %.2f, %f \t   ********",
                        object_state_grid[0], object_state_grid[1], object_state_grid[2]);
    ROS_INFO("*******************************************************************");

    if (start_state.joint_state.position[8] > planner->replan_cutoff_) {
        ROS_WARN("Pose updated received after replan cutoff: %f", start_state.joint_state.position[8]);
        return false;
    }
	// get map id
    smpl::RobotState start;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            start_state.joint_state,
            start_state.multi_dof_joint_state,
            planner->robot_model->getPlanningJoints(),
            start,
            missing))
    {	
    	ROS_ERROR("Start state is missing planning joints");
    	return false;
    }
    start.push_back(start_state.joint_state.position.back());

    // get map id of home for latching
    smpl::RobotState home;
    if (!leatherman::getJointPositions(
            planner->home_state_.joint_state,
            planner->home_state_.multi_dof_joint_state,
            planner->robot_model->getPlanningJoints(),
            home,
            missing))
    {   
        ROS_ERROR("Home state is missing planning joints");
        return false;
    }
    home.push_back(planner->home_state_.joint_state.position.back());

    // fill planning params
    PlanPathParams params;
    params.allowed_time = planner->time_bound_ + 1e-1;
    params.rc_constrained = false;
    params.shortcut_prerc = false;
    params.only_check_success = false;

    //  1. check if it is the first planning request
    std::vector<smpl::RobotState> path;
	std::pair<int, bool> path_id;
    bool ret = false;
    bool done = false;
    if (planner->current_path_.empty() /*|| start_state.joint_state.position[8] == 0.0*/) {
    	ROS_INFO("First planning request received");
		auto map_ids = planner->manip_graph.getMapIds(start);
        assert(map_ids.size() == 1);
        auto map_id = map_ids[0];
        
		assert(map_id != -1);	// should be zero
		path_id = planner->object_graph.getPathId(map_id, object_state_grid);
        printf("map id: %d path_id: %d\n", map_id, path_id.first);

        auto egraph_dir = planner->main_dir_ +  "/start_" + std::to_string(map_id)
            + "/paths/" + std::to_string(path_id.first);

        if (path_id.second) {
            ROS_INFO("Singleton Path Query");
            params.singleton_dir = egraph_dir;
        }
        ret = PlanPathUsingRootPath(
                planner,
                egraph_dir,
                start_state,
                grasps,
                object_state_grid,
                height,
                path,
                intercept_time,
                params);
        done = true;
	}
	if (!done) 
    {     // Replanning Request
		// 2. Try if the query state belong to the same experience
		auto state_first = planner->current_path_.front();
		auto map_ids = planner->manip_graph.getMapIds(state_first);
        if (map_ids.empty()) {
            ROS_WARN("The first state on the current path is not a replan state");
        }
        else {
            printf("map ids size %zu\n", map_ids.size());
            for (auto map_id : map_ids) {
                printf("map id %d\n", map_id);
        		path_id = planner->object_graph.getPathId(map_id, object_state_grid);
                if (path_id.first == -1) {
                    continue;
                }
                printf("map id: %d path_id: %d\n", map_id, path_id.first);
        		if (path_id.first != planner->current_path_id_) {
        			ROS_WARN("The query state does not belong to the current root path");
        		}
        		else {
        			ROS_INFO("The query state belongs to the current root path");

                    // reconstruct previous start state
                    auto start_state_prev = start_state;
                    for (size_t j = 0; j < 8; ++j) {
                        start_state_prev.joint_state.position[j + 1] = state_first[j];
                    }

                    auto egraph_dir = planner->main_dir_ +  "/start_" + std::to_string(map_id)
                        + "/paths/" + std::to_string(path_id.first);

                    if (path_id.second) {
                        ROS_INFO("Singleton Path Query");
                        params.singleton_dir = egraph_dir;
                    }
        			ret = PlanPathUsingRootPath(
        					planner,
                            egraph_dir,
                            start_state_prev,
        					grasps,
        					object_state_grid,
        					height,
        					path,
        					intercept_time,
                            params);

                    auto start_idx = GetStateIndexAfterTime(path, start_state.joint_state.position[8]);
                    path = PreClipPath(path, start_idx);
                    done = true;
                    break;
                }
            }
        }
	}
    if (!done) 
    {
		ROS_INFO("Back Tracking!!");
        auto last_state_idx = GetStateIndexAfterTime(planner->current_path_, planner->replan_cutoff_);
        double t = planner->current_path_[last_state_idx].back();
		// double t = planner->replan_cutoff_;
		while (t >= start_state.joint_state.position[8]) {
			// 3. Try replan from state at t
            auto start_idx = GetStateIndexAfterTime(planner->current_path_, start_state.joint_state.position[8]);
			auto from_idx = GetStateIndexAtTime(planner->current_path_, t);
			auto map_ids = planner->manip_graph.getMapIds(planner->current_path_[from_idx]);
            if (map_ids.empty()) {
                ROS_WARN("The first state is not a replan state in while");
            }
            else {
                for (auto map_id : map_ids) {
                    path_id = planner->object_graph.getPathId(map_id, object_state_grid);
                    if (path_id.first == -1) {
                        continue;
                    }
    			
    				ROS_INFO("The query state lies in the goal region of the new start state");

                    // reconstruct start state with state at t
                    auto start_state_t = start_state;
                    for (size_t j = 0; j < 8; ++j) {
                        start_state_t.joint_state.position[j + 1] = planner->current_path_[from_idx][j];
                    }

                    auto egraph_dir = planner->main_dir_ +  "/start_" + std::to_string(map_id)
                        + "/paths/" + std::to_string(path_id.first);
                    if (path_id.second) {
                        ROS_INFO("Singleton Path Query");
                        params.singleton_dir = egraph_dir;
                    }
                    ret = PlanPathUsingRootPath(
                            planner,
                            egraph_dir,
                            start_state_t,
                            grasps,
                            object_state_grid,
                            height,
                            path,
                            intercept_time,
                            params);
                    if (ret) {
                        auto to_idx = GetStateIndexAtTime(path, t);
                        path = MergePaths(planner->current_path_, path, start_idx, from_idx);
                    }
                    break;
                }
                if (ret) {
                    break;
                }
                ROS_WARN("The query state DOES NOT lie in the goal region of the new start state");
            }
            // 4. Try snap from state at t
            auto map_id_home = planner->manip_graph.getMapIds(home);
            assert(map_id_home.size() == 1);
            path_id = planner->object_graph.getPathId(map_id_home[0], object_state_grid);
            if (path_id.first == -1) {
                ROS_WARN("The query state DOES NOT lie in the goal region of the home state");
            }
            else {
                ROS_INFO("The query state lies in the goal region of the home state");

                auto egraph_dir = planner->main_dir_ +  "/start_" + std::to_string(map_id_home[0])
                    + "/paths/" + std::to_string(path_id.first);
                ret = PlanPathUsingRootPath(
                        planner,
                        egraph_dir,
                        planner->home_state_,
                        grasps,
                        object_state_grid,
                        height,
                        path,
                        intercept_time,
                        params);
                if (ret) {
                    auto to_idx = GetStateIndexAtTime(path, t);
                    // set object init pose in cc based on new goal
                    auto object_pose = ComputeObjectPose(object_state_grid, height);
                    auto goal_pose = object_pose * grasps[0].inverse();
                    planner->manip_checker->setObjectInitialPose(object_pose);
                    //
                    if (!CheckSnap(planner, planner->current_path_[from_idx], path, 1e-3)) {
                        ROS_ERROR("Snap failed, path does not exist");
                        return false;
                    }
                    path = MergePathsWithSnap(planner->current_path_, path, start_idx, from_idx, to_idx);
                }
                break;
            }
            t -= planner->replan_resolution_;
        }
	}

    if (!ret) {
        return false;
    }

    PostProcessPath(planner, path, planner->interp_resolution_, intercept_time, false);

    // for (const auto& wp : path) {
    //     ROS_INFO_STREAM("waypoint: " << wp);
    // }

    ROS_INFO("Going to profile path");
    if (ret) {
        ConvertJointVariablePathToJointTrajectory(
            planner->robot_model,
            path,
            start_state.joint_state.header.frame_id,
            start_state.multi_dof_joint_state.header.frame_id,
            start_state,
            *trajectory);
    }

    planner->current_path_ = path;
    planner->current_path_id_ = path_id.first;

    ROS_INFO("Planning Succeeded, path size %zu\n", path.size());
    return ret;
}

bool QueryNormalPlanner(
	ConveyorPlanner* planner,
	const moveit_msgs::RobotState& start_state,
	const std::vector<Eigen::Affine3d>& grasps,
    const ObjectState& object_state,
    double height,
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time)
{
	auto object_state_grid = planner->object_graph.getDiscreteCenter(object_state);

	ROS_INFO("#######    Normal Planner: Query object state: %.2f %.2f %.2f    #######", 
			object_state_grid[0], object_state_grid[1], object_state_grid[2]);

    auto object_pose = ComputeObjectPose(object_state_grid, height);
    auto goal_pose = object_pose * grasps[0].inverse();

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(object_pose);

    // clear all memory
    planner->manip_graph.eraseExperienceGraph();
    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
    	ROS_ERROR("Failed to initialize Generic Egraph heuristic");
    	return false;
	}

    PlanPathParams params;
    params.allowed_time = 1000.0;
    params.rc_constrained = false;
    params.shortcut_prerc = true;
    params.only_check_success = false;
	std::vector<smpl::RobotState> path;
    if (!PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params)) {
		ROS_INFO("Unable to plan within allowed time %f",
		params.allowed_time);
		return false;
	}

    if (!IsPathValid(planner->manip_checker, path)) {
        ROS_WARN("Path is invalid after interp");
    }

    ConvertJointVariablePathToJointTrajectory(
        planner->robot_model,
        path,
        planner->home_state_.joint_state.header.frame_id,
        planner->home_state_.multi_dof_joint_state.header.frame_id,
        start_state,
        *trajectory);

	return true;
}

bool QueryAllTestsPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height)
{
    std::vector<int> dirty_states;
	auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
	if (state_id == -1) {
		return false;
	}

	auto center_state = planner->object_graph.extractState(state_id);

	ReinitDijkstras(planner, center_state);
	int dirty_count = 0;
	while (true) {
		auto state_id = planner->hkey_dijkstra.getNextStateId();
		if (state_id == -1) {	// OPEN empty or all states covered
			ROS_INFO("All object states tested successfully, remaining dirty states %d", dirty_count);
			return true;
		}
		auto object_state = planner->object_graph.extractState(state_id);

		auto path_id = planner->object_graph.getPathId(0, object_state);

		ROS_INFO("#######    Query object state: %.2f, %.2f, %f    id: %d     Dirty count: %d   #######", 
				object_state[0], object_state[1], object_state[2], state_id, dirty_count);

		if (path_id.first == -1) {
			ROS_ERROR("Query state %d is dirty or not covered", state_id);
			dirty_count++;
            dirty_states.push_back(state_id);
			continue;
		}

		printf("Path id is %d\n", path_id.first);

        if (path_id.second) {
        	ROS_INFO("Singleton Path, skipping");
        	continue;
        }

	    auto object_pose = ComputeObjectPose(object_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

	    // update collision checker for the new object pose
	    planner->manip_checker->setObjectInitialPose(object_pose);

	    // clear all memory
	    planner->manip_graph.eraseExperienceGraph();
	    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

		// load experience graph
	    auto egraph_dir = planner->main_dir_ + "/start_0/paths/" + std::to_string(path_id.first);

	    planner->manip_graph.loadExperienceGraph(egraph_dir);
	    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
	    	ROS_ERROR("Failed to initialize Generic Egraph heuristic");
	    	return false;
		}

		std::vector<smpl::RobotState> path;
	    double intercept_time;
        PlanPathParams params;
        params.allowed_time = planner->time_bound_ + 1e-3;
        params.rc_constrained = false;
        params.only_check_success = true;
        if (!PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params)) {
			ROS_INFO("Unable to plan to the center within time %f",
			planner->time_bound_);
			getchar();
		}
        planner->current_path_.clear();
	}

    printf("Dirty States\n");
    PrintRegion(dirty_states, 1);

	return true;
}

static
double rand_sign()
{
    return (rand() << 1) + (rand() & 1);
}

enum  Planner
{
    NORMAL = 0,
    CONST_TIME,
    EGRAPH
};

bool QueryReplanningTestsPerceptionPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& home_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests)
{
    // Planner p = Planner::CONST_TIME;
    Planner p = Planner::NORMAL;
    // Planner p = Planner::EGRAPH;

    int failed_count = 0;
    // std::ofstream ofs("/home/fislam/rss_stats/const.csv");
    std::ofstream ofs("/home/fislam/rss_stats/normal.csv");
    // std::ofstream ofs("/home/fislam/rss_stats/egraph.csv");

    // params
    double noise_trans = 0.05;
    double noise_yaw = 2 * M_PI;
    double t_perception = 0.4;
    double y_img = 1.48;
    double x_min = 0.4;
    double x_max = 0.59;
    // double y_min = 1.2;
    // double y_max = 1.29;
    double t_buff = 0.4;
    double speed = 0.2;

    // **** Change for each alg ******//
    double t_bound = 2.0;
    //*******************************//

    double t_offset = t_perception + t_bound + t_buff;

    double y_plan = y_img - (speed * t_offset);

    if (p == Planner::CONST_TIME) {
        y_plan = 1.25;
    }


    mt19937 gen(1);
    uniform_real_distribution<double> distrx(x_min, x_max);
    uniform_real_distribution<double> distryaw(- M_PI, M_PI);

    //***********Tests****************//
    for (int tid = 0; tid < num_tests; ++tid) {
        // double x_plan = x_min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(x_max-x_min)));
        // double yaw_plan = static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/2 * M_PI));
        double x_plan = distrx(gen);
        double yaw_plan = distryaw(gen);

        // printf("x yaw %f %f\n", x_plan, yaw_plan);
        // getchar();

        std::vector<double> object_state = {x_plan, y_plan, yaw_plan};

        auto start_state = home_state;
        printf("Running test: %d start time %f \n", tid, home_state.joint_state.position[8]);
        moveit_msgs::RobotTrajectory traj;
        double replan_state_time = 0.0;

        //***********Replanning for the same run****************//
        int qid = 0;
        double xx_min = x_plan - 0.05;
        double xx_max = x_plan + 0.04;
        double yy_min = y_plan - 0.05;
        double yy_max = y_plan + 0.04;
        uniform_real_distribution<double> distrxx(xx_min, xx_max);
        uniform_real_distribution<double> distryy(yy_min, yy_max);
        while (replan_state_time < planner->replan_cutoff_) {
            if (qid > 0) {
                // 1. update start
                start_state.joint_state.position[8] = replan_state_time;
                for (size_t i = 0; i < traj.joint_trajectory.points.size(); ++i) {
                    if (traj.joint_trajectory.points[i].time_from_start.toSec() >= replan_state_time) {
                        for (size_t j = 0; j < 7; ++j) {
                            start_state.joint_state.position[j + 1] = traj.joint_trajectory.points[i].positions[j];
                        }
                        // printf("SETTING START TIME %f\n", replan_state_time);
                        break;
                    }
                }

                // 2. update goal
                // object_state[0] = xx_min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(xx_max-xx_min)));
                // object_state[1] = y_min + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(y_max-y_min)));
                // object_state[2] = (static_cast <double> (rand()) / (static_cast <double> (RAND_MAX/noise_yaw)));
                object_state[0] = distrxx(gen);
                object_state[1] = distryy(gen);
                object_state[2] = distryaw(gen);

            }

            auto object_state_grid = planner->object_graph.getDiscreteCenter(object_state);

            printf("\n");
            ROS_INFO("*******************************************************************");
            ROS_INFO("********   Object State: %.2f, %.2f, %f \t   ********",
                                object_state_grid[0], object_state_grid[1], object_state_grid[2]);
            ROS_INFO("*******************************************************************");

            ROS_INFO("Plan/Replan from %f",start_state.joint_state.position[8]);

            if (start_state.joint_state.position[8] > planner->replan_cutoff_) {
                ROS_ERROR("Attempted Replanning after replan cutoff %f", start_state.joint_state.position[8]);
                break;
            }

            traj.joint_trajectory.points.clear();
            auto start = smpl::clock::now();

            // TODO: add switch statement here for different planners
            bool ret;
            double intercept_time;
            switch (p) {
            case Planner::CONST_TIME:
            {
                ret = QueryConstTimePlanner(
                    planner,
                    start_state,
                    grasps,
                    object_state_grid,
                    height,
                    &traj,
                    intercept_time);
                break;                
            }
            case Planner::NORMAL:
            {
                ret = QueryNormalPlanner(
                    planner,
                    start_state,
                    grasps,
                    object_state_grid,
                    height,
                    &traj,
                    intercept_time);
                break;                
            }
            case Planner::EGRAPH:
            {
                ret = QueryEgraphPlanner(
                    planner,
                    start_state,
                    grasps,
                    object_state_grid,
                    height,
                    &traj,
                    intercept_time);
                break;
            }
            }

            auto end = smpl::clock::now();

            // doing it to account for updategoal time
            auto planning_time = std::chrono::duration<double>(end - start).count();

            printf("planning_time %f\n", planning_time);

            // if (planning_time > t_bound + t_buff) {
            //     ret = false;
            // }

            ofs << ret << '\t';
            double cost = 0;
            if (ret) {
                cost = traj.joint_trajectory.points.back().time_from_start.toSec();
            }
            ofs << start_state.joint_state.position[8] << '\t';
            ofs << planner->egraph_planner->get_n_expands() << '\t';
            // ofs << planning_time << '\t';
            // if (p != Planner::EGRAPH) {
                ofs << planner->egraph_planner->get_final_eps_planning_time() << '\t';
            // }
            ofs << cost << '\t' << '\t';

            // getchar();
            if (ret) {
                ROS_INFO("Request %d successful", qid);
            }
            else {
                ROS_WARN("Request %d failed", qid);
                failed_count++;
                // break;
            }

            replan_state_time += t_offset + planner->replan_resolution_;     // may be add some buffer here?
            qid++;
        }   // replan cycle end

        ofs << '\n';

        planner->current_path_.clear();
    }   // tests end
    ROS_INFO("Failed count %f", failed_count);

    return true;
}

bool QueryReplanningTestsPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& home_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests)
{
    int failed_count = 0;
    std::ofstream ofs("/home/fislam/rss_stats/const_time_random_replan.csv");

    int num_plans = 3;
    for (int tid = 0; tid < num_tests; ++tid) {
        printf("Running test: %d\n", tid);
        auto start_state = home_state;
        moveit_msgs::RobotTrajectory traj;
        for (int qid = 0; qid < num_plans; ++qid) {
            auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
            if (state_id == -1) {
                return false;
            }

            auto object_state = planner->object_graph.extractState(state_id);

            if (qid > 0) {
                double range = planner->replan_cutoff_/(double)num_plans;
                double LO = (qid - 1) * range;
                double HI = LO + range;
                printf("low %f high %f range %f\n", LO, HI, range);
                double replan_time = LO + static_cast <double> (rand()) /( static_cast <double> (RAND_MAX/(HI-LO)));

                for (size_t i = 0; i < traj.joint_trajectory.points.size(); ++i) {
                    if (traj.joint_trajectory.points[i].time_from_start.toSec() >= replan_time) {
                        for (size_t j = 0; j < 7; ++j) {
                            start_state.joint_state.position[j + 1] = traj.joint_trajectory.points[i].positions[j];
                        }
                        start_state.joint_state.position[8] = replan_time;
                        break;
                    }
                }  
            }

            ROS_INFO("Replan from %f",start_state.joint_state.position[8]);

            double intercept_time;
            traj.joint_trajectory.points.clear();
            auto start = smpl::clock::now();
            bool ret = QueryConstTimePlanner(
                planner,
                start_state,
                grasps,
                object_state,
                height,
                &traj,
                intercept_time);
            auto end = smpl::clock::now();

            // doing it to account for updategoal time
            auto planning_time = std::chrono::duration<double>(end - start).count();

            ofs << ret << ' ';
            double cost = 0;
            if (ret) {
                cost = traj.joint_trajectory.points.back().time_from_start.toSec();
            }
            ofs << planner->egraph_planner->get_n_expands() << ' ';
            ofs << planning_time << ' ';
            ofs << cost << '\t';

            if (ret) {
                ROS_INFO("Request %d successful", qid);
            }
            else {
                ROS_WARN("Request %d failed", qid);
                failed_count++;
                break;
            }
        }
        ofs << '\n';

        planner->current_path_.clear();
    }



        // //==================================================
        // // Second query
        // auto state_id2 = planner->hkey_dijkstra.sampleObjectState(0);
        // if (state_id2 == -1) {
        //     return false;
        // }

        // auto object_state2 = planner->object_graph.extractState(state_id2);

        // auto start_state2 = start_state1;
        // double replan_time = static_cast <float> (std::rand()) / 
        //     (static_cast <float> (RAND_MAX/planner->replan_cutoff_));

        // for (size_t i = 0; i < traj1.joint_trajectory.points.size(); ++i) {
        //     if (traj1.joint_trajectory.points[i].time_from_start.toSec() >= replan_time) {
        //         for (size_t j = 0; j < 7; ++j) {
        //             start_state2.joint_state.position[j + 1] = traj1.joint_trajectory.points[i].positions[j];
        //         }
        //         start_state2.joint_state.position[8] = replan_time;
        //         break;
        //     }
        // }    

        // moveit_msgs::RobotTrajectory traj2;
        // bool ret2 = QueryConstTimePlanner(
        //     planner,
        //     start_state2,
        //     grasps,
        //     object_state2,
        //     height,
        //     &traj2,
        //     intercept_time);


        // if (ret2) {
        //     ROS_INFO("Second Request successful");
        // }
        // else {
        //     SMPL_ERROR("Second Request failed, id: %d", state_id2);
        //     getchar();
        // }

        // //=====================================================

        // // Third query
        // auto state_id3 = planner->hkey_dijkstra.sampleObjectState(0);
        // if (state_id3 == -1) {
        //     return false;
        // }

        // auto object_state2 = planner->object_graph.extractState(state_id3);

        // auto start_state2 = start_state1;
        // double replan_time = static_cast <float> (std::rand()) / 
        //     (static_cast <float> (RAND_MAX/planner->replan_cutoff_));

        // for (size_t i = 0; i < traj1.joint_trajectory.points.size(); ++i) {
        //     if (traj1.joint_trajectory.points[i].time_from_start.toSec() >= replan_time) {
        //         for (size_t j = 0; j < 7; ++j) {
        //             start_state2.joint_state.position[j + 1] = traj1.joint_trajectory.points[i].positions[j];
        //         }
        //         start_state2.joint_state.position[8] = replan_time;
        //         break;
        //     }
        // }    

        // moveit_msgs::RobotTrajectory traj2;
        // bool ret2 = QueryConstTimePlanner(
        //     planner,
        //     start_state2,
        //     grasps,
        //     object_state2,
        //     height,
        //     &traj2,
        //     intercept_time);


        // if (ret2) {
        //     ROS_INFO("Second Request successful");
        // }
        // else {
        //     SMPL_ERROR("Second Request failed, id: %d", state_id3);
        //     getchar();
        // }

        // planner->current_path_.clear();
    // }

    return true;
}

bool QueryRandomTestsConstTimePlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests)
{
#if 0
    int failed_count = 0;
    std::ofstream ofs("/home/fislam/rss_stats/const_time_random.csv");
    for (int i = 0; i < num_tests; ++i) {
        printf("Running test: %d\n", i);
        auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
        if (state_id == -1) {
            return false;
        }
        auto object_state = planner->object_graph.extractState(state_id);

        int map_id = 0;
        auto path_id = planner->object_graph.getPathId(map_id, object_state);

        if (path_id.first == -1) {
            ROS_WARN("State uncovered");
            continue;
        }
        // printf("map id: %d path_id: %d\n", map_id, path_id.first);

        std::vector<smpl::RobotState> path;
        double intercept_time;
        auto start = smpl::clock::now();
        bool ret = PlanPathUsingRootPath(
                planner,
                start_state,
                map_id,
                path_id,
                grasps,
                object_state,
                height,
                path,
                intercept_time);
        auto end = smpl::clock::now();

        // doing it to account for updategoal time
        auto planning_time = std::chrono::duration<double>(end - start).count();

        if (!ret) {
            ROS_ERROR("Const time planner failed, reason unknown");
            failed_count++;
        }

        ofs << ret << ' ';
        double cost = 0;
        if (ret) {
            cost = path.back().back();
        }
        ofs << planner->egraph_planner->get_n_expands() << ' ';
        ofs << planning_time << ' ';
        ofs << cost << '\n';

        planner->current_path_.clear();
    }

    ROS_INFO("Total failures: %d", failed_count);
    ofs.close();
#endif
}

bool QueryRandomTestsNormalPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests)
{
    int failed_count = 0;
    std::ofstream ofs("/home/fislam/rss_stats/normal_random.csv");
    for (int i = 0; i < num_tests; ++i) {
        printf("Running test: %d\n", i);
        auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
        if (state_id == -1) {
            return false;
        }
        auto object_state = planner->object_graph.extractState(state_id);

        // ROS_INFO("#######    Query object state: %.2f, %.2f, %f    id: %d     Dirty count: %d   #######", 
        //         object_state[0], object_state[1], object_state[2], state_id);

        auto object_pose = ComputeObjectPose(object_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

        // update collision checker for the new object pose
        planner->manip_checker->setObjectInitialPose(object_pose);

        // clear all memory
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();

        if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
            ROS_ERROR("Failed to initialize Generic Egraph heuristic");
            return false;
        }

        std::vector<smpl::RobotState> path;
        double intercept_time;
        PlanPathParams params;
        params.allowed_time = 10;
        params.rc_constrained = false;
        params.only_check_success = false;

        bool ret = PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params);
        if (!ret) {
            ROS_INFO("QueryRandomTestsNormalPlanner test %d failed",i);
            failed_count++;
        }


        ofs << ret << ' ';
        double cost = 0;
        if (ret) {
            cost = path.back().back();
        }
        ofs << planner->egraph_planner->get_n_expands() << ' ';
        ofs << planner->egraph_planner->get_final_eps_planning_time() << ' ';
        ofs << cost << '\n';

        planner->current_path_.clear();
    }
    ROS_INFO("Total failures: %d", failed_count);
    ofs.close();

    return true;
}

bool QueryEgraphPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    const ObjectState& object_state,
    double height,
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time)
{
    std::string egraph_dir = "/home/fislam/egraph/";
    auto object_pose = ComputeObjectPose(object_state, height);
    auto goal_pose = object_pose * grasps[0].inverse();

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(object_pose);

    // clear all memory
    planner->egraph_planner->force_planning_from_scratch_and_free_memory();
    planner->manip_graph.eraseExperienceGraph();
    planner->manip_graph.loadExperienceGraph(egraph_dir);

    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
        ROS_ERROR("Failed to initialize Generic Egraph heuristic");
        return false;
    }

    std::vector<smpl::RobotState> path;
    PlanPathParams params;
    params.allowed_time = 2.0;
    params.rc_constrained = false;
    params.only_check_success = false;

    bool ret = PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params);
    if (!ret) {
        ROS_INFO("QueryEgraphPlanner test failed");
        return false;
    }
    ROS_INFO("Success");

    ConvertJointVariablePathToJointTrajectory(
        planner->robot_model,
        path,
        start_state.joint_state.header.frame_id,
        start_state.multi_dof_joint_state.header.frame_id,
        start_state,
        *trajectory);
    return true;
}

bool QueryRandomTestsEgraphPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests)
{
    // Construct egraph first
    int num_eg = 5;
    std::string egraph_dir = "/home/fislam/egraph/";
    auto sys_ret = system(("exec rm -r " + egraph_dir + "*").c_str());
    int count = 0;
    while (count < num_eg) {
        auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
        if (state_id == -1) {
            return false;
        }
        auto object_state = planner->object_graph.extractState(state_id);

        // ROS_INFO("#######    Query object state: %.2f, %.2f, %f    id: %d     Dirty count: %d   #######", 
        //         object_state[0], object_state[1], object_state[2], state_id);

        auto object_pose = ComputeObjectPose(object_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

        // update collision checker for the new object pose
        planner->manip_checker->setObjectInitialPose(object_pose);

        // clear all memory
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();

        if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
            ROS_ERROR("Failed to initialize Generic Egraph heuristic");
            return false;
        }

        std::vector<smpl::RobotState> path;
        double intercept_time;
        PlanPathParams params;
        params.allowed_time = 10;
        params.rc_constrained = false;
        params.only_check_success = false;

        bool ret = PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params);
        if (!ret) {
            continue;
        }

        moveit_msgs::RobotTrajectory root_traj;
        ConvertJointVariablePathToJointTrajectory(
            planner->robot_model,
            path,
            start_state.joint_state.header.frame_id,
            start_state.multi_dof_joint_state.header.frame_id,
            start_state,
            root_traj);

        WritePath(planner->robot_model, start_state, root_traj, egraph_dir, intercept_time);
        count++;
    }

    ROS_INFO("Generated experiences %zu", num_eg);
    getchar();

    // Start running tests

    int failed_count = 0;
    std::ofstream ofs("/home/fislam/rss_stats/egraph_random.csv");
    for (int i = 0; i < num_tests; ++i) {
        printf("Running test: %d\n", i);
        auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
        if (state_id == -1) {
            return false;
        }
        auto object_state = planner->object_graph.extractState(state_id);

        // ROS_INFO("#######    Query object state: %.2f, %.2f, %f    id: %d     Dirty count: %d   #######", 
        //         object_state[0], object_state[1], object_state[2], state_id);

        auto object_pose = ComputeObjectPose(object_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

        // update collision checker for the new object pose
        planner->manip_checker->setObjectInitialPose(object_pose);

        // clear all memory
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();
        planner->manip_graph.eraseExperienceGraph();
        planner->manip_graph.loadExperienceGraph(egraph_dir);

        if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
            ROS_ERROR("Failed to initialize Generic Egraph heuristic");
            return false;
        }

        std::vector<smpl::RobotState> path;
        double intercept_time;
        PlanPathParams params;
        params.allowed_time = 10;
        params.rc_constrained = false;
        params.only_check_success = false;

        bool ret = PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params);
        if (!ret) {
            ROS_INFO("QueryRandomTestsEgraphPlanner test %d failed",i);
            failed_count++;
        }
        else {
            ROS_INFO("Success");
        }


        ofs << ret << ' ';
        double cost = 0;
        if (ret) {
            cost = path.back().back();
        }
        ofs << planner->egraph_planner->get_n_expands() << ' ';
        ofs << planner->egraph_planner->get_final_eps_planning_time() << ' ';
        ofs << cost << '\n';

        planner->current_path_.clear();
    }
    ROS_INFO("Total failures: %d", failed_count);
    ofs.close();

    return true;
}

bool QueryAllTestsNormalPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height)
{
    auto state_id = planner->hkey_dijkstra.sampleObjectState(0);
    if (state_id == -1) {
        return false;
    }

    auto center_state = planner->object_graph.extractState(state_id);

    ReinitDijkstras(planner, center_state);
    int failed_count = 0;
    int num_query = 0;
    while (true) {
        printf("Running Query: %d\n", num_query);
        num_query++;
        auto state_id = planner->hkey_dijkstra.getNextStateId();
        if (state_id == -1) {   // OPEN empty or all states covered
            ROS_INFO("All object states tested successfully, failures %d", failed_count);
            return true;
        }
        auto object_state = planner->object_graph.extractState(state_id);

        ROS_INFO("#######    Query object state: %.2f, %.2f, %f    id: %d     Dirty count: %d   #######", 
                object_state[0], object_state[1], object_state[2], state_id, failed_count);

        auto object_pose = ComputeObjectPose(object_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

        // update collision checker for the new object pose
        planner->manip_checker->setObjectInitialPose(object_pose);

        // clear all memory
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();

        if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
            ROS_ERROR("Failed to initialize Generic Egraph heuristic");
            return false;
        }

        std::vector<smpl::RobotState> path;
        double intercept_time;
        PlanPathParams params;
        params.allowed_time = 5.0;
        params.rc_constrained = false;
        params.only_check_success = true;
        if (!PlanRobotPath(planner, start_state, goal_pose, path, intercept_time, params)) {
            ROS_INFO("Unable to plan to the center within time %f",
            planner->time_bound_);
            failed_count++;
        }
    }

    return true;
}