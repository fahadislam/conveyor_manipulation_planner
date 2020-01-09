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
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'mprim_filename' not found in planning params");
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

    std::vector<double> resolutions(robot->jointVariableCount() + 1);

    std::string disc_string;
    if (!params->getParam("discretization", disc_string)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'discretization' not found in planning params");
        return false;
    }
    auto disc = ParseMapFromString<double>(disc_string);
    SMPL_DEBUG_NAMED(CP_LOGGER, "Parsed discretization for %zu joints", disc.size());

    for (size_t vidx = 0; vidx < robot->jointVariableCount(); ++vidx) {
        auto& vname = robot->getPlanningJoints()[vidx];
        auto dit = disc.find(vname);
        if (dit == end(disc)) {
            SMPL_ERROR_NAMED(CP_LOGGER, "Discretization for variable '%s' not found in planning parameters", vname.c_str());
            return false;
        }
        resolutions[vidx] = dit->second;
    }

    // time dimension
    auto dit = disc.find("time");
    if (dit == end(disc)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Discretization for variable 'time' not found in planning parameters");
        return false;
    }
    resolutions[robot->jointVariableCount()] = dit->second;

    ConveyorManipLatticeActionSpaceParams action_params;
    if (!GetConveyorManipLatticeActionSpaceParams(action_params, *params)) {
        return false; // errors logged within
    }

    ////////////////////
    // Initialization //
    ////////////////////

    if (!graph->init(robot, checker, resolutions, actions)) {
        SMPL_ERROR("Failed to initialize Conveyor Manip Lattice Egraph");
        return false;
    }

    if (!actions->init(graph)) {
        SMPL_ERROR("Failed to initialize Conveyor Manip Lattice Action Space");
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
        SMPL_ERROR("Failed to load actions from file '%s'", action_params.mprim_filename.c_str());
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
    SMPL_INFO("Convert Variable Path to Robot Trajectory");

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

    SMPL_INFO("  Path includes %zu single-dof joints and %zu multi-dof joints",
            traj.joint_trajectory.joint_names.size(),
            traj.multi_dof_joint_trajectory.joint_names.size());

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

        for (size_t vidx = 0; vidx < point.size() - 1; ++vidx) {
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
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
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
    printf("first time %f samples %d\n", path[0].back(), samples);
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
            SMPL_INFO("Create plan output directory %s", p.native().c_str());
            boost::filesystem::create_directory(p);
        }

        if (!boost::filesystem::is_directory(p)) {
            SMPL_ERROR("Failed to log path. %s is not a directory", path.c_str());
            return false;
        }
    } catch (const boost::filesystem::filesystem_error& ex) {
        SMPL_ERROR("Failed to create plan output directory %s", p.native().c_str());
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

    SMPL_INFO("Log path to %s", p.native().c_str());

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
                    SMPL_WARN("Unrecognized multi-dof local variable name '%s'", local_name.c_str());
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
        SMPL_ERROR("Failed to set object goal");
        return 1;
    }

    if (!planner->object_graph.setStart(start)) {
        SMPL_ERROR("Failed to set object start");
        return 1;
    }

    int start_id = planner->object_graph.getStartStateID();
    if (start_id < 0) {
        SMPL_ERROR("Start state id is invalid");
        return 1;
    }

    int goal_id = planner->object_graph.getGoalStateID();
    if (goal_id < 0)  {
        SMPL_ERROR("Goal state id is invalid");
        return 1;
    }

    if (planner->hkey_dijkstra.set_start(start_id) == 0) {
        SMPL_ERROR("Failed to set planner start state");
        return 1;
    }

    if (planner->hkey_dijkstra.set_goal(goal_id) == 0) {
        SMPL_ERROR("Failed to set planner goal state");
        return 1;
    }

    if (!planner->hkey_dijkstra.reinit_search()) {
        SMPL_ERROR("Failed to initialize Dijsktras");
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
    const smpl::PlanningParams* params)	// pass params
{
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
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'resolution_xy' not found in planning params");
        return false;
    }
    if (!params->getParam("resolution_yaw", res_yaw)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'resolution_yaw' not found in planning params");
        return false;
    }
    if (!params->getParam("origin_x", origin_x)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'origin_x' not found in planning params");
        return false;
    }
    if (!params->getParam("origin_y", origin_y)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'origin_y' not found in planning params");
        return false;
    }

    if (!params->getParam("time_bound", planner->time_bound_)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'time_bound' not found in planning params");
        return false;
    }

    if (!params->getParam("replan_cutoff", planner->replan_cutoff_)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'replan_cutoff' not found in planning params");
        return false;
    }

    std::vector<double> object_resolutions = { res_xy, res_xy, res_yaw };
    if (!planner->object_graph.init(
    		planner->object_model,
    		planner->object_checker,
    		object_resolutions,
    		&planner->object_actions)) {
        SMPL_ERROR("Failed to initialize Manip Lattice");
        return 1;
    }

    // 2. Init actions
    if (!planner->object_actions.init(&planner->object_graph)) {
        SMPL_ERROR("Failed to initialize Manip Lattice Action Space");
        return 1;
    }
	std::string object_mprim_path;
    if (!params->getParam("object_mprim_filename", object_mprim_path)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'object_mprim_filename' not found in planning params");
        return false;
    }
    // load primitives from file, whose path is stored on the param server
    if (!planner->object_actions.load(object_mprim_path)) {
        return 1;
    }

    // 3. Init heuristic
    if (!planner->object_heuristic.init(&planner->object_graph)) {
        SMPL_ERROR("Failed to initialize Joint Dist Heuristic");
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
        SMPL_ERROR("Failed to initialize Generic Egraph heuristic");
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

    // 5. Load state to path id map
    if (!planner->object_graph.loadStateToPathIdMap()) {
    	SMPL_WARN("No state to id map found");
    }
}

static
bool IsPathValid(smpl::CollisionChecker* checker, const std::vector<smpl::RobotState>& path)
{

    for (size_t i = 1; i < path.size(); ++i) {
        if (!checker->isStateToStateValid(path[i - 1], path[i])) {
            SMPL_ERROR_STREAM("path between " << path[i - 1] << " and " << path[i] << " is invalid (" << i - 1 << " -> " << i << ")");
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
    //     SMPL_INFO_STREAM("waypoint: " << wp);
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
    //     SMPL_INFO_STREAM("waypoint: " << wp);
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
    //     SMPL_INFO_STREAM("waypoint: " << wp);
    // }

    // printf("\nCOMBINED\n");
    // for (const auto& wp : path) {
    //     SMPL_INFO_STREAM("waypoint: " << wp);
    // }

    path.push_back(lift_state);

    // printf("AFTER LAST STATE\n");
    // for (const auto& wp : path) {
    //     SMPL_INFO_STREAM("waypoint: " << wp);
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
    printf("Size of interpolated path: %zu\n", path.size());

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
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time,
    PlanPathParams params)
{
//         bool check_replancutoff,
//     bool shortcut_prerc,
//     bool postprocess = true)
    // allowed_time

    // TEMPORARY: TODO: Get shit done
    bool preprocessing = true;

    //==============================================================================//
    // By default the collision object remains inflated                             //
    // We only deflate to find shortcut successors and to check final interpolation //
    //==============================================================================//

    ///////////////
    // Set Goal  //
    ///////////////

    smpl::GoalConstraint goal;
    goal.type = smpl::GoalType::XYZ_RPY_GOAL;
    goal.pose = object_pose;

    goal.xyz_tolerance[0] = 0.01; //0.015;
    goal.xyz_tolerance[1] = 0.01; //0.015;
    goal.xyz_tolerance[2] = 0.01; //0.015;
    goal.rpy_tolerance[0] = 0.0872665; //0.05;
    goal.rpy_tolerance[1] = 0.0872665; //0.05;
    goal.rpy_tolerance[2] = 0.0872665; //0.05;

    if (!planner->manip_graph.setGoal(goal)) {
        SMPL_ERROR("Failed to set manip goal");
        return false;
    }

    planner->manip_checker->deflateCollisionObject();
    planner->egraph_manip_heuristic.updateGoal(goal);
    planner->manip_checker->inflateCollisionObject();

    if (params.rc_constrained) {
        bool ret_cutoff = planner->egraph_manip_heuristic.isReplanCutoffBeforeShortcutNode(planner->replan_cutoff_);
        if (!ret_cutoff) {
            ROS_WARN("Replan cutoff is after the shortcut node");
            return false;
        }
    }

    auto goal_id = planner->manip_graph.getGoalStateID();
    if (goal_id == -1) {
        SMPL_ERROR("No goal state has been set");
        return false;
    }

    if (planner->egraph_planner->set_goal(goal_id) == 0) {
        SMPL_ERROR("Failed to set planner goal state");
        return false;
    }

    ////////////////
    // Set Start  //
    ////////////////

    smpl::RobotState start;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            planner->home_state_.joint_state,
            planner->home_state_.multi_dof_joint_state,
            planner->robot_model->getPlanningJoints(),
            start,
            missing))
    {	
    	SMPL_ERROR("Start state is missing planning joints");
    	return false;
    }
    start.push_back(planner->home_state_.joint_state.position.back());	// time

    if (!planner->manip_graph.setStart(start)) {
        SMPL_ERROR("Failed to set start state");
        return false;
    }

    auto start_id = planner->manip_graph.getStartStateID();
    if (start_id == -1) {
        SMPL_ERROR("No start state has been set");
        return false;
    }

    if (planner->egraph_planner->set_start(start_id) == 0) {
        SMPL_ERROR("Failed to set start state");
        return false;
    }

    ////////////////
    // Plan Path  //
    ////////////////

    bool b_ret = false;
    std::vector<int> solution_state_ids;
    
    int sol_cost;
    planner->egraph_planner->m_allowed_expansions = params.allowed_time * 250;
    SMPL_INFO("Expansions bound: %d\n", planner->egraph_planner->m_allowed_expansions);
    b_ret = planner->egraph_planner->replan(params.allowed_time, &solution_state_ids, &sol_cost);

    if (params.only_check_success) {
        return b_ret;
    }

    std::vector<smpl::RobotState> path;
    std::vector<smpl::RobotState> new_path;

    if (b_ret && (solution_state_ids.size() > 0)) {
        SMPL_INFO_NAMED(CP_LOGGER, "Planning succeeded");
        SMPL_INFO_NAMED(CP_LOGGER, "  Num Expansions (Initial): %d", planner->egraph_planner->get_n_expands_init_solution());
        SMPL_INFO_NAMED(CP_LOGGER, "  Num Expansions (Final): %d", planner->egraph_planner->get_n_expands());
        SMPL_INFO_NAMED(CP_LOGGER, "  Epsilon (Initial): %0.3f", planner->egraph_planner->get_initial_eps());
        SMPL_INFO_NAMED(CP_LOGGER, "  Epsilon (Final): %0.3f", planner->egraph_planner->get_solution_eps());
        SMPL_INFO_NAMED(CP_LOGGER, "  Time (Initial): %0.3f", planner->egraph_planner->get_initial_eps_planning_time());
        SMPL_INFO_NAMED(CP_LOGGER, "  Time (Final): %0.3f", planner->egraph_planner->get_final_eps_planning_time());
        SMPL_INFO_NAMED(CP_LOGGER, "  Path Length (states): %zu", solution_state_ids.size());
        SMPL_INFO_NAMED(CP_LOGGER, "  Solution Cost: %d", sol_cost);

        new_path.clear();
        if (!planner->manip_graph.extractPath(solution_state_ids, new_path)) {
            SMPL_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    else {
        return b_ret;
    }

    intercept_time = planner->manip_graph.getInterceptTime(new_path);

    //=========================================================
    // If replan request then merge new path with current path
    // If first plan request then use the new path as it is
    //=========================================================

    double resolution = 0.05;
    PostProcessPath(planner, new_path, resolution, intercept_time, params.shortcut_prerc);

    if (!preprocessing && !planner->current_path_.empty()) {
        size_t cdx;
        for (cdx = 0; cdx < planner->current_path_.size(); ++cdx) {
            if (planner->current_path_[cdx].back() > planner->replan_cutoff_) {
                break;
            }
        }
        size_t ndx;
        double buffer = 0.0;
        for (ndx = 0; ndx < new_path.size(); ++ndx) {
            if (new_path[ndx].back() > planner->current_path_[cdx].back() + buffer) {
                break;
            }
        }

        SMPL_INFO("Snapping from time %f to %f", planner->current_path_[cdx].back(), new_path[ndx].back());

        // find path from planner->current_path_[cdx] to new_path[ndx];
        // TODO: Use this common function whereever
        double max_time = 0.0;
        double diff_time = new_path[ndx].back() - planner->current_path_[cdx].back(); 
        for (size_t j = 0; j < planner->robot_model->jointVariableCount(); ++j) {
            auto from_pos = planner->current_path_[cdx][j];
            auto to_pos = new_path[ndx][j];
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

        if (max_time > diff_time) {
            ROS_ERROR("Cannot snap in time. time diff: %f, min time %f", diff_time, max_time);
            return false;
        }

        if (!planner->manip_checker->isStateToStateValid(planner->current_path_[cdx], new_path[ndx])) {
            ROS_ERROR("Snap between the two paths is invalid");
            return false;
        }

        // Combine paths
        path.resize(new_path.size() - ndx + 1);  // +1 for planner->current_path_[cdx]
        path[0] = planner->current_path_[cdx];
        std::copy(new_path.begin() + ndx, new_path.end(), path.begin() + 1);

        // PostProcessPath(planner, path, resolution, intercept_time, params.shortcut_prerc);
    }
    else {
        path = new_path;
    }

    // printf("new path:\n");
    // for (const auto& wp : path) {
    //     SMPL_INFO_STREAM("waypoint: " << wp);
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
    //     SMPL_INFO_STREAM("waypoint: " << wp);
    // }

    auto to_check_path = path;
    to_check_path.pop_back();
    planner->manip_checker->deflateCollisionObject();
    if (!IsPathValid(planner->manip_checker, to_check_path)) {
        ROS_WARN("Path is invalid after interp");
        // getchar();
    }
    planner->manip_checker->inflateCollisionObject();

    // no need for joint_trajs anymore as we are not using cubic profiler

    ConvertJointVariablePathToJointTrajectory(
            planner->robot_model,
            path,
            planner->home_state_.joint_state.header.frame_id,
            planner->home_state_.multi_dof_joint_state.header.frame_id,
            start_state,
            *trajectory);

    planner->current_path_ = path;

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

bool PreprocessConveyorPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height)
{
	// remove all stored data
	auto sys_ret = system("exec rm -r /home/fislam/paths/*");

    /////////////////////////////////////////////
    //                 MAIN LOOP               //
    /////////////////////////////////////////////


    // - Mark an object state as "dirty" if:
    //   	a) fails to plan from scratch
    //   			or
    //   	b) can plan from scratch but can't use the same experience to plan by reuse
    // - A covered state might be a dirty state
    // - Tertminate preprocessing when there are no more uncovered states

	int center_count = 0;
    auto root_egraph_dir = "/home/fislam/paths/root/";
	while(true) {

	    ///////////////////////////////////////////////
	    // Sample a random object goal: { x, y yaw } //
	    ///////////////////////////////////////////////

    	auto state_id = planner->hkey_dijkstra.sampleObjectState();
		if (state_id == -1) {
			break;
		}

		// remove state from uncovered list
		auto center_state = planner->object_graph.extractState(state_id);

		printf("\n");
		SMPL_INFO("*******************************************************************");
	    SMPL_INFO("********   Object State: %.2f, %.2f, %f \t id: %d   ********",
	    					center_state[0], center_state[1], center_state[2], state_id);
	    SMPL_INFO("*******************************************************************");

	    //////////////////////////////////////////////////////
	    // Compute path to object goal and store experience //
	    //////////////////////////////////////////////////////

	    auto object_pose = ComputeObjectPose(center_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

	    // update collision checker for the new object pose
	    planner->manip_checker->setObjectInitialPose(object_pose);

        // clear all memory
        planner->manip_graph.eraseExperienceGraph();
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();

        if (center_count != 0) {
            planner->manip_graph.loadExperienceGraph(root_egraph_dir);
        }

	    moveit_msgs::RobotTrajectory traj;
	    double intercept_time;

        PlanPathParams params;
        params.allowed_time = 20;
        params.rc_constrained = false;
        params.shortcut_prerc = false;
        params.only_check_success = false;
        if (center_count == 0) {
            params.shortcut_prerc = true;
        }

	    if (!PlanRobotPath(planner, start_state, goal_pose, &traj, intercept_time, params)) {
        	SMPL_INFO("Unable to plan to the center within time %f",
        		params.allowed_time);

        	planner->hkey_dijkstra.markDirtyState(state_id);
        	planner->hkey_dijkstra.removeStateFromUncoveredList(state_id);
        	continue;
	    }

	    // write path to file
	    auto egraph_dir = "/home/fislam/paths/" + std::to_string(center_count);
        // auto sys_ret = system(("exec rm -r " + egraph_dir + "/*").c_str());
	    WritePath(planner->robot_model, start_state, traj, egraph_dir, intercept_time);

        if (center_count == 0) {
            WritePath(planner->robot_model, start_state, traj, root_egraph_dir, planner->replan_cutoff_);
        }

	    //////////////////////////////////
	    //     Reachability Search      //
	    //////////////////////////////////

	    ReinitDijkstras(planner, center_state);
		int covered_count = 0;
		int iter = 0;
	    while (true) {

	    	// Get next object state"

	    	// 	- For the first iteration it should be the center state
	    	auto state_id = planner->hkey_dijkstra.getNextStateId();
			if (state_id == -1) {	// OPEN empty or all states covered
				break;
			}
			else if (state_id == -2) {		// State already covered
				continue;
			}
			auto object_state = planner->object_graph.extractState(state_id);

			SMPL_INFO("		*********************************************************");
		    SMPL_INFO("		********   Next State: %.2f, %.2f, %f \t id: %d   ********",
		    					object_state[0], object_state[1], object_state[2], state_id);
		    SMPL_INFO("		*********************************************************");

		    ///////////////////////////////////////
		    //     Compute path to next state    //
		    ///////////////////////////////////////

	        // compute correspinding pose for center_state;
		    auto object_pose = ComputeObjectPose(object_state, height);
            auto goal_pose = object_pose * grasps[0].inverse();

		    // update collision checker for the new object pose
		    planner->manip_checker->setObjectInitialPose(object_pose);

			// refresh all datastructures
			planner->manip_graph.eraseExperienceGraph();
        	planner->egraph_planner->force_planning_from_scratch_and_free_memory();
        	planner->manip_graph.loadExperienceGraph(egraph_dir);

	        moveit_msgs::RobotTrajectory traj;
	        double intercept_time;

            PlanPathParams params;
            params.allowed_time = planner->time_bound_;
            params.rc_constrained = true;
            params.only_check_success = true;
            if (iter == 0) {
                params.rc_constrained = false;
                params.only_check_success = false;
                params.shortcut_prerc = true;
            }
	        if (!PlanRobotPath(planner, start_state, goal_pose, &traj, intercept_time, params)) {
                //=========================================================================//
                // dont need to mark state as dirty here, can just store the original path //
                //=========================================================================//
	        	SMPL_INFO("		Pose is NOT reachable");
	        	SMPL_INFO("-----------------------------------------------------------\n\n\n");
	        	if (iter == 0) {
	        		planner->hkey_dijkstra.markDirtyState(state_id);
	        		planner->hkey_dijkstra.removeStateFromUncoveredList(state_id);
                    auto sys_ret = system(("exec rm -r " + egraph_dir + "/*").c_str());
	        	}
	        }
	        else {
	        	if (iter == 0) {
				    // remove previous path and write path to file (to resolve mismatch)
				    // issue because of interpolation)
				    // auto sys_ret = system(("exec rm -r " + egraph_dir + "/*").c_str());
				    // WritePath(planner->robot_model, start_state, traj, egraph_dir, intercept_time);
                    // printf("second\n"); getchar();
				}
	        	planner->hkey_dijkstra.removeStateFromDirtyList(state_id); // if dirty
	        	planner->hkey_dijkstra.removeStateFromUncoveredList(state_id);
	        	planner->object_graph.setPathId(state_id, center_count);
	            SMPL_INFO("		Pose is reachable, path id: %d", center_count);
	            SMPL_INFO("-----------------------------------------------------------\n\n\n");
	        	covered_count++;
	        }
	        iter++;
	    }
	    if (covered_count > 0) {
	    	center_count++;
	    }
    	SMPL_INFO("No. center states %d", center_count);
    	// getchar();
	}
	SMPL_INFO("Preprocessing Completed");
	SMPL_INFO("No. center states %d", center_count);

	planner->object_graph.saveStateToPathIdMap();
	return true;
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

	int path_id = planner->object_graph.getPathId(object_state_grid);

	SMPL_INFO("#######    Query object state: %.2f, %.2f, %f    Path id: %d    #######", 
			object_state_grid[0], object_state_grid[1], object_state_grid[2], path_id);

	if (path_id == -1) {
		SMPL_ERROR("Query state is dirty or not covered");
		return false;
	}

    auto object_pose = ComputeObjectPose(object_state_grid, height);
    auto goal_pose = object_pose * grasps[0].inverse();

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(object_pose);

    // clear all memory
    planner->manip_graph.eraseExperienceGraph();
    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

	// load experience graph
    auto egraph_dir = "/home/fislam/paths/" + std::to_string(path_id);

    planner->manip_graph.loadExperienceGraph(egraph_dir);
    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
    	SMPL_ERROR("Failed to initialize Generic Egraph heuristic");
    	return false;
	}

    PlanPathParams params;
    params.allowed_time = planner->time_bound_ + 1e-3;
    params.rc_constrained = false;
    params.shortcut_prerc = true;
    params.only_check_success = false;
    if (!PlanRobotPath(planner, start_state, goal_pose, trajectory, intercept_time, params)) {
		SMPL_INFO("Unable to plan to the center within time %f",
		planner->time_bound_);
		return false;
	}

	return true;
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

	SMPL_INFO("#######    Normal Planner: Query object state: %.2f %.2f %.2f    #######", 
			object_state_grid[0], object_state_grid[1], object_state_grid[2]);

    auto object_pose = ComputeObjectPose(object_state_grid, height);
    auto goal_pose = object_pose * grasps[0].inverse();

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(object_pose);

    // clear all memory
    planner->manip_graph.eraseExperienceGraph();
    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
    	SMPL_ERROR("Failed to initialize Generic Egraph heuristic");
    	return false;
	}

    PlanPathParams params;
    params.allowed_time = 1000.0;
    params.rc_constrained = false;
    params.shortcut_prerc = true;
    params.only_check_success = false;
    if (!PlanRobotPath(planner, start_state, goal_pose, trajectory, intercept_time, params)) {
		SMPL_INFO("Unable to plan within allowed time %f",
		params.allowed_time);
		return false;
	}

	return true;
}

bool QueryAllTestsPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height)
{
	auto state_id = planner->hkey_dijkstra.sampleObjectState();
	if (state_id == -1) {
		return false;
	}

	auto center_state = planner->object_graph.extractState(state_id);

	ReinitDijkstras(planner, center_state);
	int dirty_count = 0;
	while (true) {
		auto state_id = planner->hkey_dijkstra.getNextStateId();
		if (state_id == -1) {	// OPEN empty or all states covered
			SMPL_INFO("All object states tested successfully, remaining dirty states %d", dirty_count);
			return true;
		}
		auto object_state = planner->object_graph.extractState(state_id);

		int path_id = planner->object_graph.getPathId(object_state);

		SMPL_INFO("#######    Query object state: %.2f, %.2f, %f    id: %d     Dirty count: %d   #######", 
				object_state[0], object_state[1], object_state[2], state_id, dirty_count);

		if (path_id == -1) {
			SMPL_ERROR("Query state %d is dirty or not covered", state_id);
			dirty_count++;
			continue;
		}

		printf("Path id is %d\n", path_id);

	    auto object_pose = ComputeObjectPose(object_state, height);
        auto goal_pose = object_pose * grasps[0].inverse();

	    // update collision checker for the new object pose
	    planner->manip_checker->setObjectInitialPose(object_pose);

	    // clear all memory
	    planner->manip_graph.eraseExperienceGraph();
	    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

		// load experience graph
	    auto egraph_dir = "/home/fislam/paths/" + std::to_string(path_id);

	    planner->manip_graph.loadExperienceGraph(egraph_dir);
	    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
	    	SMPL_ERROR("Failed to initialize Generic Egraph heuristic");
	    	return false;
		}

		moveit_msgs::RobotTrajectory traj;
	    double intercept_time;
        PlanPathParams params;
        params.allowed_time = planner->time_bound_ + 1e-3;
        params.rc_constrained = false;
        params.only_check_success = true;
        if (!PlanRobotPath(planner, start_state, goal_pose, &traj, intercept_time, params)) {
			SMPL_INFO("Unable to plan to the center within time %f",
			planner->time_bound_);
			getchar();
		}
	}

	return true;
}