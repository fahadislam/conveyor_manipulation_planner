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
#include "iterative_parabolic_time_parameterization.h"

#include <boost/filesystem.hpp>
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/utils.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/post_processing.h>
#include <smpl/ros/factories.h>
#include <smpl/stl/memory.h>
#include <smpl/time.h>

#include "conversions.h"

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

    std::vector<double> resolutions(robot->jointVariableCount());

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
    const std::vector<SingleJointTrajectory> joint_trajs,
    moveit_msgs::RobotTrajectory& traj)
{
    SMPL_INFO("Convert Variable Path to Robot Trajectory");

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
                p.accelerations[posidx] = joint_trajs[vidx].accelerations[pidx];
                p.time_from_start = ros::Duration(point.back());
            }
        }
    }
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

        if (traj.joint_trajectory.points[widx].time_from_start.toSec() >= intercept_time) {
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
        ofs << ',' << traj.joint_trajectory.points[widx].time_from_start;
        ofs << '\n';
    }

    return true;
}


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
    const Eigen::Vector3d& object_velocity,
    const smpl::PlanningParams* params)	// pass params
{
	planner->object_checker = object_checker;
	planner->manip_checker = manip_checker;
	planner->object_model = object_model;
	planner->robot_model = robot_model;

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

    if (!params->getParam("time_bound", planner->time_bound)) {
        SMPL_ERROR_NAMED(CP_LOGGER, "Parameter 'time_bound' not found in planning params");
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

void ShortcutPath(
	ConveyorPlanner* planner,
	double intercept_time,
	std::vector<smpl::RobotState>& path)
{
    // shortcut path piece-wise:
    // 		start -> intercept -> grasp -> lift

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

bool InterpolatePath(smpl::CollisionChecker* cc, std::vector<smpl::RobotState>& path)
{
    if (path.empty()) {
        return true;
    }

    auto num_joints = path.front().size();
    for (auto& pt : path) {
        if (pt.size() != num_joints) {
            SMPL_ERROR("Failed to interpolate trajectory. Input trajectory is malformed");
            return false;
        }
    }

    std::vector<smpl::RobotState> opath;

    // tack on the first point of the trajectory
    opath.push_back(path.front());

    // iterate over path segments
    for (auto i = size_t(0); i < path.size() - 1; ++i) {
        auto& curr = path[i];
        auto& next = path[i + 1];

        SMPL_DEBUG_STREAM("Interpolating between " << curr << " and " << next);

        std::vector<smpl::RobotState> ipath;
        if (!cc->interpolatePath(curr, next, ipath)) {
            SMPL_ERROR("Failed to interpolate between waypoint %zu and %zu because it's infeasible given the limits.", i, i + 1);
            return false;
        }

        // check the interpolated path for collisions, as the interpolator may
        // take a slightly different
        auto collision = false;
        for (auto& point : ipath) {
            if (!cc->isStateValid(point, false)) {
                collision = true;
                break;
            }
        }

        if (collision) {
            SMPL_ERROR("Interpolated path collides. Resorting to original waypoints");
            opath.push_back(next);
            continue;
        }

        if (!ipath.empty()) {
            // concatenate current path and the intermediate path (we already
            // have the first waypoint in the path from last iteration)
            opath.insert(end(opath), std::next(begin(ipath)), end(ipath));
        }

        SMPL_DEBUG("[%zu] path length: %zu", i, opath.size());
    }

    SMPL_INFO("Original path length: %zu   Interpolated path length: %zu", path.size(), opath.size());
    path = std::move(opath);
    return true;
}

bool PlanRobotPath(
	ConveyorPlanner* planner,
	const moveit_msgs::RobotState& start_state,
    const Eigen::Affine3d& object_pose,
    double allowed_time,
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time)
{
	std::vector<smpl::RobotState> path;

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

    planner->egraph_manip_heuristic.updateGoal(goal);

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

    smpl::RobotState initial_positions;
    std::vector<std::string> missing;
    if (!leatherman::getJointPositions(
            start_state.joint_state,
            start_state.multi_dof_joint_state,
            planner->robot_model->getPlanningJoints(),
            initial_positions,
            missing))
    {	
    	SMPL_ERROR("Start state is missing planning joints");
    	return false;
    }

    if (!planner->manip_graph.setStart(initial_positions)) {
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
    b_ret = planner->egraph_planner->replan(allowed_time, &solution_state_ids, &sol_cost);

    // if a path is returned, then pack it into msg form
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

        path.clear();
        if (!planner->manip_graph.extractPath(solution_state_ids, path)) {
            SMPL_ERROR("Failed to convert state id path to joint variable path");
            return false;
        }
    }
    else {
    	return b_ret;
    }

    intercept_time = planner->manip_graph.getInterceptTime(path);

    auto to_check_path = path;
    to_check_path.pop_back();

    if (!IsPathValid(planner->manip_checker, to_check_path)) {
    	SMPL_ERROR("Path is Invalid");
    	return false;
    }

    ////////////////////
    // Shortcut Path  //
    ////////////////////

    ShortcutPath(planner, intercept_time, path);

    /////////////////////////////////////////////////////////
    // Profile/Interpolate path and convert to trajectory  //
    /////////////////////////////////////////////////////////

    const double delta_time = 0.2;
    path = MakeInterpolatedTrajectory(path, delta_time);

    // fit spline again
    printf("Size of interpolated path: %zu\n", path.size());

    int num_joints = planner->robot_model->jointVariableCount();
    std::vector<SingleJointTrajectory> joint_trajs(num_joints);
    joint_trajs.resize(num_joints);
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

    for (size_t i = 0; i < num_joints; ++i) {
        joint_trajs[i].velocities[0] = 0.0; joint_trajs[i].velocities.back() = 0.0;
        genx::fit_cubic_spline(
                path.size(),
                &time_diff[0],
                &joint_trajs[i].positions[0],
                &joint_trajs[i].velocities[0],
                &joint_trajs[i].accelerations[0]);      
    }

    ConvertJointVariablePathToJointTrajectory(
            planner->robot_model,
            path,
            start_state.joint_state.header.frame_id,
            start_state.multi_dof_joint_state.header.frame_id,
            joint_trajs,
            *trajectory);

    return b_ret;
}

auto ComputeGoalPose(
	const ObjectState& object_state,
	const Eigen::Affine3d& grasp,
	double height)
	-> Eigen::Affine3d
{
	Eigen::Affine3d object_pose(
	                Eigen::Translation3d(object_state[0], object_state[1], height) *
	                Eigen::AngleAxisd(object_state[2], Eigen::Vector3d::UnitZ()) *
	                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitY()) *
	                Eigen::AngleAxisd(0.0, Eigen::Vector3d::UnitX()));

    return object_pose * grasp.inverse();
}

bool PreprocessConveyorPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height)
{
	double allowed_time_scratch = 20;

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
	    SMPL_INFO("********   Object State: %.2f %.2f %.2f \t id: %d   ********",
	    					center_state[0], center_state[1], center_state[2], state_id);
	    SMPL_INFO("*******************************************************************");

	    //////////////////////////////////////////////////////
	    // Compute path to object goal and store experience //
	    //////////////////////////////////////////////////////

	    auto goal_pose = ComputeGoalPose(center_state, grasps[0], height);

	    // update collision checker for the new object pose
	    planner->manip_checker->setObjectInitialPose(goal_pose);

        // clear all memory
        planner->manip_graph.eraseExperienceGraph();
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();

	    moveit_msgs::RobotTrajectory traj;
	    double intercept_time;
	    if (!PlanRobotPath(planner, start_state, goal_pose, allowed_time_scratch, &traj, intercept_time)) {
        	SMPL_INFO("Unable to plan to the center within time %f",
        		allowed_time_scratch);

        	// order of these two operations matter!
        	planner->hkey_dijkstra.markDirtyState(state_id);
        	continue;
	    }

        // clear all memory (may be unneeded here?)
        planner->manip_graph.eraseExperienceGraph();
        planner->egraph_planner->force_planning_from_scratch_and_free_memory();

	    SMPL_INFO("Total trajectory time %f, Intercept time: %f",
	    	traj.joint_trajectory.points.back().time_from_start.toSec(), intercept_time);

	    // remove previous path and write new path to file
	    auto egraph_dir = "/home/fislam/paths/" + std::to_string(center_count);
    	// auto sys_ret = system("exec rm -r /home/fislam/paths/*");
    	auto sys_ret = system(("exec rm -r " + egraph_dir + "/*").c_str());
	    WritePath(planner->robot_model, start_state, traj, egraph_dir, intercept_time);


	    planner->manip_graph.loadExperienceGraph(egraph_dir);
	    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
        	SMPL_ERROR("Failed to initialize Generic Egraph heuristic");
        	return false;
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
		    SMPL_INFO("		********   Next State: %.2f %.2f %.2f \t id: %d   ********",
		    					object_state[0], object_state[1], object_state[2], state_id);
		    SMPL_INFO("		*********************************************************");

		    ///////////////////////////////////////
		    //     Compute path to next state    //
		    ///////////////////////////////////////

	        // compute correspinding pose for center_state;
		    auto goal_pose = ComputeGoalPose(object_state, grasps[0], height);

		    // update collision checker for the new object pose
		    planner->manip_checker->setObjectInitialPose(goal_pose);

	        moveit_msgs::RobotTrajectory traj;
	        double intercept_time;
	        if (!PlanRobotPath(planner, start_state, goal_pose, planner->time_bound, &traj, intercept_time)) {
	        	SMPL_INFO("		Pose is NOT reachable");
	        	SMPL_INFO("-----------------------------------------------------------\n\n\n");
	        	if (iter == 0) {
	        		planner->hkey_dijkstra.markDirtyState(state_id);
	        	}
	        	// getchar();
	        }
	        else {
	        	planner->hkey_dijkstra.removeStateFromUncovered(state_id);
	        	planner->object_graph.setPathId(state_id, center_count);
	            SMPL_INFO("		Pose is reachable");
	            SMPL_INFO("-----------------------------------------------------------\n\n\n");
	        	covered_count++;
	        }
	        iter++;

	    }
	    if (covered_count > 0) {
	    	center_count++;
	    }
    	SMPL_INFO("No. center states %d", center_count);
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
	int path_id = planner->object_graph.getPathId(object_state);

	auto object_state_grid = planner->object_graph.getDiscreteCenter(object_state);

	SMPL_INFO("#######    Query object state: %.2f %.2f %.2f    #######", 
			object_state_grid[0], object_state_grid[1], object_state_grid[2]);

	if (path_id == -1) {
		SMPL_ERROR("Query state is dirty or not covered");
		return false;
	}

	printf("Path id is %d\n", path_id);

    //////////////////////////////////////////////////////
    // Compute path to object goal and store experience //
    //////////////////////////////////////////////////////

    auto goal_pose = ComputeGoalPose(object_state_grid, grasps[0], height);

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(goal_pose);

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

    if (!PlanRobotPath(planner, start_state, goal_pose, 1000.0, trajectory, intercept_time)) {
		SMPL_INFO("Unable to plan to the center within time %f",
		planner->time_bound);
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
	double allowed_time = 1000.0;
	auto object_state_grid = planner->object_graph.getDiscreteCenter(object_state);

	SMPL_INFO("#######    Normal Planner: Query object state: %.2f %.2f %.2f    #######", 
			object_state_grid[0], object_state_grid[1], object_state_grid[2]);

    //////////////////////////////////////////////////////
    // Compute path to object goal and store experience //
    //////////////////////////////////////////////////////

    auto goal_pose = ComputeGoalPose(object_state_grid, grasps[0], height);

    // update collision checker for the new object pose
    planner->manip_checker->setObjectInitialPose(goal_pose);

    // clear all memory
    planner->manip_graph.eraseExperienceGraph();
    planner->egraph_planner->force_planning_from_scratch_and_free_memory();

    if (!planner->egraph_manip_heuristic.init(&planner->manip_graph, &planner->manip_heuristic)) {
    	SMPL_ERROR("Failed to initialize Generic Egraph heuristic");
    	return false;
	}

    if (!PlanRobotPath(planner, start_state, goal_pose, allowed_time, trajectory, intercept_time)) {
		SMPL_INFO("Unable to plan within allowed time %f",
		allowed_time);
		return false;
	}

    SMPL_INFO("Total trajectory time %f, Intercept time: %f",
    	trajectory->joint_trajectory.points.back().time_from_start.toSec(), intercept_time);

	return true;
}

