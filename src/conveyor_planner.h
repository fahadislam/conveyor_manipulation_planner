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

#ifndef SMPL_CONVEYOR_PLANNER_H
#define SMPL_CONVEYOR_PLANNER_H

// searches
#include "hkey_dijkstra.h"
#include <smpl/search/experience_graph_planner.h>

// heuristics
#include "conveyor_manip_heuristic.h"
#include <smpl/heuristic/generic_egraph_heuristic.h>
#include <smpl/heuristic/joint_dist_heuristic.h>

// graphs
// #include <smpl/graph/manip_lattice.h>
#include "conveyor_object_lattice.h"
#include <smpl/graph/manip_lattice_action_space.h>

#include "conveyor_manip_lattice_egraph.h"
#include "conveyor_manip_lattice_action_space.h"

#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/RobotTrajectory.h>

class ConveyorManipChecker;
class ConveyorObjectModel;
class ConveyorKDLRobotModel;
class ConveyorBoundsChecker;

using ObjectState = smpl::RobotState;	// { x, y, yaw}

struct SingleJointTrajectory
{
	std::vector<double> times;
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

struct PlanPathParams
{
	double allowed_time;
	bool rc_constrained;
	bool shortcut_prerc;
	bool only_check_success;
	// bool post_process;
	std::string singleton_dir;
};

struct ConveyorPlanner
{
	// checkers
    ConveyorBoundsChecker*                  object_checker;
    ConveyorManipChecker*					manip_checker;

    // models
    ConveyorObjectModel*                    object_model;
    ConveyorKDLRobotModel*					robot_model;

	// planners
	smpl::HKeyDijkstra 						hkey_dijkstra;
	smpl::ExperienceGraphPlanner 			*egraph_planner;

	// graphs
    smpl::ConveyorObjectLattice				object_graph;
    smpl::ManipLatticeActionSpace 			object_actions;

    smpl::ConveyorManipLatticeEgraph 		manip_graph;
    smpl::ConveyorManipLatticeActionSpace 	manip_actions;

    // heuristics
    smpl::JointDistHeuristic 			    object_heuristic;
    smpl::ConveyorManipHeuristic 			manip_heuristic;
    smpl::GenericEgraphHeuristic 			egraph_manip_heuristic;

    double interp_resolution_;
    double replan_resolution_;
    double time_bound_;
    double replan_cutoff_;
    moveit_msgs::RobotState home_state_;
    std::vector<std::vector<smpl::RobotState>> home_paths_;
    std::vector<smpl::RobotState> current_path_;
    int current_path_id_;
    int start_id_;
    std::string main_dir_;

    // global variables for recursion
    bool home_query = true;

    ConveyorPlanner() :
        hkey_dijkstra(&object_graph, &object_heuristic)
    { }
};

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
    const smpl::PlanningParams* manip_params,
    bool preprocessing = false);

bool PreprocessConveyorPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    std::vector<int>& G_REM,
    std::vector<int>& G_COV);

bool PreprocessConveyorPlannerMain(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& home_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height);

bool QueryConstTimePlanner(
	ConveyorPlanner* planner,
	const moveit_msgs::RobotState& start_state,
	const std::vector<Eigen::Affine3d>& grasps,
    const ObjectState& object_state,
    double height,
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time);

bool QueryNormalPlanner(
	ConveyorPlanner* planner,
	const moveit_msgs::RobotState& start_state,
	const std::vector<Eigen::Affine3d>& grasps,
    const ObjectState& object_state,
    double height,
    moveit_msgs::RobotTrajectory* trajectory,
    double& intercept_time);

bool QueryAllTestsPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height);

bool QueryReplanningTestsPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests);

bool QueryRandomTestsNormalPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests);

bool QueryRandomTestsConstTimePlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height,
    int num_tests);

bool QueryAllTestsNormalPlanner(
    ConveyorPlanner* planner,
    const moveit_msgs::RobotState& start_state,
    const std::vector<Eigen::Affine3d>& grasps,
    double height);

#endif