////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2012, Benjamin Cohen
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

/// \author Benjamin Cohen
#include "conveyor_kdl_robot_model.h"

// standard includes
#include <stdlib.h>
#include <string>
#include <thread>
#include <chrono>         // std::chrono::seconds
#include <algorithm>
#include <set>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
// #include <smpl/ros/planner_interface.h>
#include <smpl/distance_map/edge_euclid_distance_map.h>
#include <smpl/distance_map/euclid_distance_map.h>
#include <smpl/ros/propagation_distance_field.h>
#include <sbpl_collision_checking/collision_space.h>
#include <sbpl_collision_checking/shapes.h>
#include <visualization_msgs/MarkerArray.h>
#include <smpl/angles.h>
#include <smpl/debug/marker_conversions.h>
#include <smpl/debug/visualizer_ros.h>
#include <smpl/stl/memory.h>

#include "collision_space_scene.h"
#include "pr2_allowed_collision_pairs.h"


// action
#include <pr2_controllers_msgs/JointTrajectoryAction.h>
#include <pr2_controllers_msgs/Pr2GripperCommandAction.h>
#include <actionlib/client/simple_action_client.h>

// gazebo
#include "gazebo_msgs/ModelStates.h"

#include "conveyor_planner_interface.h"
#include "conveyor_manip_checker.h"


#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>

#include <control_msgs/FollowJointTrajectoryAction.h>

using GripperCommandActionClient = actionlib::SimpleActionClient<
        pr2_controllers_msgs::Pr2GripperCommandAction>;

using FollowJointTrajectoryActionClient =
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

// std::unique_ptr<GripperCommandActionClient> gripper_command_client;
// std::unique_ptr<GripperCommandActionClient> gripper_command_client;
std::unique_ptr<FollowJointTrajectoryActionClient> follow_joint_trajectory_client;

class GripperMachine
{
public:
    std::unique_ptr<GripperCommandActionClient> gripper_command_client;
    bool OpenGripper();
    bool CloseGripper();
};


bool GripperMachine::OpenGripper()
{
    // open the gripper
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.position = 0.08;
    goal.command.max_effort = -1.0; // do not limit effort

    ROS_INFO("Send open gripper goal!");
    gripper_command_client->sendGoal(goal);
    // auto state = gripper_command_client->sendGoalAndWait(goal);
    // auto res = gripper_command_client->getResult();

    // ROS_INFO("Result:");
    // if (res) {
    //     ROS_INFO("  Effort: %f", res->effort);
    //     ROS_INFO("  Position %f", res->position);
    //     ROS_INFO("  Reached Goal: %d", res->reached_goal);
    //     ROS_INFO("  Stalled: %d", res->stalled);
    // }

    // if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_ERROR("Failed to open gripper (%s)", state.getText().c_str());
    //     return false;
    // }

    return true;
}

bool GripperMachine::CloseGripper()
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 50.0; // gently
    ROS_INFO("Send close gripper goal!");
    gripper_command_client->sendGoal(goal);
    // auto res = gripper_command_client->getResult();

    // ROS_INFO("Result:");
    // if (res) {
    //     ROS_INFO("  Effort: %f", res->effort);
    //     ROS_INFO("  Position %f", res->position);
    //     ROS_INFO("  Reached Goal: %d", res->reached_goal);
    //     ROS_INFO("  Stalled: %d", res->stalled);
    // }

    // if (state != actionlib::SimpleClientGoalState::SUCCEEDED) {
    //     ROS_ERROR("Failed to close gripper (%s)", state.getText().c_str());
    //     return false;
    // }

    return true;
}


void FillGoalConstraint(
    const std::vector<double>& pose,
    std::string frame_id,
    moveit_msgs::Constraints& goals)
{
    if (pose.size() < 6) {
        return;
    }

    goals.position_constraints.resize(1);
    goals.orientation_constraints.resize(1);
    goals.position_constraints[0].header.frame_id = frame_id;

    goals.position_constraints[0].constraint_region.primitives.resize(1);
    goals.position_constraints[0].constraint_region.primitive_poses.resize(1);
    goals.position_constraints[0].constraint_region.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.x = pose[0];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.y = pose[1];
    goals.position_constraints[0].constraint_region.primitive_poses[0].position.z = pose[2];

//    goals.position_constraints[0].position.x = pose[0];
//    goals.position_constraints[0].position.y = pose[1];
//    goals.position_constraints[0].position.z = pose[2];

    Eigen::Quaterniond q;
    smpl::angles::from_euler_zyx(pose[5], pose[4], pose[3], q);
    tf::quaternionEigenToMsg(q, goals.orientation_constraints[0].orientation);

    geometry_msgs::Pose p;
    p.position = goals.position_constraints[0].constraint_region.primitive_poses[0].position;
    p.orientation = goals.orientation_constraints[0].orientation;
    leatherman::printPoseMsg(p, "Goal");

    /// set tolerances
    goals.position_constraints[0].constraint_region.primitives[0].dimensions.resize(3, 0.015);
    goals.orientation_constraints[0].absolute_x_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_y_axis_tolerance = 0.05;
    goals.orientation_constraints[0].absolute_z_axis_tolerance = 0.05;

    ROS_INFO("Done packing the goal constraints message.");
}

auto GetCollisionCube(
    const geometry_msgs::Pose& pose,
    std::vector<double>& dims,
    const std::string& frame_id,
    const std::string& id)
    -> moveit_msgs::CollisionObject
{
    moveit_msgs::CollisionObject object;
    object.id = id;
    object.operation = moveit_msgs::CollisionObject::ADD;
    object.header.frame_id = frame_id;
    object.header.stamp = ros::Time::now();

    shape_msgs::SolidPrimitive box_object;
    box_object.type = shape_msgs::SolidPrimitive::BOX;
    box_object.dimensions.resize(3);
    box_object.dimensions[0] = dims[0];
    box_object.dimensions[1] = dims[1];
    box_object.dimensions[2] = dims[2];

    object.primitives.push_back(box_object);
    object.primitive_poses.push_back(pose);
    return object;
}

auto GetCollisionCubes(
    std::vector<std::vector<double>>& objects,
    std::vector<std::string>& object_ids,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    std::vector<moveit_msgs::CollisionObject> objs;
    std::vector<double> dims(3,0);
    geometry_msgs::Pose pose;
    pose.orientation.x = 0;
    pose.orientation.y = 0;
    pose.orientation.z = 0;
    pose.orientation.w = 1;

    if (object_ids.size() != objects.size()) {
        ROS_INFO("object id list is not same length as object list. exiting.");
        return objs;
    }

    for (size_t i = 0; i < objects.size(); i++) {
        pose.position.x = objects[i][0];
        pose.position.y = objects[i][1];
        pose.position.z = objects[i][2];
        dims[0] = objects[i][3];
        dims[1] = objects[i][4];
        dims[2] = objects[i][5];

        objs.push_back(GetCollisionCube(pose, dims, frame_id, object_ids.at(i)));
    }
    return objs;
}

auto GetCollisionObjects(
    const std::string& filename,
    const std::string& frame_id)
    -> std::vector<moveit_msgs::CollisionObject>
{
    char sTemp[1024];
    int num_obs = 0;
    std::vector<std::string> object_ids;
    std::vector<std::vector<double> > objects;
    std::vector<moveit_msgs::CollisionObject> objs;

    FILE* fCfg = fopen(filename.c_str(), "r");

    if (fCfg == NULL) {
        ROS_INFO("ERROR: unable to open objects file. Exiting.\n");
        return objs;
    }

    // get number of objects
    if (fscanf(fCfg,"%s",sTemp) < 1) {
        printf("Parsed string has length < 1.\n");
    }

    num_obs = atoi(sTemp);

    ROS_INFO("%i objects in file",num_obs);

    //get {x y z dimx dimy dimz} for each object
    objects.resize(num_obs);
    object_ids.clear();
    for (int i=0; i < num_obs; ++i) {
        if (fscanf(fCfg,"%s",sTemp) < 1) {
            printf("Parsed string has length < 1.\n");
        }
        object_ids.push_back(sTemp);

        objects[i].resize(6);
        for (int j=0; j < 6; ++j)
        {
            if (fscanf(fCfg,"%s",sTemp) < 1) {
                printf("Parsed string has length < 1.\n");
            }
            if (!feof(fCfg) && strlen(sTemp) != 0) {
                objects[i][j] = atof(sTemp);
            }
        }
    }

    return GetCollisionCubes(objects, object_ids, frame_id);
}

bool ReadInitialConfiguration(
    ros::NodeHandle& nh,
    moveit_msgs::RobotState& state)
{
    XmlRpc::XmlRpcValue xlist;

    // joint_state
    if (nh.hasParam("initial_configuration/joint_state")) {
        nh.getParam("initial_configuration/joint_state", xlist);

        if (xlist.getType() != XmlRpc::XmlRpcValue::TypeArray) {
            ROS_WARN("initial_configuration/joint_state is not an array.");
        }

        if (xlist.size() > 0) {
            for (int i = 0; i < xlist.size(); ++i) {
                state.joint_state.name.push_back(std::string(xlist[i]["name"]));

                if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeDouble) {
                    state.joint_state.position.push_back(double(xlist[i]["position"]));
                }
                else {
                    ROS_DEBUG("Doubles in the yaml file have to contain decimal points. (Convert '0' to '0.0')");
                    if (xlist[i]["position"].getType() == XmlRpc::XmlRpcValue::TypeInt) {
                        int pos = xlist[i]["position"];
                        state.joint_state.position.push_back(double(pos));
                    }
                }
            }
        }
    }
    else {
        ROS_WARN("initial_configuration/joint_state is not on the param server.");
    }

    // multi_dof_joint_state
    if (nh.hasParam("initial_configuration/multi_dof_joint_state")) {
        nh.getParam("initial_configuration/multi_dof_joint_state", xlist);

        if (xlist.getType() == XmlRpc::XmlRpcValue::TypeArray) {
            if (xlist.size() != 0) {
                auto &multi_dof_joint_state = state.multi_dof_joint_state;
                multi_dof_joint_state.joint_names.resize(xlist.size());
                multi_dof_joint_state.transforms.resize(xlist.size());
                for (int i = 0; i < xlist.size(); ++i) {
                    multi_dof_joint_state.joint_names[i] = std::string(xlist[i]["joint_name"]);

                    Eigen::Quaterniond q;
                    smpl::angles::from_euler_zyx(
                            (double)xlist[i]["yaw"], (double)xlist[i]["pitch"], (double)xlist[i]["roll"], q);

                    geometry_msgs::Quaternion orientation;
                    tf::quaternionEigenToMsg(q, orientation);

                    multi_dof_joint_state.transforms[i].translation.x = xlist[i]["x"];
                    multi_dof_joint_state.transforms[i].translation.y = xlist[i]["y"];
                    multi_dof_joint_state.transforms[i].translation.z = xlist[i]["z"];
                    multi_dof_joint_state.transforms[i].rotation.w = orientation.w;
                    multi_dof_joint_state.transforms[i].rotation.x = orientation.x;
                    multi_dof_joint_state.transforms[i].rotation.y = orientation.y;
                    multi_dof_joint_state.transforms[i].rotation.z = orientation.z;
                }
            } else {
                ROS_WARN("initial_configuration/multi_dof_joint_state array is empty");
            }
        } else {
            ROS_WARN("initial_configuration/multi_dof_joint_state is not an array.");
        }
    }

    ROS_INFO("Read initial state containing %zu joints and %zu multi-dof joints", state.joint_state.name.size(), state.multi_dof_joint_state.joint_names.size());
    return true;
}

struct RobotModelConfig
{
    std::string group_name;
    std::vector<std::string> planning_joints;
    std::string kinematics_frame;
    std::string chain_tip_link;
};

bool ReadRobotModelConfig(const ros::NodeHandle &nh, RobotModelConfig &config)
{
    if (!nh.getParam("group_name", config.group_name)) {
        ROS_ERROR("Failed to read 'group_name' from the param server");
        return false;
    }

    std::string planning_joint_list;
    if (!nh.getParam("planning_joints", planning_joint_list)) {
        ROS_ERROR("Failed to read 'planning_joints' from the param server");
        return false;
    }

    std::stringstream joint_name_stream(planning_joint_list);
    while (joint_name_stream.good() && !joint_name_stream.eof()) {
        std::string jname;
        joint_name_stream >> jname;
        if (jname.empty()) {
            continue;
        }
        config.planning_joints.push_back(jname);
    }

    // only required for generic kdl robot model?
    nh.getParam("kinematics_frame", config.kinematics_frame);
    nh.getParam("chain_tip_link", config.chain_tip_link);
    return true;
}

struct PlannerConfig
{
    std::string discretization;
    std::string mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    bool shortcut_path;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("mprim_filename", config.mprim_filename)) {
        ROS_ERROR("Failed to read param 'mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyz_snap_mprim", config.use_xyz_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_rpy_snap_mprim", config.use_rpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_rpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_xyzrpy_snap_mprim", config.use_xyzrpy_snap_mprim)) {
        ROS_ERROR("Failed to read param 'use_xyzrpy_snap_mprim' from the param server");
        return false;
    }

    if (!nh.getParam("use_short_dist_mprims", config.use_short_dist_mprims)) {
        ROS_ERROR("Failed to read param 'use_short_dist_mprims' from the param server");
        return false;
    }

    if (!nh.getParam("shortcut_path", config.shortcut_path)) {
        ROS_ERROR("Failed to read param 'shortcut_path' from the param server");
        return false;
    }

    if (!nh.getParam("xyz_snap_dist_thresh", config.xyz_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyz_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("rpy_snap_dist_thresh", config.rpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'rpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("xyzrpy_snap_dist_thresh", config.xyzrpy_snap_dist_thresh)) {
        ROS_ERROR("Failed to read param 'xyzrpy_snap_dist_thresh' from the param server");
        return false;
    }

    if (!nh.getParam("short_dist_mprims_thresh", config.short_dist_mprims_thresh)) {
        ROS_ERROR("Failed to read param 'use_xyz_snap_mprim' from the param server");
        return false;
    }

    return true;
}

auto SetupRobotModel(const std::string& urdf, const RobotModelConfig &config)
    -> std::unique_ptr<ConveyorKDLRobotModel>
{
    if (config.kinematics_frame.empty() || config.chain_tip_link.empty()) {
        ROS_ERROR("Failed to retrieve param 'kinematics_frame' or 'chain_tip_link' from the param server");
        return NULL;
    }

    ROS_INFO("Construct Generic KDL Robot Model");
    std::unique_ptr<ConveyorKDLRobotModel> rm(new ConveyorKDLRobotModel);

    if (!rm->init(urdf, config.kinematics_frame, config.chain_tip_link)) {
        ROS_ERROR("Failed to initialize robot model.");
        return NULL;
    }

    return std::move(rm);
}

typedef actionlib::SimpleActionClient< pr2_controllers_msgs::JointTrajectoryAction > TrajClient;

moveit_msgs::RobotState current_state;
moveit::core::RobotStatePtr robot_state_ptr;
class RobotArm
{
private:
  // Action client for the joint trajectory action 
  // used to trigger the arm movement action
  TrajClient* traj_client_;

public:
  //! Initialize the action client and wait for action server to come up
  RobotArm() 
  {
    // tell the action client that we want to spin a thread by default
    traj_client_ = new TrajClient("r_arm_controller/joint_trajectory_action", true);

    // wait for action server to come up
    while(!traj_client_->waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the joint_trajectory_action server");
    }
  }

  //! Clean up the action client
  ~RobotArm()
  {
    delete traj_client_;
  }


  //! Sends the command to start a given trajectory
  void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal, bool wait)
  {
    // auto state = traj_client_->sendGoalAndWait(goal);
    ROS_INFO("Sending goal\n");
    goal.trajectory.header.stamp = ros::Time::now(); // + ros::Duration(8);

    if (wait) {
        // traj_client_->sendGoalAndWait(goal);

        // ROS_INFO("Before time");
    auto start = std::chrono::steady_clock::now();

    auto state = follow_joint_trajectory_client->sendGoalAndWait(goal);

    auto end = std::chrono::steady_clock::now();
    std::cout << "Execution time: " 
        << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
    }
    else {
        follow_joint_trajectory_client->sendGoal(goal);
        // auto state = follow_joint_trajectory_client->sendGoal(goal);
        // traj_client_->sendGoal(goal);   
    }
  }

  //! Generates a simple trajectory with two waypoints, used as an example
  /*! Note that this trajectory contains two waypoints, joined together
      as a single trajectory. Alternatively, each of these waypoints could
      be in its own trajectory - a trajectory can have one or more waypoints
      depending on the desired application.
  */


  control_msgs::FollowJointTrajectoryGoal armHomeTrajectory(moveit_msgs::RobotState start_state)
  {
    //our goal variable
    // pr2_controllers_msgs::JointTrajectoryGoal goal;
    control_msgs::FollowJointTrajectoryGoal goal;
            // goal.trajectory = plan.trajectory_.joint_trajectory;

    // First, the joint names, which apply to all waypoints
    goal.trajectory.joint_names.push_back("r_shoulder_pan_joint");
    goal.trajectory.joint_names.push_back("r_shoulder_lift_joint");
    goal.trajectory.joint_names.push_back("r_upper_arm_roll_joint");
    goal.trajectory.joint_names.push_back("r_elbow_flex_joint");
    goal.trajectory.joint_names.push_back("r_forearm_roll_joint");
    goal.trajectory.joint_names.push_back("r_wrist_flex_joint");
    goal.trajectory.joint_names.push_back("r_wrist_roll_joint");

    // We will have two waypoints in this goal trajectory
    goal.trajectory.points.resize(1);
    // goal.trajectory.header.stamp = ros::Time::now() + ros::Duration(1.0);

    // First trajectory point
    // Positions
    int ind = 0;
    goal.trajectory.points[ind].positions.resize(7);
    for (size_t i = 0; i < 7; ++i) {
        goal.trajectory.points[ind].positions[i] = start_state.joint_state.position[i + 1];
    }
    // Velocities
    goal.trajectory.points[ind].velocities.resize(7);
    for (size_t j = 0; j < 7; ++j)
    {
      goal.trajectory.points[ind].velocities[j] = 0.0;
    }
    // To be reached 1 second after starting along the trajectory
    goal.trajectory.points[ind].time_from_start = ros::Duration(3.0);

    // Second trajectory point
    // ind++;
    // // goal.trajectory.points[ind + 1] = goal.trajectory.points[ind];
    // goal.trajectory.points[ind].positions.resize(7);
    // for (size_t i = 0; i < 7; ++i) {
    //     goal.trajectory.points[ind].positions[i] = start_state.joint_state.position[i + 1];
    // }
    // goal.trajectory.points[ind].time_from_start = ros::Duration(5.0);

    return goal;
  }

  //! Returns the current state of the action
  actionlib::SimpleClientGoalState getState()
  {
    return traj_client_->getState();
  }
 
};

double object_y;
void objectStateCallback(const gazebo_msgs::ModelStates::ConstPtr& msg)
{
    // printf("hey\n");
    object_y = msg->pose[1].position.y;
}



void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msg)
{
    // printf("received\n");
    moveit::core::jointStateToRobotState(*msg, *robot_state_ptr);
    moveit::core::robotStateToRobotStateMsg(*robot_state_ptr, current_state);
    // for (size_t i = 0; i < 8; ++i) {
    //     current_state.joint_state.position[i] = msg->position[i + 13];
    // }
}

void getLeftArmState(const moveit_msgs::RobotState& robot_state, smpl::RobotState& l_arm_state)
{
    std::vector<std::string> joint_names = {"l_shoulder_pan_joint",
                "l_shoulder_lift_joint",
                "l_upper_arm_roll_joint",
                "l_elbow_flex_joint",
                "l_forearm_roll_joint",
                "l_wrist_flex_joint",
                "l_wrist_roll_joint"};

    for (size_t i = 0; i < joint_names.size(); ++i) {
        for (size_t j = 0; j < robot_state.joint_state.name.size(); ++j) {
            // printf("name %s\n", robot_state.joint_state.name[j].c_str());
            if (joint_names[i] == robot_state.joint_state.name[j]) {
                l_arm_state.push_back(robot_state.joint_state.position[j]);  // order is important
            }
        }
    }
}
// int main(int argc, char* argv[])
// {
//     ros::init(argc, argv, "smpl_test");
//     ros::NodeHandle nh;
//     ros::NodeHandle ph("~");

//     // ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, objectStateCallback);
//     // ros::Subscriber sub_joint_vals = nh.subscribe("/joint_states", 1000, jointStateCallback);

//     gripper_command_client.reset(new GripperCommandActionClient(
//             "r_gripper_controller/gripper_action"));

//     moveit_msgs::RobotState start_state;
//     if (!ReadInitialConfiguration(ph, start_state)) {
//         ROS_ERROR("Failed to get initial configuration.");
//         return 1;
//     }

//     while(ros::ok())
//     {
//         ros::spinOnce();
//         OpenGripper();
//         ros::Duration(1.0).sleep();
//         CloseGripper();
//     }
//     // RobotArm arm;
//     // arm.startTrajectory(arm.armHomeTrajectory(start_state), true);
//     // CloseGripper();

//     return 0;
// }

#if 1
int main(int argc, char* argv[])
{
    ros::init(argc, argv, "smpl_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");

    ros::Subscriber sub = nh.subscribe("/gazebo/model_states", 1000, objectStateCallback);
    ros::Subscriber sub_joint_vals = nh.subscribe("/joint_states", 1000, jointStateCallback);

    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    // Let publishers set up
    ros::Duration(1.0).sleep();

    /////////////////
    // Robot Model //
    /////////////////

    // get the nominal urdf/srdf
    auto loader = robot_model_loader::RobotModelLoader();
    auto robot_model = loader.getModel();
    if (robot_model == NULL) {
        ROS_ERROR("Failed to load Robot Model");
        return 1;
    }

    // auto state = moveit::core::RobotState(robot_model);
    
    robot_state_ptr = std::make_shared<moveit::core::RobotState>(robot_model);

    ROS_INFO("Load common parameters");

    // Robot description required to initialize collision checker and robot
    // model...
    auto robot_description_key = "robot_description";
    std::string robot_description_param;
    if (!nh.searchParam(robot_description_key, robot_description_param)) {
        ROS_ERROR("Failed to find 'robot_description' key on the param server");
        return 1;
    }

    std::string robot_description;
    if (!nh.getParam(robot_description_param, robot_description)) {
        ROS_ERROR("Failed to retrieve param 'robot_description' from the param server");
        return 1;
    }

    RobotModelConfig robot_config;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model"), robot_config)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    RobotModelConfig robot_config_larm;
    if (!ReadRobotModelConfig(ros::NodeHandle("~robot_model_larm"), robot_config_larm)) {
        ROS_ERROR("Failed to read robot model config from param server");
        return 1;
    }

    // Everyone needs to know the name of the planning frame for reasons...
    // ...frame_id for the occupancy grid (for visualization)
    // ...frame_id for collision objects (must be the same as the grid, other than that, useless)
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ////////////////////
    // Occupancy Grid //
    ////////////////////

    ROS_INFO("Initialize Occupancy Grid");

    auto df_size_x = 3.0;
    auto df_size_y = 5.0;
    auto df_size_z = 3.0;
    auto df_res = 0.02;
    auto df_origin_x = -0.75;
    auto df_origin_y = -1.5;
    auto df_origin_z = 0.0;
    auto max_distance = 1.8;

    using DistanceMapType = smpl::EuclidDistanceMap;

    auto df = std::make_shared<DistanceMapType>(
            df_origin_x, df_origin_y, df_origin_z,
            df_size_x, df_size_y, df_size_z,
            df_res,
            max_distance);

    auto ref_counted = false;
    smpl::OccupancyGrid grid(df, ref_counted);

    grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(grid.getBoundingBoxVisualization());

    //////////////////////////////////
    // Initialize Collision Checker //
    //////////////////////////////////

    ROS_INFO("Initialize collision checker");

    // This whole manage storage for all the scene objects and must outlive
    // its associated CollisionSpace instance.
    CollisionSpaceScene scene;

    smpl::collision::CollisionModelConfig cc_conf;
    if (!smpl::collision::CollisionModelConfig::Load(ph, cc_conf)) {
        ROS_ERROR("Failed to load Collision Model Config");
        return 1;
    }

    smpl::collision::CollisionSpace cc;
    if (!cc.init(
            &grid,
            robot_description,
            cc_conf,
            robot_config.group_name,
            robot_config.planning_joints))
    {
        ROS_ERROR("Failed to initialize Collision Space");
        return 1;
    }

    if (cc.robotCollisionModel()->name() == "pr2") {
        smpl::collision::AllowedCollisionMatrix acm;
        for (auto& pair : PR2AllowedCollisionPairs) {
            acm.setEntry(pair.first, pair.second, true);
        }
        cc.setAllowedCollisionMatrix(acm);
    }

    /////////////////
    // Setup Scene //
    /////////////////

    ROS_INFO("Initialize scene");

    scene.SetCollisionSpace(&cc);

    std::string object_filename;
    ph.param<std::string>("object_filename", object_filename, "");

    // Read in collision objects from file and add to the scene...
    if (!object_filename.empty()) {
        auto objects = GetCollisionObjects(object_filename, planning_frame);
        for (auto& object : objects) {
            scene.ProcessCollisionObjectMsg(object);
        }
    }

    auto rm = SetupRobotModel(robot_description, robot_config);
    if (!rm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    // Read in start state from file and update the scene...
    // Start state is also required by the planner...
    moveit_msgs::RobotState start_state;
    if (!ReadInitialConfiguration(ph, start_state)) {
        ROS_ERROR("Failed to get initial configuration.");
        return 1;
    }

    // Set reference state in the robot planning model...
    smpl::urdf::RobotState reference_state;
    InitRobotState(&reference_state, &rm->m_robot_model);
    for (auto i = 0; i < start_state.joint_state.name.size(); ++i) {
        auto* var = GetVariable(&rm->m_robot_model, &start_state.joint_state.name[i]);
        if (var == NULL) {
            ROS_WARN("Failed to do the thing");
            continue;
        }
        ROS_INFO("Set joint %s to %f", start_state.joint_state.name[i].c_str(), start_state.joint_state.position[i]);
        SetVariablePosition(&reference_state, var, start_state.joint_state.position[i]);
    }
    SetReferenceState(rm.get(), GetVariablePositions(&reference_state));

    // Set reference state in the collision model...
    if (!scene.SetRobotState(start_state)) {
        ROS_ERROR("Failed to set start state on Collision Space Scene");
        return 1;
    }

    cc.setWorldToModelTransform(Eigen::Affine3d::Identity());

    SV_SHOW_INFO(grid.getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());


    // read object model

    std::vector<double> object_pose(6, 0);
    ph.param("conveyor_object/pose/x", object_pose[0], 0.0);
    ph.param("conveyor_object/pose/y", object_pose[1], 0.0);
    ph.param("conveyor_object/pose/z", object_pose[2], 0.0);
    ph.param("conveyor_object/pose/roll", object_pose[3], 0.0);
    ph.param("conveyor_object/pose/pitch", object_pose[4], 0.0);
    ph.param("conveyor_object/pose/yaw", object_pose[5], 0.0);

    std::vector<double> grasp(6, 0);
    ph.param("conveyor_object/grasp/x", grasp[0], 0.0);
    ph.param("conveyor_object/grasp/y", grasp[1], 0.0);
    ph.param("conveyor_object/grasp/z", grasp[2], 0.0);
    ph.param("conveyor_object/grasp/roll", grasp[3], 0.0);
    ph.param("conveyor_object/grasp/pitch", grasp[4], 0.0);
    ph.param("conveyor_object/grasp/yaw", grasp[5], 0.0);

    Eigen::Vector3d object_velocity;
    ph.param("conveyor_object/velocity/x", object_velocity[0], 0.0);
    ph.param("conveyor_object/velocity/y", object_velocity[1], 0.0);
    ph.param("conveyor_object/velocity/z", object_velocity[2], 0.0);

    std::string shape_type;
    double radius;
    double height;
    ph.param<std::string>("conveyor_object/shape", shape_type, "Cylinder");
    ph.param("conveyor_object/cylinder_params/radius", radius, 0.0);


    std::unique_ptr<smpl::collision::CollisionShape> collision_shape;
    if (shape_type == "Box") {
        double size_x, size_y, size_z;
        ph.param("conveyor_object/box_params/size_x", size_x, 0.0);
        ph.param("conveyor_object/box_params/size_y", size_y, 0.0);
        ph.param("conveyor_object/box_params/size_z", size_z, 0.0);
        collision_shape = smpl::make_unique<smpl::collision::BoxShape>(
                size_x,
                size_y,
                size_z);
    }
    else if (shape_type == "Cylinder")
    {
        double radius, height;
        ph.param("conveyor_object/cylinder_params/radius", radius, 0.0);
        ph.param("conveyor_object/cylinder_params/height", height, 0.0);
        collision_shape = smpl::make_unique<smpl::collision::CylinderShape>(radius, height);
    }
    else {
        assert(0);
    }

#if 1
    ////////////////////////////////////
    /////   LEFT ARM MECHANISM      ////
    ////////////////////////////////////

    auto rm_larm = SetupRobotModel(robot_description, robot_config_larm);
    if (!rm_larm) {
        ROS_ERROR("Failed to set up Robot Model");
        return 1;
    }

    GripperMachine l_gripper;
    l_gripper.gripper_command_client.reset(new GripperCommandActionClient(
                "l_gripper_controller/gripper_action"));
    if (!l_gripper.gripper_command_client->waitForServer(ros::Duration(10.0))) {
        ROS_ERROR("Gripper Action client not available");
        return 1;
    }
    ////////////////////////////////////
    /////   Open left arm gripper   ////
    ////////////////////////////////////

    // l_gripper.OpenGripper();
    // ROS_INFO("Hand me object and press enter");
    // getchar();

    ////////////////////////////////////
    /////   Close left arm gripper   ////
    ////////////////////////////////////
    l_gripper.CloseGripper();

    ////////////////////////////////////////////
    /////   Use left hand pose of object    ////
    ////////////////////////////////////////////
    ros::spinOnce();

    smpl::RobotState l_arm_state;
    getLeftArmState(current_state, l_arm_state);

    // for (auto v : l_arm_state) {
    //     printf("%f \n", v);
    // }
    // printf("\n");

    auto l_arm_pose = rm_larm->computeFK(l_arm_state);
    object_pose[0] = l_arm_pose.translation().x();
    object_pose[1] = l_arm_pose.translation().y();
    object_pose[2] = l_arm_pose.translation().z();

    printf("lgripper position %f %f %f\n", object_pose[0], object_pose[1], object_pose[2]);
    // getchar();
#endif

    Eigen::Affine3d object_transform(
                Eigen::Translation3d(object_pose[0], object_pose[1], object_pose[2]) *
                Eigen::AngleAxisd(object_pose[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(object_pose[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(object_pose[3], Eigen::Vector3d::UnitX()));

    Eigen::Affine3d grasp_transform(
                Eigen::Translation3d(grasp[0], grasp[1], grasp[2]) *
                Eigen::AngleAxisd(grasp[5], Eigen::Vector3d::UnitZ()) *
                Eigen::AngleAxisd(grasp[4], Eigen::Vector3d::UnitY()) *
                Eigen::AngleAxisd(grasp[3], Eigen::Vector3d::UnitX()));

    auto goal_transform = object_transform * grasp_transform.inverse();
    auto rot = goal_transform.rotation().eulerAngles(2, 1, 0);

    std::vector<double> goal(6, 0);
    goal[0] = goal_transform.translation().x();
    goal[1] = goal_transform.translation().y();
    goal[2] = goal_transform.translation().z();
    goal[3] = rot[2];
    goal[4] = rot[1];
    goal[5] = rot[0];

    // Conveyor checker
    smpl::collision::CollisionObject collision_object;
    collision_object.shapes.push_back(collision_shape.get());
    collision_object.shape_poses.push_back(object_transform);
    ConveyorManipChecker cchecker;
    if (!Init(&cchecker, rm.get(), &rm->m_robot_model, &cc, &collision_object, object_velocity)) {
        return false;
    }
    ///////////////////
    // Planner Setup //
    ///////////////////

    PlannerConfig planning_config;
    if (!ReadPlannerConfig(ros::NodeHandle("~planning"), planning_config)) {
        ROS_ERROR("Failed to read planner config");
        return 1;
    }

    smpl::ConveyorPlannerInterface planner(rm.get(), &cchecker, &grid, object_velocity);

    smpl::PlanningParams params;

    params.addParam("discretization", planning_config.discretization);
    params.addParam("mprim_filename", planning_config.mprim_filename);
    params.addParam("use_xyz_snap_mprim", planning_config.use_xyz_snap_mprim);
    params.addParam("use_rpy_snap_mprim", planning_config.use_rpy_snap_mprim);
    params.addParam("use_xyzrpy_snap_mprim", planning_config.use_xyzrpy_snap_mprim);
    params.addParam("use_short_dist_mprims", planning_config.use_short_dist_mprims);
    params.addParam("xyz_snap_dist_thresh", planning_config.xyz_snap_dist_thresh);
    params.addParam("rpy_snap_dist_thresh", planning_config.rpy_snap_dist_thresh);
    params.addParam("xyzrpy_snap_dist_thresh", planning_config.xyzrpy_snap_dist_thresh);
    params.addParam("short_dist_mprims_thresh", planning_config.short_dist_mprims_thresh);
    params.addParam("epsilon", 100.0);
    params.addParam("search_mode", false);
    params.addParam("allow_partial_solutions", false);
    params.addParam("target_epsilon", 1.0);
    params.addParam("delta_epsilon", 1.0);
    params.addParam("improve_solution", false);
    params.addParam("bound_expansions", true);
    params.addParam("repair_time", 1.0);
    params.addParam("bfs_inflation_radius", 0.02);
    params.addParam("bfs_cost_per_cell", 100);

    params.shortcut_path = planning_config.shortcut_path;
    params.plan_output_dir = "/home/fislam/paths/";

    if (!planner.init(params)) {
        ROS_ERROR("Failed to initialize Planner Interface");
        return 1;
    }

    //////////////
    // Planning //
    //////////////

    moveit_msgs::MotionPlanRequest req;
    moveit_msgs::MotionPlanResponse res;

    ph.param("allowed_planning_time", req.allowed_planning_time, 10.0);
    req.goal_constraints.resize(1);
    FillGoalConstraint(goal, planning_frame, req.goal_constraints[0]);
    req.group_name = robot_config.group_name;
    req.max_acceleration_scaling_factor = 1.0;
    req.max_velocity_scaling_factor = 1.0;
    req.num_planning_attempts = 1;
//    req.path_constraints;
    req.planner_id = "arastar.conveyor.conveyor";
    // req.planner_id = "egwastar.euclid_egraph.conveyor_manip_lattice_egraph";
    req.start_state = start_state;
//    req.trajectory_constraints;
//    req.workspace_parameters;

    // plan
    ROS_INFO("Calling solve...");
    moveit_msgs::PlanningScene planning_scene;
    planning_scene.robot_state = start_state;
    if (!planner.solve(planning_scene, req, res)) {
        ROS_ERROR("Failed to plan.");
        return 1;
    }

    // profile

    // robot_trajectory::RobotTrajectory rt(robot_model, "right_arm");
    // rt.setRobotTrajectoryMsg(*robot_state_ptr, res.trajectory);
    // trajectory_processing::IterativeParabolicTimeParameterization iptp;

    // if (!iptp.computeTimeStamps(rt, 2.0, 2.0)) {
    //     ROS_ERROR("Failed to compute timestamps");
    // }

    // rt.getRobotTrajectoryMsg(res.startTrajectorytory);
    ///////////////////////////////////
    // Visualizations and Statistics //
    ///////////////////////////////////

    auto planning_stats = planner.getPlannerStats();

    ROS_INFO("Planning statistics");
    for (auto& entry : planning_stats) {
        ROS_INFO("    %s: %0.3f", entry.first.c_str(), entry.second);
    }

    double intercept_time = planning_stats["intercept time"];

#if 1
    ROS_INFO("Animate path");
    size_t pidx = 0;
    while (ros::ok()) {
        auto& point = res.trajectory.joint_trajectory.points[pidx];
        auto full_state = point.positions;
        full_state.push_back(point.time_from_start.toSec());
        auto markers_smpl = cchecker.getCollisionModelVisualization(full_state);
        auto markers = smpl::visual::ConvertMarkersToMarkerArray(markers_smpl);
        for (auto& m : markers.markers) {
            m.ns = "path_animation";
        }
        SV_SHOW_INFO(markers);
        if (pidx != res.trajectory.joint_trajectory.points.size() - 1) {
            auto& point_next = res.trajectory.joint_trajectory.points[pidx + 1];
            auto time = point_next.time_from_start - point.time_from_start;
            ros::Duration(time).sleep();
        }
        pidx++;
        pidx %= res.trajectory.joint_trajectory.points.size();
    }
    return 1;

#else
    /////////////////////////////
    /////   Init  machines   ////
    /////////////////////////////

    // gripper client
    GripperMachine r_gripper;
    r_gripper.gripper_command_client.reset(new GripperCommandActionClient(
                "r_gripper_controller/gripper_action"));
    if (!r_gripper.gripper_command_client->waitForServer(ros::Duration(10.0))) {
        ROS_ERROR("Gripper Action client not available");
        return 1;
    }

    // trajectory client
    follow_joint_trajectory_client.reset(new FollowJointTrajectoryActionClient(
            "r_arm_controller/follow_joint_trajectory"));

    if (!follow_joint_trajectory_client->waitForServer(ros::Duration(10.0))) {
        ROS_ERROR("Follow Joint Trajectory client not available");
        return 1;
    }

    /////////////////////////////////////////////////////
    /////   Move right arm to home and open gripper  ////
    /////////////////////////////////////////////////////

    RobotArm arm;
    arm.startTrajectory(arm.armHomeTrajectory(start_state), true);
    r_gripper.OpenGripper();


    // // goal trajectory
    control_msgs::FollowJointTrajectoryGoal goal_arm;
    goal_arm.trajectory = res.trajectory.joint_trajectory;

    printf("start %f end %f\n", goal_arm.trajectory.points[0].time_from_start.toSec(),
        goal_arm.trajectory.points.back().time_from_start.toSec());

    ROS_INFO("press key"); getchar();
    l_gripper.OpenGripper();

    arm.startTrajectory(goal_arm, false);
    // arm.startTrajectory(goal_arm, true);
    // ROS_INFO("Sleeping");
    // ros::Duration(intercept_time).sleep();
    std::this_thread::sleep_for(std::chrono::microseconds(int(intercept_time * 1000000.0)));
    ROS_INFO("closing gripper at intercept time %f", intercept_time);
    r_gripper.CloseGripper();

    // wait for gripper to close
    std::this_thread::sleep_for(std::chrono::microseconds(int(1.0 * 1000000.0)));

    ROS_INFO("Back home");
    arm.startTrajectory(arm.armHomeTrajectory(start_state), true);
    r_gripper.OpenGripper();
    return 1;

#endif
    // while(ros::ok())
    // {
    //     ros::spinOnce();
    //     if (std::fabs(object_y - goal[1]) < 0.001 || true) {
    //         // printf("press to execute pick\n");
    //         // getchar();

    //         arm.startTrajectory(goal_arm, false);
    //         ros::Duration(intercept_time).sleep();
    //         printf("closing gripper\n");
    //         CloseGripper();
    //         // while (!arm.getState().isDone()) {
    //         //     std::this_thread::sleep_for(std::chrono::milliseconds(20)); 
    //         // }
    //         // break;
    //     }
    // }
    // return 1;
}

#endif
