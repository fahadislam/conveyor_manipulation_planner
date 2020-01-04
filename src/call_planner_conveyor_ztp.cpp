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
#include "conveyor_kdl_robot_model.h"

// standard includes
#include <stdlib.h>
#include <string>
#include <thread>
#include <vector>

// system includes
#include <eigen_conversions/eigen_msg.h>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <moveit_msgs/GetMotionPlan.h>
#include <moveit_msgs/PlanningScene.h>
#include <ros/ros.h>
#include <kdl_conversions/kdl_msg.h>
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
#include <actionlib/server/simple_action_server.h>
#include <control_msgs/FollowJointTrajectoryAction.h>
#include <tf/transform_listener.h>

#include "conveyor_manip_checker.h"
#include "conveyor_planner.h"
#include "conveyor_object_model.h"

using GripperCommandActionClient = actionlib::SimpleActionClient<
        pr2_controllers_msgs::Pr2GripperCommandAction>;

using FollowJointTrajectoryActionClient =
        actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>;

std::unique_ptr<tf::TransformListener> g_listener;
geometry_msgs::PoseStamped g_pose_msg;

bool g_request_active = false;
double g_time_first_estimate = 0.0;
double g_time_current_estimate = 0.0;
double g_time_perception = 0.0;


enum  PlannerMode
{
    NORMAL_QUERY = 0,
    CONST_TIME_PLAN,
    CONST_TIME_REPLAN,
    ALL_TESTS_QUERY,
    PREPROCESS
};

enum  ExecutionMode
{
    SIMULATION = 0, // animation
    REAL_ROBOT_PERCEPTION,
    REAL_ROBOT_HARDCODED

};

class GripperMachine
{
public:
    bool init()
    {
        gripper_command_client.reset(new GripperCommandActionClient(
                    "r_gripper_controller/gripper_action"));
        if (!gripper_command_client->waitForServer(ros::Duration(1.0))) {
            ROS_ERROR("Gripper Action client not available");
        }
        return true;
    }

    std::unique_ptr<GripperCommandActionClient> gripper_command_client;
    bool OpenGripper(bool wait);
    bool CloseGripper(bool wait);
};


bool GripperMachine::OpenGripper(bool wait)
{
    // open the gripper
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.position = 0.08;
    goal.command.max_effort = -1.0; // do not limit effort

    ROS_INFO("Send open gripper goal!");
    if (wait) {
        auto state = gripper_command_client->sendGoalAndWait(goal);
    }
    else {
        gripper_command_client->sendGoal(goal);
    }

    return true;
}

bool GripperMachine::CloseGripper(bool wait)
{
    pr2_controllers_msgs::Pr2GripperCommandGoal goal;
    goal.command.position = 0.0;
    goal.command.max_effort = 50.0; // gently

    ROS_INFO("Send close gripper goal!");
    if (wait) {
        auto state = gripper_command_client->sendGoalAndWait(goal);
    }
    else {
        gripper_command_client->sendGoal(goal);
    }

    return true;
}

class RobotArm
{
private:
    std::unique_ptr<FollowJointTrajectoryActionClient> traj_client_;

public:
//! Initialize the action client and wait for action server to come up
    bool init()
    {
        // tell the action client that we want to spin a thread by default
        traj_client_.reset(new FollowJointTrajectoryActionClient(
            "r_arm_controller/follow_joint_trajectory"));

        // wait for action server to come up
        if(!traj_client_->waitForServer(ros::Duration(1.0))){
            ROS_ERROR("joint_trajectory_action server not available");
        }
        return true;
    }

    //! Sends the command to start a given trajectory
    void startTrajectory(control_msgs::FollowJointTrajectoryGoal goal, bool wait)
    {
        if (wait) {
            auto start = std::chrono::steady_clock::now();
            auto state = traj_client_->sendGoalAndWait(goal);
            auto end = std::chrono::steady_clock::now();
            std::cout << "Execution time: " 
                << std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count() << std::endl;
            }
        else {
            traj_client_->sendGoal(goal);
        }
    }

    control_msgs::FollowJointTrajectoryGoal armHomeTrajectory(moveit_msgs::RobotState start_state)
    {
        control_msgs::FollowJointTrajectoryGoal goal;

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

        // Positions
        goal.trajectory.points[0].positions.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            goal.trajectory.points[0].positions[i] = start_state.joint_state.position[i + 1];
        }
        // Velocities
        goal.trajectory.points[0].velocities.resize(7);
        for (size_t j = 0; j < 7; ++j)
        {
            goal.trajectory.points[0].velocities[j] = 0.0;
        }
        goal.trajectory.points[0].time_from_start = ros::Duration(2.0);

        return goal;
    }

};

namespace smpl {

/// Add a simple box obstacle in the center of the grid
void SetupOccupancyGrid(smpl::OccupancyGrid& grid)
{
    const int x_count = grid.numCellsX();
    const int y_count = grid.numCellsY();
    const int z_count = grid.numCellsZ();

    std::vector<Eigen::Vector3d> points;

    // add horizontal strip down the middle, with holes on the ends
    for (int gx = 1; gx < x_count - 1; ++gx) {
        double cx, cy, cz;
        grid.gridToWorld(gx, y_count >> 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) + 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);

        grid.gridToWorld(gx, (y_count >> 1) - 1, 0, cx, cy, cz);
        points.emplace_back(cx, cy, cz);
    }

    SMPL_INFO("Add %zu points to grid", points.size());
    grid.addPointsToField(points);
}

} // namespace

//------------------------------------------------

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
    std::string manip_mprim_filename;
    bool use_xyz_snap_mprim;
    bool use_rpy_snap_mprim;
    bool use_xyzrpy_snap_mprim;
    bool use_short_dist_mprims;
    bool shortcut_path;
    double xyz_snap_dist_thresh;
    double rpy_snap_dist_thresh;
    double xyzrpy_snap_dist_thresh;
    double short_dist_mprims_thresh;

    // object manip params
    std::string object_mprim_filename;
    double resolution_xy;
    double resolution_yaw;
    double origin_x;
    double origin_y;
    double size_x;
    double size_y;
    double time_bound;
    double buffer_time;
};

bool ReadPlannerConfig(const ros::NodeHandle &nh, PlannerConfig &config)
{
    if (!nh.getParam("discretization", config.discretization)) {
        ROS_ERROR("Failed to read 'discretization' from the param server");
        return false;
    }

    if (!nh.getParam("manip_mprim_filename", config.manip_mprim_filename)) {
        ROS_ERROR("Failed to read param 'maniip_mprim_filename' from the param server");
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

    // object params
    if (!nh.getParam("object_mprim_filename", config.object_mprim_filename)) {
        ROS_ERROR("Failed to read param 'object_mprim_filename' from the param server");
        return false;
    }

    if (!nh.getParam("resolution_xy", config.resolution_xy)) {
        ROS_ERROR("Failed to read param 'resolution_xy' from the param server");
        return false;
    }

    if (!nh.getParam("resolution_yaw", config.resolution_yaw)) {
        ROS_ERROR("Failed to read param 'resolution_yaw' from the param server");
        return false;
    }

    if (!nh.getParam("origin_x", config.origin_x)) {
        ROS_ERROR("Failed to read param 'origin_x' from the param server");
        return false;
    }

    if (!nh.getParam("origin_y", config.origin_y)) {
        ROS_ERROR("Failed to read param 'origin_y' from the param server");
        return false;
    }

    if (!nh.getParam("size_x", config.size_x)) {
        ROS_ERROR("Failed to read param 'size_x' from the param server");
        return false;
    }

    if (!nh.getParam("size_y", config.size_y)) {
        ROS_ERROR("Failed to read param 'size_y' from the param server");
        return false;
    }

    if (!nh.getParam("time_bound", config.time_bound)) {
        ROS_ERROR("Failed to read param 'time_bound' from the param server");
        return false;
    }

    if (!nh.getParam("buffer_time", config.buffer_time)) {
        ROS_ERROR("Failed to read param 'buffer_time' from the param server");
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

bool TryHardTransformPose(
    const std::string& frame_id,
    const geometry_msgs::PoseStamped& pose_in,
    geometry_msgs::PoseStamped& pose_out)
{
    while (true) {
        try {
            if (!ros::ok()) return false;
            g_listener->transformPose(frame_id, pose_in, pose_out);
            return true;
        }
        catch (tf::TransformException ex){
            ROS_ERROR("%s",ex.what());
            ros::Duration(0.01).sleep();
        }
    }
}

void DopePoseCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    ROS_INFO("Dope pose callback!!");
    g_pose_msg = *msg;
    g_time_current_estimate = g_pose_msg.header.stamp.toSec();
    g_time_perception = ros::Time::now().toSec() - g_time_current_estimate;
    if (!g_request_active) {
        g_time_first_estimate = g_time_current_estimate;
    }
    g_request_active = true;
}

bool GetNewObjectGoal(std::vector<double>& object_pose)
{
    while (ros::ok() && !g_request_active) {
        ros::Duration(0.001).sleep();
    }
    std::string frame_id = "base_footprint";
    if (!g_listener->waitForTransform(frame_id, g_pose_msg.header.frame_id, ros::Time(0), ros::Duration(10.0))) {
        ROS_WARN("Transform not available after waiting");
        return false;
    }
    else {
        geometry_msgs::PoseStamped pose_out;       
        TryHardTransformPose(frame_id, g_pose_msg, pose_out);
        object_pose[0] = pose_out.pose.position.x;
        object_pose[1] = pose_out.pose.position.y;

        Eigen::Quaterniond q;
        q.x() = pose_out.pose.orientation.x;
        q.y() = pose_out.pose.orientation.y;
        q.z() = pose_out.pose.orientation.z;
        q.w() = pose_out.pose.orientation.w;
        double pitch, roll;
        smpl::angles::get_euler_zyx(q, object_pose[2], pitch, roll);
        // printf("r: %f, p: %f, y: %f\n", roll, pitch, object_pose[2]);
    }
    SMPL_INFO("Object state [perceived]: %.2f %.2f %f",
                object_pose[0], object_pose[1], object_pose[2]);
    return true;
}

void AnimatePath(
    ConveyorManipChecker* m_cc,
    const moveit_msgs::RobotTrajectory* traj)
{
    ROS_INFO("Animate path");
    size_t pidx = 0;
    while (ros::ok()) {
        auto& point = traj->joint_trajectory.points[pidx];
        auto full_state = point.positions;
        full_state.push_back(point.time_from_start.toSec());
        auto markers_smpl = m_cc->getCollisionModelVisualization(full_state);
        auto markers = smpl::visual::ConvertMarkersToMarkerArray(markers_smpl);
        for (auto& m : markers.markers) {
            m.ns = "path_animation";
        }
        SV_SHOW_INFO(markers);
        // if (pidx != traj->joint_trajectory.points.size() - 1) {
        //     auto& point_next = traj->joint_trajectory.points[pidx + 1];
        //     auto time = point_next.time_from_start - point.time_from_start;
        //     ros::Duration(time).sleep();
        // }
        printf("time %f\n", point.time_from_start.toSec());
        getchar();
        pidx++;
        pidx %= traj->joint_trajectory.points.size();
    }
}

bool PickupObject(
    moveit_msgs::RobotTrajectory* traj,
    double intercept_time,
    moveit_msgs::RobotState start_state,
    GripperMachine *gripper,
    RobotArm* arm)
{
    // Start pick up
    static ros::Time g_first_exec_stamp;
    control_msgs::FollowJointTrajectoryGoal goal_conveyor;
    goal_conveyor.trajectory = traj->joint_trajectory;
    ROS_INFO("Starting pick up!");

    //============================================================================//
    //  if it's the object's first trajectory then store the execution timestamp  //
    //  if it's a replan trajectory then reuse the stored first timestamp         //
    //============================================================================//

    if (traj->joint_trajectory.points[0].time_from_start.toSec() < 1e-6) {
        goal_conveyor.trajectory.header.stamp = ros::Time::now();
        g_first_exec_stamp = goal_conveyor.trajectory.header.stamp; 
    }
    else {
        goal_conveyor.trajectory.header.stamp = g_first_exec_stamp;
    }

    if (g_first_exec_stamp.toSec() + traj->joint_trajectory.points[0].time_from_start.toSec() 
        > ros::Time::now().toSec()) {
        ROS_ERROR("Replan execution time has passed, fix your bug!");
    }

    arm->startTrajectory(goal_conveyor, false);

    //==========================================================//
    // wait until intercept time or until updated pose received //
    //==========================================================//

    auto begin = ros::Time::now();
    while (true) {
        auto elapsed_time = ros::Time::now() - begin;
        auto pose_time_since = g_time_current_estimate - begin.toSec();
        // printf("begin time %f current %f\n", begin.toSec(), g_time_current_estimate);
        if (elapsed_time.toSec() > intercept_time) {    // time to intercept
            break;
        }
        if (pose_time_since > 1e-6) {   // updated pose_out
            ROS_WARN("New pose received; returning");
            return false;
        }
        ros::Duration(0.001).sleep();
    }

    // Close gripper
    ROS_INFO("Closing gripper");
    gripper->CloseGripper(true);

    // Go back to home configuration
    ROS_INFO("Going back to home configuration");
    arm->startTrajectory(arm->armHomeTrajectory(start_state), true);
    gripper->OpenGripper(true);
    return true;
}

bool PickupObjectAtOffset(
    moveit_msgs::RobotTrajectory* traj,
    double time_offset,
    double intercept_time,
    moveit_msgs::RobotState start_state,
    GripperMachine *gripper,
    RobotArm* arm)
{
    auto t_now = ros::Time::now();
    auto t_pick = g_pose_msg.header.stamp + ros::Duration(time_offset);
    auto t_wait = t_pick - t_now;
    if (t_wait.toSec() < 0.0) {
        ROS_ERROR("Sorry you missed the object by %f secs", t_wait.toSec());
        return false;
    }
    
    ROS_INFO("Wait time before pickup: %f", t_wait.toSec());
    ros::Duration(t_wait).sleep();

    return PickupObject(traj, intercept_time, start_state, gripper, arm);
}

bool ExecutePickup(
    ExecutionMode execution_mode,
    double time_offset,
    double intercept_time,
    moveit_msgs::RobotState start_state,
    ConveyorManipChecker* manip_cchecker,
    moveit_msgs::RobotTrajectory* traj,
    GripperMachine *gripper,
    RobotArm* arm)
{
    switch (execution_mode) {
    case ExecutionMode::SIMULATION:
    {
        AnimatePath(manip_cchecker, traj);
        return true;
    }
    case ExecutionMode::REAL_ROBOT_HARDCODED:
    {
        return PickupObject(traj, intercept_time, start_state, gripper, arm);
    }
    case ExecutionMode::REAL_ROBOT_PERCEPTION:
    {
        return PickupObjectAtOffset(traj, time_offset, intercept_time, start_state, gripper, arm);
    }
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "conveyor_test");
    ros::NodeHandle nh;
    ros::NodeHandle ph("~");


    g_listener.reset(new tf::TransformListener);

    ros::Subscriber sub_dope = nh.subscribe("/dope/pose_sugar", 1000, DopePoseCallback);

    ros::AsyncSpinner spinner(2);
    spinner.start();

    ROS_INFO("Initialize visualizer");
    smpl::VisualizerROS visualizer(nh, 100);
    smpl::viz::set_visualizer(&visualizer);

    ///////////////////////
    // Robot Robot Model //
    ///////////////////////

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

    // Everyone needs to know the name of the planning frame for reasons...
    // ...frame_id for the occupancy grid (for visualization)
    // ...frame_id for collision objects (must be the same as the grid, other than that, useless)
    std::string planning_frame;
    if (!ph.getParam("planning_frame", planning_frame)) {
        ROS_ERROR("Failed to retrieve param 'planning_frame' from the param server");
        return 1;
    }

    ///////////////////////
    // Read object model //
    ///////////////////////


    std::vector<double> grasp(6, 0);
    ph.param("conveyor_object/grasp/x", grasp[0], 0.0);
    ph.param("conveyor_object/grasp/y", grasp[1], 0.0);
    ph.param("conveyor_object/grasp/z", grasp[2], 0.0);
    ph.param("conveyor_object/grasp/roll", grasp[3], 0.0);
    ph.param("conveyor_object/grasp/pitch", grasp[4], 0.0);
    ph.param("conveyor_object/grasp/yaw", grasp[5], 0.0);

    double object_height;
    ph.param("conveyor_object/height", object_height, 0.0);

    Eigen::Affine3d grasp_transform(
            Eigen::Translation3d(grasp[0], grasp[1], grasp[2]) *
            Eigen::AngleAxisd(grasp[5], Eigen::Vector3d::UnitZ()) *
            Eigen::AngleAxisd(grasp[4], Eigen::Vector3d::UnitY()) *
            Eigen::AngleAxisd(grasp[3], Eigen::Vector3d::UnitX()));

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
    smpl::OccupancyGrid manip_grid(df, ref_counted);

    manip_grid.setReferenceFrame(planning_frame);
    SV_SHOW_INFO(manip_grid.getBoundingBoxVisualization());

    /////////////////////////////////////////
    // Initialize Parent Collision Checker //
    /////////////////////////////////////////

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
            &manip_grid,
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

    SV_SHOW_INFO(manip_grid.getDistanceFieldVisualization(0.2));

    SV_SHOW_INFO(cc.getCollisionRobotVisualization());
    SV_SHOW_INFO(cc.getCollisionWorldVisualization());
    SV_SHOW_INFO(cc.getOccupiedVoxelsVisualization());
    ros::Duration(1.0).sleep();

    ///////////////////////////////////////
    // Initialize Conveyor Manip Checker //
    ///////////////////////////////////////

    smpl::collision::CollisionObject collision_object;
    collision_object.shapes.push_back(collision_shape.get());
    ConveyorManipChecker manip_cchecker;
    if (!Init(&manip_cchecker, rm.get(), &rm->m_robot_model, &cc, &collision_object, object_velocity)) {
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


    smpl::PlanningParams params;

    params.addParam("discretization", planning_config.discretization);
    params.addParam("manip_mprim_filename", planning_config.manip_mprim_filename);
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

    // conveyor params
    params.addParam("object_mprim_filename", planning_config.object_mprim_filename);
    params.addParam("resolution_xy", planning_config.resolution_xy);
    params.addParam("resolution_yaw", planning_config.resolution_yaw);
    params.addParam("origin_x", planning_config.origin_x);
    params.addParam("origin_y", planning_config.origin_y);
    params.addParam("size_x", planning_config.size_x);
    params.addParam("size_y", planning_config.size_y);
    params.addParam("time_bound", planning_config.time_bound);
    params.addParam("buffer_time", planning_config.buffer_time);

    params.shortcut_path = planning_config.shortcut_path;

    //////////////////////////////////
    //   Initialize Object Model    //
    //////////////////////////////////

    // Create Object Model
    ConveyorObjectModel object_model;

    // Create and Initialize the Environment
    const double grid_res = planning_config.resolution_xy;
    const double world_size_x = planning_config.size_x;
    const double world_size_y = planning_config.size_y;
    const double world_size_z = planning_config.resolution_xy;
    const double world_origin_x = planning_config.origin_x;
    const double world_origin_y = planning_config.origin_y;
    const double world_origin_z = 0.0;
    const double max_distance_m = 4.0;
    const bool ref_count = false;
    smpl::OccupancyGrid object_grid(
            world_size_x, world_size_y, world_size_z,
            grid_res,
            world_origin_x, world_origin_y, world_origin_z,
            max_distance_m,
            ref_count);
    SetupOccupancyGrid(object_grid);

    //  Create Bounds Checker
    ConveyorBoundsChecker object_bounds_checker(&object_grid);


    //////////////////////////////////////
    //   Initialize Conveyor Planner    //
    //////////////////////////////////////

    ConveyorPlanner conveyor_planner;
    if (!Init(
			&conveyor_planner,
	    	&object_bounds_checker,
	    	&manip_cchecker,
	    	&object_model,
	    	rm.get(),
	    	&object_grid,
	    	&manip_grid,
            object_velocity,
	    	&params)) {
    	return 1;
    }

    std::vector<Eigen::Affine3d> grasps;
    grasps.push_back(grasp_transform);
    // TODO: pass all grasps

    // PlannerMode planner_mode = PlannerMode::CONST_TIME_PLAN;
    PlannerMode planner_mode = PlannerMode::CONST_TIME_REPLAN;
    // PlannerMode planner_mode = PlannerMode::NORMAL_QUERY;
    // PlannerMode planner_mode = PlannerMode::PREPROCESS;
    // PlannerMode planner_mode = PlannerMode::ALL_TESTS_QUERY;

    // ExecutionMode execution_mode = ExecutionMode::SIMULATION;
    // ExecutionMode execution_mode = ExecutionMode::REAL_ROBOT_HARDCODED;
    ExecutionMode execution_mode = ExecutionMode::REAL_ROBOT_PERCEPTION;

    bool ret_plan, ret_exec;
    double intercept_time;
    double time_offset;

    // std::vector<double> object_state = {0.53, 1.39, -2.268929}; // for hardcoded modes
    // std::vector<double> object_state = {0.40, 1.05, 0.0}; // invalid path example
    std::vector<double> object_state = {0.40, 1.30, 1.0}; //{0.50, 1.37, 1.134464}; // invalid path example
    std::vector<double> object_state_old = {0.420000, 1.320000, 2.356194}; // INCONSISTENT GOAL
    std::vector<double> object_state_new = {0.450000, 1.380000, 0.0}; // INCONSISTENT GOAL

    moveit_msgs::RobotTrajectory traj;

    // Init gripper and traj clients
    RobotArm arm;
    GripperMachine gripper;

    if (execution_mode != ExecutionMode::SIMULATION && planner_mode != PlannerMode::PREPROCESS) {
    // go home
        arm.init();
        gripper.init();
        arm.startTrajectory(arm.armHomeTrajectory(start_state), true);
        gripper.OpenGripper(true);
    }

    start_state.joint_state.name.push_back("time");
    start_state.joint_state.position.push_back(0.0);

    moveit_msgs::RobotState replan_state;
    while(ros::ok()) {
        // Wait for the object pose if using perception
        if (execution_mode == ExecutionMode::REAL_ROBOT_PERCEPTION) {
            ROS_INFO("Waiting for the object...");
            object_state.resize(3);
            if (!GetNewObjectGoal(object_state)) {
                ROS_ERROR("Failed to find object goal pose");
            }

            //=============================================================================//
            // add offset to object state to account for the perception and planning times //
            //=============================================================================//

            time_offset = g_time_perception + ( planning_config.time_bound + planning_config.buffer_time );
            printf("time offset %f + %f = %f\n",
                    g_time_perception, planning_config.time_bound + planning_config.buffer_time, time_offset);

            object_state[1] += time_offset * object_velocity[1];
            
            //=========================================================================================//
            // time_elapsed is the time duration between the first and current estimate of the objects //
            // 1. if time_elapsed = 0 it means it is the first estimate of the object                  //
            // 2. if time_elapsed > 0 it means it is a subsequent estimate and we need to replan       //
            //      - in this case we compute the new object pose in retrospect                        //
            //=========================================================================================//

            double time_elapsed = g_time_current_estimate - g_time_first_estimate;
            printf("time elapsed since first pose received: %f\n", time_elapsed);

            if (time_elapsed < 1e-6) {
                object_state_old = object_state;
            }
            else {
                replan_state = start_state;
                replan_state.joint_state.velocity.resize(8);
                // replan_state.joint_state.effort.resize(8);
                object_state_new = object_state;
                double dist_moved = fabs(time_elapsed * object_velocity[1]);
                object_state_new[1] += dist_moved;

                // find the new start state
                for (size_t i = 0; i < traj.joint_trajectory.points.size(); ++i) {
                    if (traj.joint_trajectory.points[i].time_from_start.toSec() > time_elapsed + time_offset) {
                        for (size_t j = 0; j < 7; ++j) {
                            replan_state.joint_state.position[j + 1] = traj.joint_trajectory.points[i].positions[j];
                            replan_state.joint_state.velocity[j + 1] = traj.joint_trajectory.points[i].velocities[j];
                            // replan_state.joint_state.effort[j + 1] = traj.joint_trajectory.points[i].accelerations[j];
                        }
                        replan_state.joint_state.position[8] = traj.joint_trajectory.points[i].time_from_start.toSec();
                        break;
                    }
                }

                SMPL_INFO("Replan from time: %f", replan_state.joint_state.position[8]);
                planner_mode = PlannerMode::CONST_TIME_REPLAN;
            }
        }

        switch (planner_mode) {
        case PlannerMode::CONST_TIME_PLAN:
        {
            ret_plan = QueryConstTimePlanner(
                        &conveyor_planner,
                        start_state,
                        grasps,
                        object_state,
                        object_height,
                        &traj,
                        intercept_time);
            break;
        }
        case PlannerMode::CONST_TIME_REPLAN:
        {
            ret_plan = QueryConstTimeReplanner(
                        &conveyor_planner,
                        replan_state,
                        grasps,
                        object_state_old,
                        object_state_new,
                        object_height,
                        &traj,
                        intercept_time);
            break;
        }
        case PlannerMode::NORMAL_QUERY:
        {
            ret_plan = QueryNormalPlanner(
                        &conveyor_planner,
                        start_state,
                        grasps,
                        object_state,
                        object_height,
                        &traj,
                        intercept_time);
            break;
        }
        case PlannerMode::PREPROCESS:
        {
            ret_plan = PreprocessConveyorPlanner(
                &conveyor_planner,
                start_state,
                grasps,
                object_height);
            break;
        }
        case PlannerMode::ALL_TESTS_QUERY:
        {
            ret_plan = QueryAllTestsPlanner(
                &conveyor_planner,
                start_state,
                grasps,
                object_height);
            break;
        }
        default:
        {
            SMPL_ERROR("Unknown Planner Mode");
        }
        }

        if (planner_mode == PlannerMode::PREPROCESS ||
            planner_mode == PlannerMode::ALL_TESTS_QUERY) {
            break;
        }
        
        if (ret_plan) {
            SMPL_INFO("Total trajectory time: %f, intercept time: %f",
                    traj.joint_trajectory.points.back().time_from_start.toSec(), intercept_time);
            ret_exec = ExecutePickup(
                            execution_mode,
                            time_offset,
                            intercept_time,
                            start_state,
                            &manip_cchecker,
                            &traj,
                            &gripper,
                            &arm);
            if (ret_exec) {
                SMPL_INFO("Object picked up successfully!!");
            }
            else {
                SMPL_WARN("Execution Incomplete; request is still active");
                continue;   // g_request_active remains true;
            }
        }
        else {
            if (execution_mode == ExecutionMode::SIMULATION) {
                break;
            }
        }
        g_request_active = false;
        if (planner_mode = PlannerMode::CONST_TIME_REPLAN) {
            SMPL_INFO("Switching back to CONST_TIME_PLAN mode");
            planner_mode = PlannerMode::CONST_TIME_PLAN;
        }
    }

    return 1;
}
