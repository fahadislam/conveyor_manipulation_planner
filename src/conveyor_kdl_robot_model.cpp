
#include "conveyor_kdl_robot_model.h"

#include <sbpl_kdl_robot_model/kdl_robot_model.h>

// system includes
#include <eigen_conversions/eigen_kdl.h>
#include <kdl/frames.hpp>
#include <kdl_parser/kdl_parser.hpp>
#include <leatherman/print.h>
#include <leatherman/utils.h>
#include <ros/console.h>
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/stl/memory.h>

// #include <fcl/geometry/bvh/BVH_model.h>
// #include <fcl/narrowphase/collision_object.h>
// #include <fcl/narrowphase/collision_request.h> // CollisionRequest, CollisionResult
// #include <fcl/narrowphase/collision.h> // collide
#include <geometric_shapes/mesh_operations.h>

#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>

bool ConveyorKDLRobotModel::init(
    const std::string& robot_description,
    const std::string& base_link,
    const std::string& tip_link,
    int free_angle)
{
    if (!KDLRobotModel::init(
            robot_description, base_link, tip_link, free_angle))
    {
        return false;
    }

    m_cart_to_jnt_vel_solver = smpl::make_unique<KDL::ChainIkSolverVel_pinv>(m_chain);

    return true;
}

bool ConveyorKDLRobotModel::computeInverseVelocity(
        const smpl::RobotState& jnt_positions,
        const std::vector<double>& cart_velocities,
        smpl::RobotState& jnt_velocities)
{
    KDL::JntArray  q_(jnt_positions.size());
    KDL::JntArray  qdot_(jnt_positions.size());
    KDL::Twist     xdot_;
    jnt_velocities.resize(jnt_positions.size());

    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
    }

    xdot_.vel(0) = cart_velocities[0];
    xdot_.vel(1) = cart_velocities[1];
    xdot_.vel(2) = cart_velocities[2];
    xdot_.rot(0) = cart_velocities[3];
    xdot_.rot(1) = cart_velocities[4];
    xdot_.rot(2) = cart_velocities[5];

    if (!m_cart_to_jnt_vel_solver->CartToJnt(q_, xdot_, qdot_) < 0) {
        ROS_WARN("Failed to find inverse joint velocities");
        return false;
    }

    for (size_t i = 0; i < jnt_velocities.size(); ++i) {
        jnt_velocities[i] = qdot_(i);
    }
    return true;
}

auto ConveyorKDLRobotModel::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::InverseKinematicsInterface>()
        || class_code == smpl::GetClassCode<InverseVelocityInterface>()
        || class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>()) return this;

    return URDFRobotModel::getExtension(class_code);
}
