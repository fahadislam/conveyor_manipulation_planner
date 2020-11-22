
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
    m_jnt_to_cart_vel_solver = smpl::make_unique<KDL::ChainFkSolverVel_recursive>(m_chain);
    KDL::Vector gravity(0.0, 0.0, 0.0);
    m_fd_solver = smpl::make_unique<KDL::ChainFdSolver_RNE>(m_chain, gravity);
    m_id_solver = smpl::make_unique<KDL::ChainIdSolver_RNE>(m_chain, gravity);

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

bool ConveyorKDLRobotModel::computeAccelerations(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        const smpl::RobotState& jnt_torques,
        smpl::RobotState& jnt_accelerations)
{
    KDL::JntArray  q_(jnt_positions.size());
    KDL::JntArray  qdot_(jnt_positions.size());
    KDL::JntArray  torques_(jnt_positions.size());
    KDL::JntArray  qdotdot_(jnt_positions.size());
    KDL::Wrenches wr(11);
    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
        qdot_(i) = jnt_velocities[i];
        torques_(i) = jnt_torques[i];
    }

    int ret = m_fd_solver->CartToJnt(q_, qdot_, torques_, wr, qdotdot_);
    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        jnt_accelerations[i] = qdotdot_(i);
        printf("i %zu acc %.2f torq %.2f\n", i, qdotdot_(i), torques_(i));
    }
    getchar();
    return true;
}

bool ConveyorKDLRobotModel::integrate(
        double t,
        double dt,
        smpl::RobotState& jnt_positions,
        smpl::RobotState& jnt_velocities,
        const smpl::RobotState& jnt_torques)
{
    KDL::JntArray  q_(jnt_positions.size());
    KDL::JntArray  qdot_(jnt_positions.size());
    KDL::JntArray  torques_(jnt_positions.size());
    KDL::JntArray  qdotdot_(jnt_positions.size());
    KDL::Wrenches wr(11);
    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
        qdot_(i) = jnt_velocities[i];
        torques_(i) = jnt_torques[i];
    }

    KDL::JntArray  dq_(jnt_positions.size());
    KDL::JntArray  dqdot_(jnt_positions.size());
    KDL::JntArray  qtemp_(jnt_positions.size());
    KDL::JntArray  qdottemp_(jnt_positions.size());
    unsigned int nj = jnt_positions.size();
    m_fd_solver->RK4Integrator(
        nj,
        t,
        dt,
        q_,
        qdot_,
        torques_,
        wr,
        *m_fd_solver.get(),
        qdotdot_,
        dq_,
        dqdot_,
        qtemp_,
        qdottemp_);

    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        jnt_positions[i] += dq_(i);
        jnt_velocities[i] += dqdot_(i);
    }
}

bool ConveyorKDLRobotModel::computeTorques(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        const smpl::RobotState& jnt_accelerations,
        smpl::RobotState& jnt_torques)
{
    KDL::JntArray  q_(jnt_positions.size());
    KDL::JntArray  qdot_(jnt_positions.size());
    KDL::JntArray  torques_(jnt_positions.size());
    KDL::JntArray  qdotdot_(jnt_positions.size());
    KDL::Wrenches wr(11);
    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
        qdot_(i) = jnt_velocities[i];
        qdotdot_(i) = jnt_accelerations[i];
    }

    int ret = m_id_solver->CartToJnt(q_, qdot_, qdotdot_, wr, torques_);
    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        jnt_torques[i] = torques_(i);
        // printf("i %zu acc %.2f torq %.2f\n", i, qdotdot_(i), torques_(i));
    }

    // integration
    // for (size_t i = 0; i < jnt_positions.size(); ++i) {
    //     torques_(i) = 0.0;;
    // }
    // double t = 0.0;
    // double dt = 0.2;
    // KDL::JntArray  dq_(jnt_positions.size());
    // KDL::JntArray  dqdot_(jnt_positions.size());
    // KDL::JntArray  qtemp_(jnt_positions.size());
    // KDL::JntArray  qdottemp_(jnt_positions.size());
    // unsigned int nj = 7;
    // m_fd_solver->RK4Integrator(
    //     nj,
    //     t,
    //     dt,
    //     q_,
    //     qdot_,
    //     torques_,
    //     wr,
    //     *m_fd_solver.get(),
    //     qdotdot_,
    //     dq_,
    //     dqdot_,
    //     qtemp_,
    //     qdottemp_);
    // for (size_t i = 0; i < jnt_positions.size(); ++i) {
    //     printf("%zu | tau: %.2f q: %.2f qdot: %.2f qdotdot: %.2f\n",
    //         i, torques_(i), dq_(i), dqdot_(i), qdotdot_(i));
    // }
    // getchar();
    return true;
}

bool ConveyorKDLRobotModel::computeForwardVelocity(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        std::vector<double>& cart_velocities)
{
    KDL::JntArray  q_(jnt_positions.size());
    KDL::JntArray  qdot_(jnt_positions.size());
    // KDL::Twist     xdot_;
    // jnt_velocities.resize(jnt_positions.size());

    for (size_t i = 0; i < jnt_positions.size(); ++i) {
        q_(i) = jnt_positions[i];
        qdot_(i) = jnt_velocities[i];
    }

    KDL::JntArrayVel qqdot_(q_, qdot_);
    KDL::FrameVel cart_;
    if (!m_jnt_to_cart_vel_solver->JntToCart(qqdot_, cart_) < 0) {
        ROS_WARN("Failed to find inverse joint velocities");
        return false;
    }
    auto xdot_ = cart_.GetTwist();
    cart_velocities.resize(6);
    cart_velocities[0] = xdot_.vel(0);
    cart_velocities[1] = xdot_.vel(1);
    cart_velocities[2] = xdot_.vel(2);
    cart_velocities[3] = xdot_.rot(0);
    cart_velocities[4] = xdot_.rot(1);
    cart_velocities[5] = xdot_.rot(2);
    return true;
}

auto ConveyorKDLRobotModel::getExtension(size_t class_code) -> smpl::Extension*
{
    if (class_code == smpl::GetClassCode<smpl::InverseKinematicsInterface>()
        || class_code == smpl::GetClassCode<InverseVelocityInterface>()
        || class_code == smpl::GetClassCode<ForwardVelocityInterface>()
        || class_code == smpl::GetClassCode<ForwardDynamicsInterface>()
        || class_code == smpl::GetClassCode<InverseDynamicsInterface>()
        || class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>()) return this;

    return URDFRobotModel::getExtension(class_code);
}
