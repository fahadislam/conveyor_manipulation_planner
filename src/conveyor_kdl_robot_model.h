#include <sbpl_kdl_robot_model/kdl_robot_model.h>

#ifndef CONVEYOR_KDL_ROBOT_MODEL_H
#define CONVEYOR_KDL_ROBOT_MODEL_H

class InverseVelocityInterface : public virtual smpl::RobotModel
{
public:

    /// \brief Return the number of redundant joint variables.
    virtual bool computeInverseVelocity(
        const smpl::RobotState& jnt_positions,
        const std::vector<double>& cart_velocities,
        smpl::RobotState& jnt_velocities) = 0;
};

class ForwardDynamicsInterface : public virtual smpl::RobotModel
{
public:

    /// \brief Return the number of redundant joint variables.
    virtual bool computeAcceleration(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        const smpl::RobotState& jnt_torques,
        smpl::RobotState& jnt_accelerations) = 0;
};

class ForwardVelocityInterface : public virtual smpl::RobotModel
{
public:

    /// \brief Return the number of redundant joint variables.
    virtual bool computeForwardVelocity(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        std::vector<double>& cart_velocities) = 0;
};

class ConveyorKDLRobotModel : public smpl::KDLRobotModel, public virtual InverseVelocityInterface, ForwardVelocityInterface, ForwardDynamicsInterface
{
public:

    // ConveyorKDLRobotModel();

    // ~ConveyorKDLRobotModel();

    bool init(
        const std::string& robot_description,
        const std::string& base_link,
        const std::string& tip_link,
        int free_angle = DEFAULT_FREE_ANGLE_INDEX);

    bool computeInverseVelocity(
        const smpl::RobotState& jnt_positions,
        const std::vector<double>& cart_velocities,
        smpl::RobotState& jnt_velocities);

    bool computeAcceleration(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        const smpl::RobotState& jnt_torques,
        smpl::RobotState& jnt_accelerations);

    bool computeForwardVelocity(
        const smpl::RobotState& jnt_positions,
        const smpl::RobotState& jnt_velocities,
        std::vector<double>& cart_velocities);

    auto getExtension(size_t class_code) -> smpl::Extension* override;

private:


	std::unique_ptr<KDL::ChainIkSolverVel_pinv>         m_cart_to_jnt_vel_solver;
    std::unique_ptr<KDL::ChainFkSolverVel_recursive>    m_jnt_to_cart_vel_solver;
    std::unique_ptr<KDL::ChainFdSolver_RNE>             m_fd_solver;

};

#endif