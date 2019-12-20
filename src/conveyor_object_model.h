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

#ifndef SMPL_CONVEYOR_OBJECT_MODEL_H
#define SMPL_CONVEYOR_OBJECT_MODEL_H

#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/graph/manip_lattice.h>
#include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heuristic/joint_dist_heuristic.h>

/// \brief Defines a Kinematic Model for an (x, y, theta) conveyor object
///
/// RobotModel base: basic requirements (variable types and limits)
///
/// ForwardKinematicsInterface: forward kinematics interface required by much
/// of smpl; trivial in this case to establish frame of reference
class ConveyorObjectModel :
//    public virtual smpl::RobotModel,
    public smpl::ForwardKinematicsInterface
{
public:

    ConveyorObjectModel() : smpl::RobotModel(), smpl::ForwardKinematicsInterface()
    {
        const std::vector<std::string> joint_names = { "x", "y", "theta" };
        setPlanningJoints(joint_names);
    }

    /// \name Required Public Functions from ForwardKinematicsInterface
    ///@{
    Eigen::Affine3d computeFK(const smpl::RobotState& state) override
    {
        return Eigen::Affine3d(Eigen::Translation3d(state[0], state[1], state[2]));
    }
    ///@}

    /// \name Required Public Functions from Robot Model
    ///@{
    double minPosLimit(int jidx) const override { return 0.0; }
    double maxPosLimit(int jidx) const override { return 0.0; }
    bool hasPosLimit(int jidx) const override { return false; }
    bool isContinuous(int jidx) const override {return (jidx == 2) ? true : false;};
    double velLimit(int jidx) const override { return 0.0; }
    double accLimit(int jidx) const override { return 0.0; }

    bool checkJointLimits(
        const smpl::RobotState& angles,
        bool verbose = false) override
    {
        return true;
    }
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override
    {
        if (class_code == smpl::GetClassCode<smpl::RobotModel>() ||
            class_code == smpl::GetClassCode<smpl::ForwardKinematicsInterface>())
        {
            return this;
        } else {
            return nullptr;
        }
    }
    ///@}
};

/// \brief Defines a collision checker for an (x,y) point robot in a grid world.
class ConveyorBoundsChecker : public smpl::CollisionChecker
{
public:

    ConveyorBoundsChecker(smpl::OccupancyGrid* grid) :
        Extension(), m_grid(grid)
    { }

    /// \name Required Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override
    {
        if (class_code == smpl::GetClassCode<smpl::CollisionChecker>()) {
            return this;
        }
        return nullptr;
    }
    ///@}

    /// \name Required Functions from CollisionChecker
    ///@{
    bool isStateValid(const smpl::RobotState& state, bool verbose) override;

    bool isStateToStateValid(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        bool verbose) override;

    bool interpolatePath(
        const smpl::RobotState& start,
        const smpl::RobotState& finish,
        std::vector<smpl::RobotState>& path) override;
    ///@}

private:

    // a bit heavy-weight for this, since it overlays a distance transform
    smpl::OccupancyGrid* m_grid;

    double conveyorZ;
};

#endif