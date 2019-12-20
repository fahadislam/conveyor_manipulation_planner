////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2008, Benjamin Cohen, Andrew Dornbush
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
/// \author Andrew Dornbush

#ifndef SMPL_CONVEYOR_MANIP_LATTICE_H
#define SMPL_CONVEYOR_MANIP_LATTICE_H

// standard includes
#include <time.h>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

// system includes
#include <boost/functional/hash.hpp>

// project includes
#include <smpl/angles.h>
#include <smpl/time.h>
#include <smpl/collision_checker.h>
#include <smpl/occupancy_grid.h>
#include <smpl/planning_params.h>
#include <smpl/robot_model.h>
#include <smpl/types.h>
#include <smpl/graph/robot_planning_space.h>
#include <smpl/graph/action_space.h>

namespace smpl {

class RobotHeuristic;

typedef std::vector<int> RobotCoord;

struct ConveyorManipLatticeState
{
    RobotCoord coord;   // discrete coordinate
    RobotState state;   // corresponding continuous coordinate
};

inline
bool operator==(const ConveyorManipLatticeState& a, const ConveyorManipLatticeState& b)
{
    return a.coord == b.coord;
}

} // namespace smpl

namespace std {

template <>
struct hash<smpl::ConveyorManipLatticeState>
{
    typedef smpl::ConveyorManipLatticeState argument_type;
    typedef std::size_t result_type;
    result_type operator()(const argument_type& s) const;
};

} // namespace std

namespace smpl {

class ConveyorObjectStateExtension : public virtual Extension
{
public:

    virtual ~ConveyorObjectStateExtension() { }

    virtual const auto extractConveyorObjectState(double t) -> std::pair<Eigen::Affine3d, Eigen::Vector3d> = 0;
};

/// \class Discrete space constructed by expliciting discretizing each joint
class ConveyorManipLattice :
    public RobotPlanningSpace,
    public PoseProjectionExtension,
    public ExtractRobotStateExtension,
    public ConveyorObjectStateExtension
{
public:

    ~ConveyorManipLattice();

    bool init(
        RobotModel* robot,
        CollisionChecker* checker,
        const std::vector<double>& resolutions,
        ActionSpace* actions);

    auto resolutions() const -> const std::vector<double>& { return m_coord_deltas; }
    auto actionSpace() -> ActionSpace* { return m_actions; }
    auto actionSpace() const -> const ActionSpace* { return m_actions; }

    auto getStartConfiguration() const -> RobotState;

    void setVisualizationFrameId(const std::string& frame_id);
    auto visualizationFrameId() const -> const std::string&;

    auto getDiscreteCenter(const RobotState& state) const -> RobotState;

    void clearStates();

    /// \name Reimplemented Public Functions from RobotPlanningSpace
    ///@{
    void GetLazySuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs,
        std::vector<bool>* true_costs) override;
    int GetTrueCost(int parent_id, int child_id) override;
    ///@}

    void setObjectVelocity(Eigen::Vector3d velocity);

    const auto extractConveyorObjectState(double t)
        -> std::pair<Eigen::Affine3d, Eigen::Vector3d> override;
    /// \name Required Public Functions from ExtractRobotStateExtension
    ///@{
    auto extractState(int state_id) -> const RobotState& override;
    ///@}

    /// \name Required Public Functions from PoseProjectionExtension
    ///@{
    bool projectToPose(int state_id, Affine3& pos) override;
    ///@}

    /// \name Required Public Functions from RobotPlanningSpace
    ///@{
    bool setStart(const RobotState& state) override;
    bool setGoal(const GoalConstraint& goal) override;
    int getStartStateID() const override;
    int getGoalStateID() const override;
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    double getInterceptTime(const std::vector<RobotState>& path);
    ///@}

    /// \name Required Public Functions from Extension
    ///@{
    virtual Extension* getExtension(size_t class_code) override;
    ///@}

    /// \name Required Public Functions from DiscreteSpaceInformation
    ///@{
    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;
    void PrintState(int state_id, bool verbose, FILE* fout = nullptr) override;
    void GetPreds(
        int state_id,
        std::vector<int>* preds,
        std::vector<int>* costs) override;
    ///@}

protected:

    /// \name discretization methods
    ///@{
    void coordToState(const RobotCoord& coord, RobotState& state) const;
    void stateToCoord(const RobotState& state, RobotCoord& coord) const;
    ///@}

    ConveyorManipLatticeState* getHashEntry(int state_id) const;

    int getHashEntry(const RobotCoord& coord);
    int createHashEntry(const RobotCoord& coord, const RobotState& state);
    int getOrCreateState(const RobotCoord& coord, const RobotState& state);
    int reserveHashEntry();

    Affine3 computePlanningFrameFK(const RobotState& state) const;

    int cost(
        ConveyorManipLatticeState* HashEntry1,
        ConveyorManipLatticeState* HashEntry2,
        bool bState2IsGoal) const;

    bool checkAction(const RobotState& state, const Action& action);

    bool isGoal(const RobotState& state);

    auto getStateVisualization(const RobotState& vars, const std::string& ns)
        -> std::vector<visual::Marker>;

    std::vector<ConveyorManipLatticeState*> m_states;
    int m_goal_state_id = -1;
private:

    ForwardKinematicsInterface* m_fk_iface = nullptr;
    ActionSpace* m_actions = nullptr;

    // cached from robot model
    std::vector<double> m_min_limits;
    std::vector<double> m_max_limits;
    std::vector<bool> m_continuous;
    std::vector<bool> m_bounded;

    std::vector<int> m_coord_vals;
    std::vector<double> m_coord_deltas;

    int m_start_state_id = -1;

    // maps from coords to stateID
    typedef ConveyorManipLatticeState StateKey;
    typedef PointerValueHash<StateKey> StateHash;
    typedef PointerValueEqual<StateKey> StateEqual;
    hash_map<StateKey*, int, StateHash, StateEqual> m_state_to_id;

    // maps from stateID to coords

    std::string m_viz_frame_id;

    // conveyor object velocity
    Eigen::Vector3d m_object_velocity;

    bool setGoalPose(const GoalConstraint& goal);
    bool setGoalPoses(const GoalConstraint& goal);
    bool setGoalConfiguration(const GoalConstraint& goal);
    bool setUserGoal(const GoalConstraint& goal);

    void startNewSearch();

    /// \name planning
    ///@{
    ///@}
};

} // namespace smpl

#endif
