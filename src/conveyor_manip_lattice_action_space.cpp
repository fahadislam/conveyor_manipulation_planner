////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2010, Benjamin Cohen, Andrew Dornbush
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

#include "conveyor_manip_lattice_action_space.h"
#include "conveyor_manip_lattice.h"

// standard includes
#include <limits>
#include <numeric>

// project includes
#include <smpl/angles.h>
#include <smpl/console/console.h>
// #include <smpl/graph/manip_lattice.h>
#include <smpl/heuristic/robot_heuristic.h>

// #include <sbpl_collision_checking/collision_space.h>
// #include <sbpl_collision_checking/shape_visualization.h>
#include <smpl/debug/visualize.h>
// #include <smpl/debug/visualizer_ros.h>

// kdl headers
// #include <kdl/frames.hpp>
#include <kdl/chain.hpp>
// #include <kdl/chainfksolverpos_recursive.hpp>
// #include <kdl/chainiksolverpos_nr_jl.hpp>
// #include <kdl/chainiksolvervel_pinv.hpp>
// #include <kdl/chainjnttojacsolver.hpp>
#include <kdl/jntarray.hpp>
// #include <kdl/tree.hpp>
// #include <kdl_parser/kdl_parser.hpp>

namespace smpl {

bool ConveyorManipLatticeActionSpace::init(ConveyorManipLattice* space)
{
    if (!ActionSpace::init(space)) {
        return false;
    }

    // NOTE: other default thresholds will be set in readParameters, with
    // default values specified in PlanningParams
    m_mprim_thresh[MotionPrimitive::Type::LONG_DISTANCE] =
            std::numeric_limits<double>::infinity();

    clear();

    RobotModel* robot = planningSpace()->robot();

    m_fk_iface = robot->getExtension<ForwardKinematicsInterface>();
    m_ik_iface = robot->getExtension<InverseKinematicsInterface>();
    m_id_iface = robot->getExtension<InverseVelocityInterface>();
    m_ecos = planningSpace()->getExtension<ConveyorObjectStateExtension>();

    if (!m_fk_iface) {
        SMPL_WARN("Conveyor Manip Lattice Action Set requires Forward Kinematics Interface");
    }

    if (!m_ik_iface) {
        SMPL_WARN("Conveyor Manip Lattice Action Set recommends Inverse Kinematics Interface");
    }

    if (!m_id_iface) {
        SMPL_WARN("Conveyor Manip Lattice Action Set recommends Inverse Velocity Interface");
    }

    if (!m_ecos) {
        SMPL_WARN("Conveyor Manip Heuristic recommends ConveyorObjectStateExtension");
    }

    useMultipleIkSolutions(false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
    useAmp(MotionPrimitive::SNAP_TO_RPY, false);
    useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
    useAmp(MotionPrimitive::SHORT_DISTANCE, false);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ, 0.2);
    ampThresh(MotionPrimitive::SNAP_TO_RPY, 0.2);
    ampThresh(MotionPrimitive::SNAP_TO_XYZ_RPY, 0.2);
    ampThresh(MotionPrimitive::SHORT_DISTANCE, 0.2);
    m_amp_thresh = 0.2;

    return true;
}

/// \brief Load motion primitives from file.
///
/// Read in the discrete variable deltas for each motion. If short distance
/// motion primitives are included in the file, they are also enabled.
///
/// Action Set File Format
///
/// Motion_Primitives(degrees): <i actions> <j planning joint variables> <k short distance motion primitives>
/// dv11         dv12        ... dv1m
/// ...
/// dv(i-k)1     dv(i-k)2    ... dv(i-k)m
/// dv(i-k+1)1   dv(i-k+1)2  ... dv(i-k+1)m
/// ...
/// dvi1         dvi2        ... dvim
bool ConveyorManipLatticeActionSpace::load(const std::string& action_filename)
{
    FILE* fCfg = fopen(action_filename.c_str(), "r");
    if (!fCfg) {
        SMPL_ERROR("Failed to open action set file. (file: '%s')", action_filename.c_str());
        return false;
    }

    char sTemp[1024] = { 0 };
    int nrows = 0;
    int ncols = 0;
    int short_mprims = 0;

    // read and check header
    if (fscanf(fCfg, "%1023s", sTemp) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
    }

    if (strcmp(sTemp, "Motion_Primitives(degrees):") != 0) {
        SMPL_ERROR("First line of motion primitive file should be 'Motion_Primitives(degrees):'. Please check your file. (parsed string: %s)\n", sTemp);
        return false;
    }

    // read number of actions
    if (fscanf(fCfg, "%d", &nrows) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read length of joint array
    if (fscanf(fCfg, "%d", &ncols) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    // read number of short distance motion primitives
    if (fscanf(fCfg, "%d", &short_mprims) < 1) {
        SMPL_ERROR("Parsed string has length < 1.");
        return false;
    }

    if (short_mprims == nrows) {
        SMPL_WARN("# of motion prims == # of short distance motion prims. No long distance motion prims set.");
    }

    std::vector<double> mprim(ncols, 0);

    bool have_short_dist_mprims = short_mprims > 0;
    if (have_short_dist_mprims) {
        useAmp(MotionPrimitive::SHORT_DISTANCE, true);
    }

    ConveyorManipLattice* lattice = static_cast<ConveyorManipLattice*>(planningSpace());

    for (int i = 0; i < nrows; ++i) {
        // read joint delta
        for (int j = 0; j < ncols; ++j) {
            double d;
            if (fscanf(fCfg, "%lf", &d) < 1)  {
                SMPL_ERROR("Parsed string has length < 1.");
                return false;
            }
            if (feof(fCfg)) {
                SMPL_ERROR("End of parameter file reached prematurely. Check for newline.");
                return false;
            }
            mprim[j] = d * lattice->resolutions()[j];
            SMPL_DEBUG("Got %0.3f deg -> %0.3f rad", d, mprim[j]);
        }

        if (i < (nrows - short_mprims)) {
            addMotionPrim(mprim, false);
        } else {
            addMotionPrim(mprim, true);
        }
    }

    fclose(fCfg);
    return true;
}

/// \brief Add a long or short distance motion primitive to the action set
/// \param mprim The angle delta for each joint, in radians
/// \param short_dist true = short distance; false = long distance
/// \param add_converse Whether to add the negative of this motion primitive
///     to the action set
void ConveyorManipLatticeActionSpace::addMotionPrim(
    const std::vector<double>& mprim,
    bool short_dist_mprim,
    bool add_converse)
{
    MotionPrimitive m;

    if (short_dist_mprim) {
        m.type = MotionPrimitive::SHORT_DISTANCE;
    } else {
        m.type = MotionPrimitive::LONG_DISTANCE;
    }

    m.action.push_back(mprim);
    m_mprims.push_back(m);

    if (add_converse) {
        for (RobotState& state : m.action) {
            for (size_t i = 0; i < state.size(); ++i) {
                state[i] *= -1.0;
            }
        }
        m_mprims.push_back(m);
    }
}

/// \brief Remove long and short motion primitives and disable adaptive motions.
///
/// Thresholds for short distance and adaptive motions are retained
void ConveyorManipLatticeActionSpace::clear()
{
    m_mprims.clear();

    // add all amps to the motion primitive set
    MotionPrimitive mprim;

    mprim.type = MotionPrimitive::SNAP_TO_RPY;
    mprim.action.clear();
    m_mprims.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ;
    mprim.action.clear();
    m_mprims.push_back(mprim);

    mprim.type = MotionPrimitive::SNAP_TO_XYZ_RPY;
    mprim.action.clear();
    m_mprims.push_back(mprim);

    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
        m_mprim_enabled[i] = (i == MotionPrimitive::Type::LONG_DISTANCE);
    }
}

int ConveyorManipLatticeActionSpace::longDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::LONG_DISTANCE ? 1 : 0);
            });
}

int ConveyorManipLatticeActionSpace::shortDistCount() const
{
    return std::accumulate(
            begin(), end(), 0,
            [](int count, const MotionPrimitive& prim)
            {
                return count +
                        (prim.type == MotionPrimitive::SHORT_DISTANCE ? 1 : 0);
            });
}

bool ConveyorManipLatticeActionSpace::useAmp(MotionPrimitive::Type type) const
{
    return m_mprim_enabled[type];
}

bool ConveyorManipLatticeActionSpace::useMultipleIkSolutions() const
{
    return m_use_multiple_ik_solutions;
}

bool ConveyorManipLatticeActionSpace::useLongAndShortPrims() const
{
    return m_use_long_and_short_dist_mprims;
}

double ConveyorManipLatticeActionSpace::ampThresh(MotionPrimitive::Type type) const
{
    return m_mprim_thresh[type];
}

void ConveyorManipLatticeActionSpace::useAmp(MotionPrimitive::Type type, bool enable)
{
    m_mprim_enabled[type] = enable;
}

void ConveyorManipLatticeActionSpace::useMultipleIkSolutions(bool enable)
{
    m_use_multiple_ik_solutions = enable;
}

void ConveyorManipLatticeActionSpace::useLongAndShortPrims(bool enable)
{
    m_use_long_and_short_dist_mprims = enable;
}

void ConveyorManipLatticeActionSpace::ampThresh(
    MotionPrimitive::Type type,
    double thresh)
{
    if (type != MotionPrimitive::LONG_DISTANCE) {
        m_mprim_thresh[type] = thresh;
    }
}

auto ConveyorManipLatticeActionSpace::getStartGoalDistances(const RobotState& state)
    -> std::pair<double, double>
{
    if (!m_fk_iface) {
        return std::make_pair(0.0, 0.0);
    }

    auto pose = m_fk_iface->computeFK(state);

    if (planningSpace()->numHeuristics() > 0) {
        RobotHeuristic* h = planningSpace()->heuristic(0);
        double start_dist = h->getMetricStartDistance(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2]);
        double goal_dist = h->getMetricGoalDistance(
                pose.translation()[0],
                pose.translation()[1],
                pose.translation()[2]);
        return std::make_pair(start_dist, goal_dist);
    } else {
        return std::make_pair(0.0, 0.0);
    }
}

double EuclideanDistance(
    double x1, double y1, double z1,
    double x2, double y2, double z2)
{
    const double dx = x2 - x1;
    const double dy = y2 - y1;
    const double dz = z2 - z1;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool ConveyorManipLatticeActionSpace::apply(
    const RobotState& parent,
    std::vector<Action>& actions)
{
    RobotState parent_positions(planningSpace()->robot()->jointVariableCount());
    std::copy(parent.begin(), parent.begin() + planningSpace()->robot()->jointVariableCount(), parent_positions.begin());

    double goal_dist, start_dist;
    std::tie(start_dist, goal_dist) = getStartGoalDistances(parent_positions);



    Eigen::Affine3d object_pose;
    Eigen::Vector3d object_velocity;
    std::tie(object_pose, object_velocity) = m_ecos->extractConveyorObjectState(parent.back());

    // intercept distance
    auto pose = m_fk_iface->computeFK(parent_positions);
    double intercept_dist = EuclideanDistance(
            object_pose.translation()[0],
            object_pose.translation()[1],
            object_pose.translation()[2],
            pose.translation()[0],
            pose.translation()[1],
            pose.translation()[2]);

    // adaptive dynamic motion primitive
    if (intercept_dist <= m_amp_thresh) {
        Eigen::VectorXd jnt_positions(parent.size() - 1);
        for (size_t i = 0; i < jnt_positions.size(); ++i) {
            jnt_positions[i] = parent[i];
        }
        Eigen::VectorXd jnt_velocities(7);
        jnt_velocities << 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0;
        RobotState parent_velocities(7,0);
        Action adaptive_action;
        auto tol = planningSpace()->goal().xyz_tolerance;
        // ROS_INFO("Before computing");
        // printf("applying..........................\n");
        if (computeAdaptiveAction(
                parent.back(),
                parent_positions,
                parent_velocities,
                object_pose,
                object_velocity,
                adaptive_action)) {
            actions.push_back(std::move(adaptive_action));
            SMPL_DEBUG("Adaptive primitive added successfully");
            // ROS_INFO("After computing");
            // getchar();
        }
        else {
            SMPL_DEBUG("Adaptive primitive failed");
        }

        // wait prim
        Action action_wait;
        action_wait.resize(1);
        action_wait[0] = parent;
        action_wait[0].back() +=  0.1;
        actions.push_back(std::move(action_wait));
    }

    for (auto& prim : m_mprims) {
        (void)getAction(parent, goal_dist, start_dist, prim, actions);
    }

    if (actions.empty()) {
        SMPL_WARN_ONCE("No motion primitives specified");
    }

    return true;
}

bool ConveyorManipLatticeActionSpace::getAction(
    const RobotState& parent,
    double goal_dist,
    double start_dist,
    const MotionPrimitive& mp,
    std::vector<Action>& actions)
{
    if (!mprimActive(start_dist, goal_dist, mp.type)) {
        return false;
    }

    GoalType goal_type = planningSpace()->goal().type;
    auto& goal_pose = planningSpace()->goal().pose;

    switch (mp.type) {
    case MotionPrimitive::LONG_DISTANCE:  // fall-through
    case MotionPrimitive::SHORT_DISTANCE:
    {
        Action action;
        if (!applyMotionPrimitive(parent, mp, action)) {
            return false;
        }
        actions.push_back(std::move(action));
        return true;
    }
    case MotionPrimitive::SNAP_TO_RPY:
    {
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_XYZ,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ:
    {
        return computeIkAction(
                parent,
                goal_pose,
                goal_dist,
                ik_option::RESTRICT_RPY,
                actions);
    }
    case MotionPrimitive::SNAP_TO_XYZ_RPY:
    {
        if (planningSpace()->goal().type != GoalType::JOINT_STATE_GOAL) {
            return computeIkAction(
                    parent,
                    goal_pose,
                    goal_dist,
                    ik_option::UNRESTRICTED,
                    actions);
        }

        // goal is 7dof; instead of computing IK, use the goal itself as the IK
        // solution
        Action action = { planningSpace()->goal().angles };
        actions.push_back(std::move(action));

        return true;
    }
    default:
        SMPL_ERROR("Motion Primitives of type '%d' are not supported.", mp.type);
        return false;
    }
}

bool ConveyorManipLatticeActionSpace::applyMotionPrimitive(
    const RobotState& state,
    const MotionPrimitive& mp,
    Action& action)
{
    action = mp.action;
    for (size_t i = 0; i < action.size(); ++i) {
        action[i].push_back(0.0);   // time
    }

    ConveyorManipLattice* lattice = static_cast<ConveyorManipLattice*>(planningSpace());
    for (size_t i = 0; i < action.size(); ++i) {
        if (action[i].size() != state.size()) {
            return false;
        }

        for (size_t j = 0; j < mp.action[i].size(); ++j) {
            action[i][j] = action[i][j] + state[j];
        }

        // add time steps respecting velocity limits
        double max_time = 0.0;
        for (size_t j = 0; j < mp.action[i].size(); ++j) {
            auto from_pos = state[j];
            auto to_pos = action[i][j];
            auto vel = planningSpace()->robot()->velLimit(j);
            if (vel <= 0.0) {
                continue;
            }
            auto t = 0.0;
            if (planningSpace()->robot()->isContinuous(j)) {
                t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
            } else {
                t = fabs(to_pos - from_pos) / vel;
            }
            max_time = std::max(max_time, t);
            // max_time *= 0.5;
        }
        action[i][mp.action[i].size()] = state[mp.action[i].size()] + max_time;
    }
    return true;
}

bool ConveyorManipLatticeActionSpace::computeIkAction(
    const RobotState& state,
    const Affine3& goal,
    double dist_to_goal,
    ik_option::IkOption option,
    std::vector<Action>& actions)
{
    SMPL_ERROR("IK action not supported");
    return false;

    if (!m_ik_iface) {
        return false;
    }

    if (m_use_multiple_ik_solutions) {
        //get actions for multiple ik solutions
        std::vector<RobotState> solutions;
        if (!m_ik_iface->computeIK(goal, state, solutions, option)) {
            return false;
        }
        for (auto& solution : solutions) {
            Action action = { std::move(solution) };
            actions.push_back(std::move(action));
        }
    } else {
        //get single action for single ik solution
        RobotState ik_sol;
        if (!m_ik_iface->computeIK(goal, state, ik_sol)) {
            return false;
        }

        Action action = { std::move(ik_sol) };
        actions.push_back(std::move(action));
    }

    return true;
}

// int calls = 0;
// int successes = 0;
// int failures_limits = 0;
// int failures_lift = 0;

bool ConveyorManipLatticeActionSpace::computeAdaptiveAction(
    const double time_start,
    const RobotState& jnt_positions,
    const RobotState& jnt_velocities,
    const Eigen::Affine3d& object_pose,
    const Eigen::Vector3d& object_velocity,
    std::vector<RobotState>& action)
{
    // calls++;
    // printf("calls %d\n", calls);
    // Constants
    double dt = 0.01;
    double gain_p = 1.0;
    double gain_r = 1.0;
    double gripping_time = 0.5;
    int max_iterations = 1000;
    double lift_time = 0.5;
    double lift_offset_y = -0.05;
    double lift_offset_z = 0.05;

    // Variables
    Eigen::Affine3d x_;
    auto q_ = jnt_positions;
    auto q_dot = jnt_velocities;
    std::vector<double> x_dot(6);
    Eigen::Affine3d xo_ = object_pose;
    double intercept_time;

    bool synched = false;
    for(int iter = 0; iter< max_iterations; iter++)
    {
        auto x_ = m_fk_iface->computeFK(q_);
        auto diff = xo_ * x_.inverse(); 
        auto rot = diff.rotation().eulerAngles(2, 1, 0);
        x_dot[0] = gain_p * (xo_.translation().x() - x_.translation().x()) + object_velocity[0];
        x_dot[1] = gain_p * (xo_.translation().y() - x_.translation().y()) + object_velocity[1];
        x_dot[2] = gain_p * (xo_.translation().z() - x_.translation().z()) + object_velocity[2];
        x_dot[3] = gain_r * rot[2];
        x_dot[4] = gain_r * rot[1];
        x_dot[5] = gain_r * rot[0];

        if (!m_id_iface->computeInverseVelocity(q_, x_dot, q_dot)) {
            SMPL_INFO("Failed to compute inverse velocity");
            return false;
        }

        // Add waypoint to action
        auto wp = q_;
        double time_state = time_start + dt * (iter);
        wp.push_back(time_state);
        action.push_back(std::move(wp));

        // Check if within tolerance
        // ROS_INFO("dx: %f dy: %f dz: %f", diff.translation().x(), diff.translation().y(), diff.translation().z());
        // ROS_INFO("dR: %f dP: %f dY: %f", rot(0), rot(1), rot(2));
        // getchar();

        // auto quat = Quaternion(diff.rotation());
        // auto theta = normalize_angle(2.0 * acos(quat.dot(quat)));

        Quaternion qg(xo_.rotation());
        Quaternion q(x_.rotation());
        if (q.dot(qg) < 0.0) {
            qg = Quaternion(-qg.w(), -qg.x(), -qg.y(), -qg.z());
        }

        auto theta = normalize_angle(2.0 * acos(q.dot(qg)));

        double dx = fabs(xo_.translation().x() - x_.translation().x());
        double dy = fabs(xo_.translation().y() - x_.translation().y());
        double dz = fabs(xo_.translation().z() - x_.translation().z());

        if (theta < planningSpace()->goal().rpy_tolerance[0] 
                // std::fabs(rot(0)) <= planningSpace()->goal().rpy_tolerance[0]
                // && std::fabs(rot(1)) <= planningSpace()->goal().rpy_tolerance[1]
                // && std::fabs(rot(2)) <= planningSpace()->goal().rpy_tolerance[2]
                && std::fabs(dx) <= planningSpace()->goal().xyz_tolerance[0]
                && std::fabs(dy) <= planningSpace()->goal().xyz_tolerance[1]
                && std::fabs(dz) <= planningSpace()->goal().xyz_tolerance[2]) {
            if (!synched) {
                intercept_time = time_state;
                synched = true;
                // printf("Intercept time %f\n", intercept_time);
                // double time_state = action.back().back();
                // for (auto ss : q_)
                //     printf(" %f", ss);
                // printf(" %f\n", time_state);

                // printf("    tolerances     %f %f %f\n", planningSpace()->goal().xyz_tolerance[0], planningSpace()->goal().xyz_tolerance[1], planningSpace()->goal().xyz_tolerance[2]);
                // printf("    dif1 position1 %f %f %f\n", diff.translation().x(), diff.translation().y(), diff.translation().z());
                // printf("    dif2 position1 %f %f %f\n", dx, dy, dz);
                // printf("    dif3 theta     %f\n", theta);
                // printf("    goal position1 %f %f %f\n", xo_.translation().x(), xo_.translation().y(), xo_.translation().z());
                // printf("    grip position1 %f %f %f\n", x_.translation().x(), x_.translation().y(), x_.translation().z());
                // m_intercept_time = intercept_time;
                // getchar();

            }
            if (time_state - intercept_time >= gripping_time) {
                // return true;
                // Add lift motion
                // x_.translation().y() += lift_offset_y;
                x_.translation().z() += lift_offset_z;
                if (!m_ik_iface->computeIK(x_, q_, q_)) {
                    SMPL_DEBUG("Failed to lift up");
                    // failures_lift++;
                    // printf("failures_lift %d\n", failures_lift);
                    return false;
                }
                wp = q_;
                double time_state = action.back().back() + lift_time;
                wp.push_back(time_state);

                action.push_back(std::move(wp));
                // SMPL_INFO("Added adaptive primitive");
                // successes++;
                // printf("successes %d\n", successes);

                return true;
            }
        }

        // Move arm joints
        for(size_t i = 0; i < q_.size(); ++i) {
            q_[i] += (q_dot[i] * dt);
        }

        // TODO: check velocity limit

        // Check joint limits
        if (!planningSpace()->robot()->checkJointLimits(q_)) {
            // SMPL_INFO("Violates joint limits");
            // failures_limits++;
            // printf("failures_limits %d\n", failures_limits);
            return false;
        }

        // Move object
        xo_.translation().x() += object_velocity[0] * dt;
        xo_.translation().y() += object_velocity[1] * dt;
        xo_.translation().z() += object_velocity[2] * dt;    

    }

    // SMPL_INFO("Failed to reach object");
    return false;
}

bool ConveyorManipLatticeActionSpace::mprimActive(
    double start_dist,
    double goal_dist,
    MotionPrimitive::Type type) const
{
    // TODO: this seems a awkward..."short distance" motion primitives should be
    // the default since "long distance" primitives are usually implemented as
    // an optimization in regions far from the start or goal, and often we
    // always need "short distance" motion primitives near the start and goal.
    // -Andrew
    if (type == MotionPrimitive::LONG_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return true;
        }
        const bool near_goal =
                goal_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_start =
                start_dist <= m_mprim_thresh[MotionPrimitive::SHORT_DISTANCE];
        const bool near_endpoint = near_goal || near_start;
        return !(m_mprim_enabled[MotionPrimitive::SHORT_DISTANCE] && near_endpoint);
    } else if (type == MotionPrimitive::SHORT_DISTANCE) {
        if (m_use_long_and_short_dist_mprims) {
            return m_mprim_enabled[type];
        }
        const bool near_goal = goal_dist <= m_mprim_thresh[type];
        const bool near_start = start_dist <= m_mprim_thresh[type];
        const bool near_endpoint = near_goal || near_start;
        return m_mprim_enabled[type] && near_endpoint;
    } else {
        return m_mprim_enabled[type] && goal_dist <= m_mprim_thresh[type];
    }
}

} // namespace smpl
