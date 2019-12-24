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

// #include <smpl/graph/manip_lattice.h>
#include "conveyor_object_lattice.h"

// standard includes
#include <iomanip>
#include <sstream>

// system includes
#include <sbpl/planners/planner.h>

#include <smpl/angles.h>
#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/heuristic/robot_heuristic.h>
#include <smpl/debug/visualize.h>
#include <smpl/debug/marker_utils.h>
#include <smpl/spatial.h>
// #include "../profiling.h"

// #include <boost/archive/binary_iarchive.hpp>
// #include <boost/archive/binary_oarchive.hpp>
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iostream>


namespace smpl {

ConveyorObjectLattice::~ConveyorObjectLattice()
{
}

void ConveyorObjectLattice::GetSuccs(
    int state_id,
    std::vector<int>* succs,
    std::vector<int>* costs)
{
    assert(state_id >= 0 && state_id < m_states.size() && "state id out of bounds");
    assert(succs && costs && "successor buffer is null");
    assert(m_actions && "action space is uninitialized");

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "expanding state %d", state_id);

    // goal state should be absorbing
    // if (state_id == m_goal_state_id) {
    //     return;
    // }

    ManipLatticeState* parent_entry = getHashEntry(state_id);//m_states[state_id];

    assert(parent_entry);
    assert(parent_entry->coord.size() >= robot()->jointVariableCount());

    // log expanded state details
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  coord: " << parent_entry->coord);
    SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  angles: " << parent_entry->state);

    auto* vis_name = "expansion";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(parent_entry->state, vis_name));

    int goal_succ_count = 0;

    std::vector<Action> actions;
    if (!actionSpace()->apply(parent_entry->state, actions)) {
        SMPL_WARN("Failed to get actions");
        return;
    }

    SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "  actions: %zu", actions.size());

    // check actions for validity
    RobotCoord succ_coord(robot()->jointVariableCount(), 0);
    for (size_t i = 0; i < actions.size(); ++i) {
        auto& action = actions[i];

        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "    action %zu:", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      waypoints: %zu", action.size());

        if (!checkAction(parent_entry->state, action)) {
            continue;
        }

        // compute destination coords
        stateToCoord(action.back(), succ_coord);

        // get the successor

        // check if hash entry already exists, if not then create one
        int succ_state_id = getOrCreateState(succ_coord, action.back());
        ManipLatticeState* succ_entry = getHashEntry(succ_state_id);

        // check if this state meets the goal criteria
        // auto is_goal_succ = isGoal(action.back());
        // if (is_goal_succ) {
        //     // update goal state
        //     ++goal_succ_count;
        // }

        // put successor on successor list with the proper cost
        // if (is_goal_succ) {
        //     succs->push_back(m_goal_state_id);
        // } else {
            succs->push_back(succ_state_id);
        // }
        costs->push_back(cost(parent_entry, succ_entry, false));

        // log successor details
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "      succ: %zu", i);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        id: %5i", succ_state_id);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << succ_coord);
        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "        state: " << succ_entry->state);
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "        cost: %5d", cost(parent_entry, succ_entry, false));
    }

    if (goal_succ_count > 0) {
        SMPL_DEBUG_NAMED(G_EXPANSIONS_LOG, "Got %d goal successors!", goal_succ_count);
    }
}


void ConveyorObjectLattice::setPathId(int state_id, int path_id)
{
    auto entry = getHashEntry(state_id);
    m_state_to_pid[entry->coord] = path_id;

    SMPL_INFO_NAMED(G_LOG, "  state id: %d,     path id: %d", state_id, path_id);
}

int ConveyorObjectLattice::getPathId(RobotState state)
{
    RobotCoord coord(robot()->jointVariableCount());
    stateToCoord(state, coord);

    SMPL_INFO_STREAM_NAMED(G_EXPANSIONS_LOG, "        coord: " << coord);


    auto it = m_state_to_pid.find(coord);
    if (it != m_state_to_pid.end()) {
        return it->second;
    }
    return -1;
}

bool ConveyorObjectLattice::saveStateToPathIdMap()
{
    std::ofstream ofs("/home/fislam/paths/state_to_pathid_map");
    boost::archive::text_oarchive oarch(ofs);
    oarch << m_state_to_pid;

    ofs.close();
    return true;
}

bool ConveyorObjectLattice::loadStateToPathIdMap()
{
    std::ifstream ifs("/home/fislam/paths/state_to_pathid_map");
    if(!ifs.is_open()) {
        return false;
    }
    boost::archive::text_iarchive iarch(ifs);
    iarch >>  m_state_to_pid;
    return true;
}


} // namespace smpl
