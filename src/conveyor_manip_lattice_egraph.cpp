////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2016, Andrew Dornbush
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

#include "conveyor_manip_lattice_egraph.h"
#include "conveyor_manip_lattice_action_space.h"

#include <fstream>

#include <boost/filesystem.hpp>

#include <smpl/console/console.h>
#include <smpl/console/nonstd.h>
#include <smpl/csv_parser.h>
#include <smpl/debug/visualize.h>
// #include <smpl/graph/manip_lattice_action_space.h>
#include <smpl/heap/intrusive_heap.h>

namespace smpl {

auto ConveyorManipLatticeEgraph::RobotCoordHash::operator()(const argument_type& s) const ->
    result_type
{
    std::size_t seed = 0;
    boost::hash_combine(seed, boost::hash_range(s.begin(), s.end()));
    return seed;
}

bool ConveyorManipLatticeEgraph::extractPath(
    const std::vector<int>& idpath,
    std::vector<RobotState>& path)
{
    SMPL_DEBUG_STREAM_NAMED(G_LOG, "State ID Path: " << idpath);
    if (idpath.empty()) {
        return true;
    }

    // attempt to handle paths of length 1...do any of the sbpl planners still
    // return a single-point path in some cases?
    if (idpath.size() == 1) {
        auto state_id = idpath[0];

        if (state_id == getGoalStateID()) {
            auto* entry = getHashEntry(getStartStateID());
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", getStartStateID());
                return false;
            }
            path.push_back(entry->state);
        } else {
            auto* entry = getHashEntry(state_id);
            if (!entry) {
                SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", state_id);
                return false;
            }
            path.push_back(entry->state);
        }

        auto* vis_name = "goal_config";
        SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
        return true;
    }

    if (idpath[0] == getGoalStateID()) {
        SMPL_ERROR_NAMED(G_LOG, "Cannot extract a non-trivial path starting from the goal state");
        return false;
    }

    std::vector<RobotState> opath;

    // grab the first point
    {
        auto* entry = getHashEntry(idpath[0]);
        if (!entry) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get state entry for state %d", idpath[0]);
            return false;
        }
        opath.push_back(entry->state);
    }

    // grab the rest of the points
    for (size_t i = 1; i < idpath.size(); ++i) {
        auto prev_id = idpath[i - 1];
        auto curr_id = idpath[i];
        SMPL_DEBUG_NAMED(G_LOG, "Extract motion from state %d to state %d", prev_id, curr_id);

        if (prev_id == getGoalStateID()) {
            SMPL_ERROR_NAMED(G_LOG, "Cannot determine goal state predecessor state during path extraction");
            return false;
        }

        // find the successor state corresponding to the cheapest valid action

        ConveyorManipLatticeState* prev_entry = getHashEntry(prev_id);
        const RobotState& prev_state = prev_entry->state;

        std::vector<Action> actions;
        if (!actionSpace()->apply(prev_state, actions)) {
            SMPL_ERROR_NAMED(G_LOG, "Failed to get actions while extracting the path");
            return false;
        }

        SMPL_DEBUG_NAMED(G_LOG, "Check for transition via normal successors");
        ConveyorManipLatticeState* best_state = nullptr;
        RobotCoord succ_coord(robot()->jointVariableCount() + 1);
        int best_cost = std::numeric_limits<int>::max();

        for (const Action& action : actions) {
            // check the validity of this transition
            if (!checkAction(prev_state, action)) {
                continue;
            }

            if (curr_id == getGoalStateID()) {
                SMPL_DEBUG_NAMED(G_LOG, "Search for transition to goal state");

                // skip non-goal states
                if (!isGoal(action.back()) && !(action.size() > 1)) {
                    continue;
                }
                else {
                    for (size_t i = 0; i < action.size() - 1; ++i) {
                        SMPL_DEBUG_STREAM_NAMED(G_EXPANSIONS_LOG, "  ADAPT STATE: " << action[i]);
                        opath.push_back(action[i]);
                    }
                }

                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ConveyorManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);

                SMPL_INFO_STREAM_NAMED(G_EXPANSIONS_LOG, "  GOAL COORD: " << succ_coord);

                const int edge_cost = cost(prev_entry, succ_entry, true);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_state = succ_entry;
                    if (best_state == nullptr) {
                    }
                }
            } else {
                stateToCoord(action.back(), succ_coord);
                int succ_state_id = getHashEntry(succ_coord);
                ConveyorManipLatticeState* succ_entry = getHashEntry(succ_state_id);
                assert(succ_entry);
                if (succ_state_id != curr_id) {
                    continue;
                }

                const int edge_cost = cost(prev_entry, succ_entry, false);
                if (edge_cost < best_cost) {
                    best_cost = edge_cost;
                    best_state = succ_entry;
                }
            }
        }

        if (best_state) {
            SMPL_DEBUG_STREAM_NAMED(G_LOG, "Extract successor state " << best_state->state);
            opath.push_back(best_state->state);
            continue;
        }
        else {
        }

        bool found = false;
        // check for shortcut transition
        auto pnit = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), prev_id);
        auto cnit = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), curr_id);
        if (pnit != m_egraph_state_ids.end() &&
            cnit != m_egraph_state_ids.end())
        {
            ExperienceGraph::node_id pn =
                    std::distance(m_egraph_state_ids.begin(), pnit);
            ExperienceGraph::node_id cn =
                    std::distance(m_egraph_state_ids.begin(), cnit);

            SMPL_INFO("Check for shortcut from %d to %d (egraph %zu -> %zu)!", prev_id, curr_id, pn, cn);

            std::vector<ExperienceGraph::node_id> node_path;
            found = findShortestExperienceGraphPath(pn, cn, node_path);
            if (found) {
                for (ExperienceGraph::node_id n : node_path) {
                    int state_id = m_egraph_state_ids[n];
                    ConveyorManipLatticeState* entry = getHashEntry(state_id);
                    assert(entry);
                    opath.push_back(entry->state);
                }
            }
        }
        if (found) {
            continue;
        }

        // check for snap transition
        SMPL_DEBUG_NAMED(G_LOG, "Check for snap successor");
        int cost;
        if (snap(prev_id, curr_id, cost)) {
            SMPL_ERROR("Snap from %d to %d with cost %d", prev_id, curr_id, cost);
            ConveyorManipLatticeState* entry = getHashEntry(curr_id);
            assert(entry);
            opath.push_back(entry->state);
            continue;
        }

        SMPL_ERROR_NAMED(G_LOG, "Failed to find valid goal successor during path extraction");
        return false;
    }

    // we made it!
    path = std::move(opath);
    auto* vis_name = "goal_config";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(path.back(), vis_name));
    return true;
}

bool ConveyorManipLatticeEgraph::loadExperienceGraph(const std::vector<std::string>& paths)
{
    SMPL_INFO("Load Experience Graph at %s", paths[0].c_str());


    for (const auto& path : paths) {
        boost::filesystem::path p(path);
        if (!boost::filesystem::is_directory(p)) {
            SMPL_ERROR("'%s' is not a directory", paths[0].c_str());
            return false;
        }

        for (auto dit = boost::filesystem::directory_iterator(p);
            dit != boost::filesystem::directory_iterator(); ++dit)
        {
            auto& filepath = dit->path().generic_string();
            std::vector<RobotState> egraph_states;
            if (!parseExperienceGraphFile(filepath, egraph_states)) {
                continue;
            }

            if (egraph_states.empty()) {
                continue;
            }

            SMPL_INFO("Create hash entries for experience graph states");

            auto& pp = egraph_states.front();  // previous robot state
            RobotCoord pdp(robot()->jointVariableCount() + 1); // previous robot coord
            stateToCoord(egraph_states.front(), pdp);

            auto pid = m_egraph.insert_node(pp);
            m_coord_to_nodes[pdp].push_back(pid);

            // --
            // int entry_id = reserveHashEntry();
            // auto* entry = getHashEntry(entry_id);
            // entry->coord = pdp;
            // entry->state = pp;
            // ++
            int entry_id = getOrCreateState(pdp, pp);
            // printf("front egraph node id is %d\n", entry_id);

            // map state id <-> experience graph state
            m_egraph_state_ids.resize(pid + 1, -1);
            m_egraph_state_ids[pid] = entry_id;
            m_state_to_node[entry_id] = pid;

            std::vector<RobotState> edge_data;
            for (size_t i = 1; i < egraph_states.size(); ++i) {
                auto& p = egraph_states[i];
                RobotCoord dp(robot()->jointVariableCount() + 1);
                stateToCoord(p, dp);
                if (dp != pdp) {
                    // found a new discrete state along the path

                    auto id = m_egraph.insert_node(p);
                    m_coord_to_nodes[dp].push_back(id);

                    // --
                    // int entry_id = reserveHashEntry();
                    // printf("    egraph node id is %d\n", entry_id);
                    // auto* entry = getHashEntry(entry_id);
                    // entry->coord = dp;
                    // entry->state = p;
                    // ++
                    int entry_id = getOrCreateState(dp, p);

                    m_egraph_state_ids.resize(id + 1, -1);
                    m_egraph_state_ids[id] = entry_id;
                    m_state_to_node[entry_id] = id;
                    m_egraph.insert_edge(pid, id, edge_data);

                    pdp = dp;
                    pid = id;
                    edge_data.clear();
                } else {
                    // gather intermediate robot states
                    edge_data.push_back(p);
                }
            }
        }        
    }


    SMPL_INFO("Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

bool ConveyorManipLatticeEgraph::loadExperienceGraph(const std::string& path)
{
    SMPL_INFO("Load Experience Graph at %s", path.c_str());

    boost::filesystem::path p(path);
    if (!boost::filesystem::is_directory(p)) {
        SMPL_ERROR("'%s' is not a directory", path.c_str());
        return false;
    }

    for (auto dit = boost::filesystem::directory_iterator(p);
        dit != boost::filesystem::directory_iterator(); ++dit)
    {
        auto& filepath = dit->path().generic_string();
        std::vector<RobotState> egraph_states;
        if (!parseExperienceGraphFile(filepath, egraph_states)) {
            continue;
        }

        if (egraph_states.empty()) {
            continue;
        }

        SMPL_INFO("Create hash entries for experience graph states");

        auto& pp = egraph_states.front();  // previous robot state
        RobotCoord pdp(robot()->jointVariableCount() + 1); // previous robot coord
        stateToCoord(egraph_states.front(), pdp);

        auto pid = m_egraph.insert_node(pp);
        m_coord_to_nodes[pdp].push_back(pid);

        // --
        // int entry_id = reserveHashEntry();
        // auto* entry = getHashEntry(entry_id);
        // entry->coord = pdp;
        // entry->state = pp;
        // ++
        int entry_id = getOrCreateState(pdp, pp);
        // printf("front egraph node id is %d\n", entry_id);

        // map state id <-> experience graph state
        m_egraph_state_ids.resize(pid + 1, -1);
        m_egraph_state_ids[pid] = entry_id;
        m_state_to_node[entry_id] = pid;

        std::vector<RobotState> edge_data;
        for (size_t i = 1; i < egraph_states.size(); ++i) {
            auto& p = egraph_states[i];
            RobotCoord dp(robot()->jointVariableCount() + 1);
            stateToCoord(p, dp);
            if (dp != pdp) {
                // found a new discrete state along the path

                auto id = m_egraph.insert_node(p);
                m_coord_to_nodes[dp].push_back(id);

                // --
                // int entry_id = reserveHashEntry();
                // printf("    egraph node id is %d\n", entry_id);
                // auto* entry = getHashEntry(entry_id);
                // entry->coord = dp;
                // entry->state = p;
                // ++
                int entry_id = getOrCreateState(dp, p);

                m_egraph_state_ids.resize(id + 1, -1);
                m_egraph_state_ids[id] = entry_id;
                m_state_to_node[entry_id] = id;
                m_egraph.insert_edge(pid, id, edge_data);

                pdp = dp;
                pid = id;
                edge_data.clear();
            } else {
                // gather intermediate robot states
                edge_data.push_back(p);
            }
        }
    }

    SMPL_INFO("Experience graph contains %zu nodes and %zu edges", m_egraph.num_nodes(), m_egraph.num_edges());
    return true;
}

void ConveyorManipLatticeEgraph::getExperienceGraphNodes(
    int state_id,
    std::vector<ExperienceGraph::node_id>& nodes)
{
    auto it = m_state_to_node.find(state_id);
    if (it != m_state_to_node.end()) {
        nodes.push_back(it->second);
    }
}

bool ConveyorManipLatticeEgraph::shortcut(
    int first_id,
    int second_id,
    int& cost)
{
    ConveyorManipLatticeState* first_entry = getHashEntry(first_id);
    ConveyorManipLatticeState* second_entry = getHashEntry(second_id);
    if (!first_entry | !second_entry) {
        SMPL_WARN("No state entries for state %d or state %d", first_id, second_id);
        return false;
    }

    // check if can snap in time
    double max_time = 0.0;
    for (size_t j = 0; j < robot()->jointVariableCount(); ++j) {
        auto from_pos = first_entry->state[j];
        auto to_pos = second_entry->state[j];
        auto vel = robot()->velLimit(j);
        if (vel <= 0.0) {
            continue;
        }
        auto t = 0.0;
        if (robot()->isContinuous(j)) {
            t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
        } else {
            t = fabs(to_pos - from_pos) / vel;
        }
        max_time = std::max(max_time, t);
    }

    SMPL_INFO_STREAM("Shortcut " << first_entry->state << " -> " << second_entry->state);

    double time_diff = second_entry->state.back() - first_entry->state.back();
    if (time_diff < max_time) {
        SMPL_WARN("Failed shortcut time-wise!");
        return false;
    }
    SMPL_INFO("  Shortcut %d -> %d!", first_id, second_id);

    auto* vis_name = "shortcut";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(first_entry->state, "shortcut_from"));
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(second_entry->state, "shortcut_to"));

    cost = 1000 * max_time;
    return true;
}

bool ConveyorManipLatticeEgraph::snap(
    int first_id,
    int second_id,
    int& cost)
{
    auto* first_entry = getHashEntry(first_id);
    auto* second_entry = getHashEntry(second_id);

    RobotState first_state_positions(robot()->jointVariableCount());
    std::copy(first_entry->state.begin(), first_entry->state.begin() + robot()->jointVariableCount(), first_state_positions.begin());

    RobotState second_state_positions(robot()->jointVariableCount());
    std::copy(second_entry->state.begin(), second_entry->state.begin() + robot()->jointVariableCount(), second_state_positions.begin());

    if (!first_entry | !second_entry) {
        SMPL_WARN("No state entries for state %d or state %d", first_id, second_id);
        return false;
    }

    SMPL_DEBUG_STREAM("Snap " << first_entry->state << " -> " << second_entry->state);
    auto* vis_name = "snap";
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(first_state_positions, "snap_from"));
    SV_SHOW_INFO_NAMED(vis_name, getStateVisualization(second_state_positions, "snap_to"));

    // check if can snap in time
    double max_time = 0.0;
    for (size_t j = 0; j < robot()->jointVariableCount(); ++j) {
        auto from_pos = first_entry->state[j];
        auto to_pos = second_entry->state[j];
        auto vel = robot()->velLimit(j);
        if (vel <= 0.0) {
            continue;
        }
        auto t = 0.0;
        if (robot()->isContinuous(j)) {
            t = angles::shortest_angle_dist(from_pos, to_pos) / vel;
        } else {
            t = fabs(to_pos - from_pos) / vel;
        }
        max_time = std::max(max_time, t);
    }

    double time_diff = second_entry->state.back() - first_entry->state.back();
    if (time_diff < max_time) {
        SMPL_WARN("Failed snap time-wise!");
        return false;
    }


    if (!collisionChecker()->isStateToStateValid(
            first_entry->state, second_entry->state))
    {
        SMPL_WARN("Failed snap!");
        return false;
    }

    SMPL_INFO("  Snap %d -> %d!", first_id, second_id);
    cost = 1000 * max_time;
    return true;
}

const ExperienceGraph* ConveyorManipLatticeEgraph::getExperienceGraph() const
{
    return &m_egraph;
}

ExperienceGraph* ConveyorManipLatticeEgraph::getExperienceGraph()
{
    return &m_egraph;
}

int ConveyorManipLatticeEgraph::getStateID(ExperienceGraph::node_id n) const
{
    if (n >= m_egraph_state_ids.size()) {
        return -1;
    } else {
        return m_egraph_state_ids[n];
    }
}

Extension* ConveyorManipLatticeEgraph::getExtension(size_t class_code)
{
    if (class_code == GetClassCode<ExperienceGraphExtension>()) {
        return this;
    } else {
        return ConveyorManipLattice::getExtension(class_code);
    }

    if (class_code == GetClassCode<RobotPlanningSpace>() ||
        class_code == GetClassCode<ExtractRobotStateExtension>() ||
        class_code == GetClassCode<ConveyorObjectStateExtension>())
    {
        return this;
    }

    if (class_code == GetClassCode<PointProjectionExtension>() ||
        class_code == GetClassCode<PoseProjectionExtension>())
    {
        // if (m_fk_iface) {
            return this;
        // }
    }

    return nullptr;
}

void ConveyorManipLatticeEgraph::eraseExperienceGraph()
{
    m_coord_to_nodes.clear();
    m_state_to_node.clear();
    m_egraph.m_nodes.clear();
    m_egraph.m_edges.clear();
    m_egraph_state_ids.clear();

    m_egraph.m_nodes.shrink_to_fit();
    m_egraph.m_edges.shrink_to_fit();
    m_egraph_state_ids.shrink_to_fit();

    clearStates();
}

bool ConveyorManipLatticeEgraph::checkExperienceGraphState(int state_id) const
{
    auto it = std::find(m_egraph_state_ids.begin(), m_egraph_state_ids.end(), state_id);
    if (it != m_egraph_state_ids.end()) {
        return true;
    }
    return false;
}

bool ConveyorManipLatticeEgraph::checkReplanCutoffInPath(
    const std::vector<RobotState>& path,
    double replan_cutoff)
{
    if (m_egraph.num_nodes() == 0) {
        return true;
    }

    RobotState path_state;
    bool found = false;
    for (const auto& wp : path) {
        if (fabs(wp.back() - replan_cutoff) < 1e-6) {
            path_state = wp;
            found = true;
            break;
        }
    }
    if (!found) {
        return false;
    }

    RobotState eg_state;
    found = false;
    for (const auto& egn : m_egraph.m_nodes) {
        if (fabs(egn.state.back() - replan_cutoff) < 1e-6) {
            eg_state = egn.state;
            found = true;
            break;
        }
    }
    if (!found) {
        SMPL_ERROR("Replan cutoff state not found in experience");
        return false;
    }

    for (size_t i = 0; i < path_state.size(); ++i) {
        if (fabs(path_state[i] - eg_state[i] > 1e-6)) {
            return false;
        }
    }
    return true;
}

bool ConveyorManipLatticeEgraph::findShortestExperienceGraphPath(
    ExperienceGraph::node_id start_node,
    ExperienceGraph::node_id goal_node,
    std::vector<ExperienceGraph::node_id>& path)
{
    struct ExperienceGraphSearchNode : heap_element
    {
        int g;
        bool closed;
        ExperienceGraphSearchNode* bp;
        ExperienceGraphSearchNode() :
            g(std::numeric_limits<int>::max()),
            closed(false),
            bp(nullptr)
        { }
    };

    struct NodeCompare
    {
        bool operator()(
            const ExperienceGraphSearchNode& a,
            const ExperienceGraphSearchNode& b)
        {
            return a.g < b.g;
        }
    };

    typedef intrusive_heap<ExperienceGraphSearchNode, NodeCompare> heap_type;

    std::vector<ExperienceGraphSearchNode> search_nodes(m_egraph.num_nodes());

    heap_type open;

    search_nodes[start_node].g = 0;
    open.push(&search_nodes[start_node]);
    int exp_count = 0;
    while (!open.empty()) {
        ++exp_count;
        ExperienceGraphSearchNode* min = open.min();
        open.pop();
        min->closed = true;

        if (min == &search_nodes[goal_node]) {
            SMPL_ERROR("Found shortest experience graph path");
            ExperienceGraphSearchNode* ps = nullptr;
            for (ExperienceGraphSearchNode* s = &search_nodes[goal_node];
                s; s = s->bp)
            {
                if (s != ps) {
                    path.push_back(std::distance(search_nodes.data(), s));
                    ps = s;
                } else {
                    SMPL_ERROR("Cycle detected!");
                }
            }
            std::reverse(path.begin(), path.end());
            return true;
        }

        ExperienceGraph::node_id n = std::distance(search_nodes.data(), min);
        auto adj = m_egraph.adjacent_nodes(n);
        for (auto ait = adj.first; ait != adj.second; ++ait) {
            ExperienceGraphSearchNode& succ = search_nodes[*ait];
            if (succ.closed) {
                continue;
            }
            int new_cost = min->g + 1;
            if (new_cost < succ.g) {
                succ.g = new_cost;
                succ.bp = min;
                if (open.contains(&succ)) {
                    open.decrease(&succ);
                } else {
                    open.push(&succ);
                }
            }
        }
    }

    SMPL_INFO("Expanded %d nodes looking for shortcut", exp_count);
    return false;
}

bool ConveyorManipLatticeEgraph::parseExperienceGraphFile(
    const std::string& filepath,
    std::vector<RobotState>& egraph_states) const
{
    std::ifstream fin(filepath);
    if (!fin.is_open()) {
        return false;
    }

    CSVParser parser;
    const bool with_header = true;
    if (!parser.parseStream(fin, with_header)) {
        SMPL_ERROR("Failed to parse experience graph file '%s'", filepath.c_str());
        return false;
    }

    SMPL_INFO("Parsed experience graph file");
    SMPL_INFO("  Has Header: %s", parser.hasHeader() ? "true" : "false");
    SMPL_INFO("  %zu records", parser.recordCount());
    SMPL_INFO("  %zu fields", parser.fieldCount());

    const size_t jvar_count = robot()->getPlanningJoints().size();
    if (parser.fieldCount() != jvar_count + 1) {
        SMPL_ERROR("Parsed experience graph contains insufficient number of joint variables");
        return false;
    }

    egraph_states.reserve(parser.totalFieldCount());
    for (size_t i = 0; i < parser.recordCount(); ++i) {
        RobotState state(jvar_count + 1);
        for (size_t j = 0; j < parser.fieldCount(); ++j) {
            try {
                state[j] = std::stod(parser.fieldAt(i, j));
            } catch (const std::invalid_argument& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            } catch (const std::out_of_range& ex) {
                SMPL_ERROR("Failed to parse egraph state variable (%s)", ex.what());
                return false;
            }
        }
        egraph_states.push_back(std::move(state));
    }

    SMPL_INFO("Read %zu states from experience graph file", egraph_states.size());
    return true;
}

/// An attempt to construct the discrete experience graph by discretizing all
/// input continuous states and connecting them via edges available in the
/// canonical action set. This turns out to not work very well since the points
/// are not often able to be connected by the limited action set. It also
/// necessitates imposing restrictions on the action set, since context-specific
/// actions don't make sense before an actual planning request.
void ConveyorManipLatticeEgraph::rasterizeExperienceGraph()
{
//    std::vector<RobotCoord> egraph_coords;
//    for (const RobotState& state : egraph_states) {
//        RobotCoord coord(robot()->jointVariableCount());
//        stateToCoord(state, coord);
//        egraph_coords.push_back(std::move(coord));
//    }
//
//    auto it = std::unique(egraph_coords.begin(), egraph_coords.end());
//    egraph_coords.erase(it, egraph_coords.end());
//
//    SMPL_INFO("Experience contains %zu discrete states", egraph_coords.size());
//    for (const RobotCoord& coord : egraph_coords) {
//        SMPL_STREAM_INFO("  " << coord);
//    }
//
//    SMPL_INFO("Insert states into experience graph and map coords to experience graph nodes");
//
//    // insert all coords into egraph and initialize coord -> egraph node mapping
//    RobotState state(robot()->jointVariableCount());
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        coordToState(*it, state);
//        m_coord_to_nodes[*it] = m_egraph.insert_node(state);
//    }
//
//    SMPL_INFO("Insert experience graph edges into experience graph");
//
//    int edge_count = 0;
//    ConveyorManipLatticeActionSpace* aspace =
//            dynamic_cast<ConveyorManipLatticeActionSpace*>(actionSpace().get());
//    if (!aspace) {
//        SMPL_ERROR("ConveyorManipLatticeEgraph requires action space to be a ConveyorManipLatticeActionSpace");
//        return false;
//    }
//
//    // save action space configuration
//    bool mprim_enabled_state[MotionPrimitive::NUMBER_OF_MPRIM_TYPES];
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        mprim_enabled_state[i] = aspace->useAmp((MotionPrimitive::Type)i);
//    }
//    bool use_long_and_short_mprims = aspace->useLongAndShortPrims();
//
//    // disable context-specific motion primitives
//    aspace->useAmp(MotionPrimitive::SHORT_DISTANCE, true);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_RPY, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ, false);
//    aspace->useAmp(MotionPrimitive::SNAP_TO_XYZ_RPY, false);
//
//    for (auto it = egraph_coords.begin(); it != egraph_coords.end(); ++it) {
//        const ExperienceGraph::node_id n = m_coord_to_nodes[*it];
//        RobotState source(robot()->jointVariableCount());
//        coordToState(*it, source);
//        std::vector<Action> actions;
//        aspace->apply(source, actions);
//        SMPL_INFO("%zu actions from egraph state", actions.size());
//        for (const Action& action : actions) {
//            RobotCoord last(robot()->jointVariableCount());
//            stateToCoord(action.back(), last);
//            SMPL_INFO("Check for experience graph edge " << *it << " -> " << last);
//            auto iit = m_coord_to_nodes.find(last);
//            if (iit != m_coord_to_nodes.end() && !m_egraph.edge(n, iit->second)) {
//                m_egraph.insert_edge(n, iit->second);
//                ++edge_count;
//            }
//        }
//    }
//    SMPL_INFO("Experience graph contains %d edges", edge_count);
//
//    // restore action space configuration
//    for (int i = 0; i < MotionPrimitive::NUMBER_OF_MPRIM_TYPES; ++i) {
//        aspace->useAmp((MotionPrimitive::Type)i, mprim_enabled_state[i]);
//    }
//    aspace->useLongAndShortPrims(use_long_and_short_mprims);
}

} // namespace smpl
