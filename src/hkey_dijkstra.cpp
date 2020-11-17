////////////////////////////////////////////////////////////////////////////////
// Copyright (c) 2017, Andrew Dornbush, Fahad Islam
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

/// \author Andrew Dornbush
/// \author Fahad Islam

#include "hkey_dijkstra.h"

#include <algorithm>

// system includes
#include <sbpl/utils/key.h>

// project includes
#include <smpl/time.h>
#include <smpl/console/console.h>

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>
#include <boost/serialization/unordered_map.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iostream>

namespace smpl {

static const char* SLOG = "search";
static const char* SELOG = "search.expansions";

HKeyDijkstra::HKeyDijkstra(
    DiscreteSpaceInformation* space,
    Heuristic* heur)
:
    SBPLPlanner(),
    m_space(space),
    m_heur(heur),
    m_time_params(),
    m_initial_eps(1.0),
    m_final_eps(1.0),
    m_delta_eps(1.0),
    m_allow_partial_solutions(false),
    m_states(),
    m_start_state_id(-1),
    m_goal_state_id(-1),
    m_open(),
    m_incons(),
    m_curr_eps(1.0),
    m_iteration(1),
    m_call_number(0),
    m_last_start_state_id(-1),
    m_last_goal_state_id(-1),
    m_last_eps(1.0),
    m_expand_count_init(0),
    m_expand_count(0),
    m_search_time_init(clock::duration::zero()),
    m_search_time(clock::duration::zero()),
    m_satisfied_eps(std::numeric_limits<double>::infinity())
{
    environment_ = space;

    m_time_params.bounded = true;
    m_time_params.improve = true;
    m_time_params.type = TimeParameters::TIME;
    m_time_params.max_expansions_init = 0;
    m_time_params.max_expansions = 0;
    m_time_params.max_allowed_time_init = clock::duration::zero();
    m_time_params.max_allowed_time = clock::duration::zero();
}

HKeyDijkstra::~HKeyDijkstra()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
}

enum ReplanResultCode
{
    SUCCESS = 0,
    PARTIAL_SUCCESS,
    START_NOT_SET,
    GOAL_NOT_SET,
    TIMED_OUT,
    EXHAUSTED_OPEN_LIST
};

bool HKeyDijkstra::reinit_search()
{
    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
    m_open.clear();
    m_incons.clear();
    ++m_call_number; // trigger state reinitializations

    reinitSearchState(start_state);
    reinitSearchState(goal_state);

    start_state->g = 0;
    start_state->f = computeKey(start_state);
    m_open.push(start_state);

    m_iteration = 1; // 0 reserved for "not closed on any iteration"

    m_expand_count_init = 0;
    m_search_time_init = clock::duration::zero();

    m_expand_count = 0;
    m_search_time = clock::duration::zero();

    m_curr_eps = m_initial_eps;

    m_satisfied_eps = std::numeric_limits<double>::infinity();

    m_last_start_state_id = m_start_state_id;

    return true;
}

void HKeyDijkstra::setUncoveredStates(const std::vector<int>& state_ids)
{
    m_uc_states = state_ids;
    m_init_uc_states = m_uc_states;
    m_dirty_states.clear();
    for (auto ss : m_states) {
        ss->covered = false;
        ss->dirty = false;
    }
    m_subregions.clear();
}

auto HKeyDijkstra::getAllStates()
    -> std::vector<int>
{
    return m_uc_states;
}

auto HKeyDijkstra::getUncoveredStates()
    -> std::vector<int>
{
    return m_dirty_states;
}

auto HKeyDijkstra::getCoveredStates()
    -> std::vector<int>
{
    auto covered_states = m_init_uc_states;
    subtractStates(covered_states, m_dirty_states);
    return covered_states;
}

void HKeyDijkstra::subtractStates(
    std::vector<int>& from_state_ids,
    const std::vector<int>& state_ids)
{
    for (const int id : state_ids) {
        auto it = std::find(from_state_ids.begin(), from_state_ids.end(), id);
        if (it != from_state_ids.end()) {
            from_state_ids.erase(it);
        }
    }
}

void HKeyDijkstra::addStates(
    std::vector<int>& to_state_ids,
    const std::vector<int>& state_ids)
{
    for (const int id : state_ids) {
        auto it = std::find(to_state_ids.begin(), to_state_ids.end(), id);
        if (it == to_state_ids.end()) {
            to_state_ids.push_back(id);
        }
    }
}

auto HKeyDijkstra::getSubregion(int start_id, int subregion_id)
    -> std::vector<int>
{
    // printf("start_id %d, subregion_id %d, m_goal_regions.size() %zu, subregion.size() %zu\n",
    //     start_id, subregion_id, m_goal_regions.size(), m_goal_regions[start_id].size());
    assert(start_id < m_goal_regions.size());
    assert(subregion_id < m_goal_regions[start_id].size());
    return m_goal_regions[start_id][subregion_id];
}

void HKeyDijkstra::appendSubregions()
{
    m_goal_regions.push_back(std::move(m_subregions));
}

int HKeyDijkstra::replan(
    const TimeParameters& params,
    std::vector<int>* solution,
    int* cost)
{
    SMPL_DEBUG_NAMED(SLOG, "Find path to goal");

    if (m_start_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Start state not set");
        return !START_NOT_SET;
    }
    if (m_goal_state_id < 0) {
        SMPL_ERROR_NAMED(SLOG, "Goal state not set");
        return !GOAL_NOT_SET;
    }

    m_time_params = params;

    SearchState* start_state = getSearchState(m_start_state_id);
    SearchState* goal_state = getSearchState(m_goal_state_id);

    if (m_start_state_id != m_last_start_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Reinitialize search");
        m_open.clear();
        m_incons.clear();
        ++m_call_number; // trigger state reinitializations

        reinitSearchState(start_state);
        reinitSearchState(goal_state);

        start_state->g = 0;
        start_state->f = computeKey(start_state);
        m_open.push(start_state);

        m_iteration = 1; // 0 reserved for "not closed on any iteration"

        m_expand_count_init = 0;
        m_search_time_init = clock::duration::zero();

        m_expand_count = 0;
        m_search_time = clock::duration::zero();

        m_curr_eps = m_initial_eps;

        m_satisfied_eps = std::numeric_limits<double>::infinity();

        m_last_start_state_id = m_start_state_id;
    }

    if (m_goal_state_id != m_last_goal_state_id) {
        SMPL_DEBUG_NAMED(SLOG, "Refresh heuristics, keys, and reorder open list");
        recomputeHeuristics();
        reorderOpen();

        m_last_goal_state_id = m_goal_state_id;
    }

    auto start_time = clock::now();
    int num_expansions = 0;
    clock::duration elapsed_time = clock::duration::zero();

    int err;
    while (m_satisfied_eps > m_final_eps) {
        if (m_curr_eps == m_satisfied_eps) {
            if (!m_time_params.improve) {
                break;
            }
            // begin a new search iteration
            ++m_iteration;
            m_curr_eps -= m_delta_eps;
            m_curr_eps = std::max(m_curr_eps, m_final_eps);
            for (SearchState* s : m_incons) {
                s->incons = false;
                m_open.push(s);
            }
            reorderOpen();
            m_incons.clear();
            SMPL_DEBUG_NAMED(SLOG, "Begin new search iteration %d with epsilon = %0.3f", m_iteration, m_curr_eps);
        }
        err = improvePath(start_time, goal_state, num_expansions, elapsed_time);
        if (m_curr_eps == m_initial_eps) {
            m_expand_count_init += num_expansions;
            m_search_time_init += elapsed_time;
        }
        if (err) {
            break;
        }
        SMPL_DEBUG_NAMED(SLOG, "Improved solution");
        m_satisfied_eps = m_curr_eps;
    }

    m_search_time += elapsed_time;
    m_expand_count += num_expansions;

    if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
        if (m_allow_partial_solutions && !m_open.empty()) {
            SearchState* next_state = m_open.min();
            extractPath(next_state, *solution, *cost);
            return !SUCCESS;
        }
        return !err;
    }

    extractPath(goal_state, *solution, *cost);
    return !SUCCESS;
}

int HKeyDijkstra::replan(
    double allowed_time,
    std::vector<int>* solution)
{
    int cost;
    return replan(allowed_time, solution, &cost);
}

// decide whether to start the search from scratch
//
// if start changed
//     reset the search to its initial state
// if goal changed
//     reevaluate heuristics
//     reorder the open list
//
// case scenario_hasnt_changed (start and goal the same)
//   case have solution for previous epsilon
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           pass
//   case dont have solution
//       case epsilon lowered
//           reevaluate heuristics and reorder the open list
//       case epsilon raised
//           reevaluate heuristics and reorder the open list
// case scenario_changed
int HKeyDijkstra::replan(
    double allowed_time,
    std::vector<int>* solution,
    int* cost)
{
    TimeParameters tparams = m_time_params;
    if (tparams.max_allowed_time_init == tparams.max_allowed_time) {
        // NOTE/TODO: this may lead to awkward behavior, if the caller sets the
        // allowed time to the current repair time, the repair time will begin
        // to track the allowed time for further calls to replan. perhaps set
        // an explicit flag for using repair time or an indicator value as is
        // done with ReplanParams
        tparams.max_allowed_time_init = to_duration(allowed_time);
        tparams.max_allowed_time = to_duration(allowed_time);
    } else {
        tparams.max_allowed_time_init = to_duration(allowed_time);
        // note: retain original allowed improvement time
    }
    return replan(tparams, solution, cost);
}

int HKeyDijkstra::replan(
    std::vector<int>* solution,
    ReplanParams params)
{
    int cost;
    return replan(solution, params, &cost);
}

int HKeyDijkstra::replan(
    std::vector<int>* solution,
    ReplanParams params,
    int* cost)
{
    // note: if replan fails before internal time parameters are updated (this
    // happens if the start or goal has not been set), then the internal
    // epsilons may be affected by this set of ReplanParams for future calls to
    // replan where ReplanParams is not used and epsilon parameters haven't been
    // set back to their desired values.
    TimeParameters tparams;
    convertReplanParamsToTimeParams(params, tparams);
    return replan(tparams, solution, cost);
}

/// Force the planner to forget previous search efforts, begin from scratch,
/// and free all memory allocated by the planner during previous searches.
int HKeyDijkstra::force_planning_from_scratch_and_free_memory()
{
    force_planning_from_scratch();
    m_open.clear();
    for (SearchState* s : m_states) {
        if (s != NULL) {
            delete s;
        }
    }
    m_states.clear();
    m_states.shrink_to_fit();
    return 0;
}

/// Return the suboptimality bound of the current solution for the current search.
double HKeyDijkstra::get_solution_eps() const
{
    return m_satisfied_eps;
}

/// Return the number of expansions made in progress to the final solution.
int HKeyDijkstra::get_n_expands() const
{
    return m_expand_count;
}

/// Return the initial suboptimality bound
double HKeyDijkstra::get_initial_eps()
{
    return m_initial_eps;
}

/// Return the time consumed by the search in progress to the initial solution.
double HKeyDijkstra::get_initial_eps_planning_time()
{
    return to_seconds(m_search_time_init);
}

/// Return the time consumed by the search in progress to the final solution.
double HKeyDijkstra::get_final_eps_planning_time()
{
    return to_seconds(m_search_time);
}

/// Return the number of expansions made in progress to the initial solution.
int HKeyDijkstra::get_n_expands_init_solution()
{
    return m_expand_count_init;
}

/// Return the final suboptimality bound.
double HKeyDijkstra::get_final_epsilon()
{
    return m_final_eps;
}

/// Return statistics for each completed search iteration.
void HKeyDijkstra::get_search_stats(std::vector<PlannerStats>* s)
{
    PlannerStats stats;
    stats.eps = m_curr_eps;
//    stats.cost; // TODO: implement
    stats.expands = m_expand_count;
    stats.time = to_seconds(m_search_time);
    s->push_back(stats);
}

/// Set the desired suboptimality bound for the initial solution.
void HKeyDijkstra::set_initialsolution_eps(double eps)
{
    m_initial_eps = eps;
}

/// Set the goal state.
int HKeyDijkstra::set_goal(int goal_state_id)
{
    m_goal_state_id = goal_state_id;
    return 1;
}

/// Set the start state.
int HKeyDijkstra::set_start(int start_state_id)
{
    m_start_state_id = start_state_id;
    return 1;
}

/// Force the search to forget previous search efforts and start from scratch.
int HKeyDijkstra::force_planning_from_scratch()
{
    m_last_start_state_id = -1;
    m_last_goal_state_id = -1;
    return 0;
}

/// Set whether the number of expansions is bounded by time or total expansions
/// per call to replan().
int HKeyDijkstra::set_search_mode(bool first_solution_unbounded)
{
    m_time_params.bounded = !first_solution_unbounded;
    return 0;
}

/// Notify the search of changes to edge costs in the graph.
void HKeyDijkstra::costs_changed(const StateChangeQuery& changes)
{
    force_planning_from_scratch();
}

// Recompute heuristics for all states.
void HKeyDijkstra::recomputeHeuristics()
{
    for (SearchState* s : m_states) {
        if (s != NULL) {
            s->h = m_heur->GetGoalHeuristic(s->state_id);
        }
    }
}

// Convert TimeParameters to ReplanParams. Uses the current epsilon values
// to fill in the epsilon fields.
void HKeyDijkstra::convertTimeParamsToReplanParams(
    const TimeParameters& t,
    ReplanParams& r) const
{
    r.max_time = to_seconds(t.max_allowed_time_init);
    r.return_first_solution = !t.bounded && !t.improve;
    if (t.max_allowed_time_init == t.max_allowed_time) {
        r.repair_time = -1.0;
    } else {
        r.repair_time = to_seconds(t.max_allowed_time);
    }

    r.initial_eps = m_initial_eps;
    r.final_eps = m_final_eps;
    r.dec_eps = m_delta_eps;
}

// Convert ReplanParams to TimeParameters. Sets the current initial, final, and
// delta eps from ReplanParams.
void HKeyDijkstra::convertReplanParamsToTimeParams(
    const ReplanParams& r,
    TimeParameters& t)
{
    t.type = TimeParameters::TIME;

    t.bounded = !r.return_first_solution;
    t.improve = !r.return_first_solution;

    t.max_allowed_time_init = to_duration(r.max_time);
    if (r.repair_time > 0.0) {
        t.max_allowed_time = to_duration(r.repair_time);
    } else {
        t.max_allowed_time = t.max_allowed_time_init;
    }

    m_initial_eps = r.initial_eps;
    m_final_eps = r.final_eps;
    m_delta_eps = r.dec_eps;
}

// Test whether the search has run out of time.
bool HKeyDijkstra::timedOut(
    int elapsed_expansions,
    const clock::duration& elapsed_time) const
{
    if (!m_time_params.bounded) {
        return false;
    }

    switch (m_time_params.type) {
    case TimeParameters::EXPANSIONS:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_expansions >= m_time_params.max_expansions_init;
        } else {
            return elapsed_expansions >= m_time_params.max_expansions;
        }
    case TimeParameters::TIME:
        if (m_satisfied_eps == std::numeric_limits<double>::infinity()) {
            return elapsed_time >= m_time_params.max_allowed_time_init;
        } else {
            return elapsed_time >= m_time_params.max_allowed_time;
        }
    case TimeParameters::USER:
        return m_time_params.timed_out_fun();
    default:
        SMPL_ERROR_NAMED(SLOG, "Invalid timer type");
        return true;
    }

    return true;
}


int HKeyDijkstra::sampleObjectState(int subregion_id)
{
    // printf("\n\n\n");
    // SMPL_INFO("##############    Uncovered: %zu, Total: %zu, Dirty: %zu    #############",
    //         m_uc_states.size(), m_init_uc_states.size(), m_dirty_states.size());

    if (m_uc_states.empty()) {
        SMPL_DEBUG("All non-dirty states covered, remaining dirty states %zu",
                    m_dirty_states.size());
        return -1;
    }

    // else if (m_uc_states.size() == 1) {
    //     return m_uc_states[0];
    // }

    int idx = (std::rand() % (m_uc_states.size()));

    auto state = getSearchState(m_uc_states[idx]);
    assert(state->covered);

    // to take care of the case when the sampled object
    // state turns out to be dirty
    // we dont want to add an extra subregion

    assert(subregion_id == m_subregions.size()
        || subregion_id == m_subregions.size() - 1);
    if (subregion_id == m_subregions.size()) {
        std::vector<int> sr;
        m_subregions.push_back(sr);
    }

    m_eidx = 0;

    // printf("Num subregions: %zu\n", m_subregions.size());

    return m_uc_states[idx];
}

void HKeyDijkstra::markDirtyState(int state_id)
{
    SMPL_DEBUG("Marking state %d as dirty", state_id);
    auto state = getSearchState(state_id);
    assert(!state->dirty);
    state->dirty = true;
    m_dirty_states.push_back(state_id);
}

int HKeyDijkstra::getNextStateId()
{
    // SMPL_INFO("     Uncovered: %zu, Total %zu, Dirty %zu, Expands: %d, Open: %zu\n ",
    //         m_uc_states.size(), m_init_uc_states.size(), m_dirty_states.size(), m_expand_count, m_open.size());

    // if (m_open.empty()) {
    //     SMPL_INFO("Open list got empty");
    //     return -1;
    // }
    // printf("m_eidx %d\n", m_eidx);

    // if (m_uc_states.empty()) {
    //     SMPL_DEBUG("No more uncovered states");
    //     return -1;
    // }

    if (m_eidx > m_uc_states.size() - 1) {
        SMPL_DEBUG("No more uncovered states");
        return -1;
    }

    // SearchState* min_state = m_open.min();
    SearchState* min_state = getSearchState(m_uc_states[m_eidx]);
    m_eidx++;


    SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

    // m_open.pop();
    assert(min_state->iteration_closed != m_iteration);
    assert(min_state->g != INFINITECOST);

    min_state->iteration_closed = m_iteration;
    min_state->eg = min_state->g;

    // expand(min_state);

    ++m_expand_count;

    if (min_state->covered && !min_state->dirty) {
        printf("State %d already covered and not dirty, skipping!\n", min_state->state_id);
        return -2;
    }
    // printf("size %zu expands %d\n", m_open.size(), m_expand_count);

    return min_state->state_id;
}

void HKeyDijkstra::removeStateFromUncoveredList(int state_id)
{
    auto state = getSearchState(state_id);
    if (!state->covered) {
        SMPL_DEBUG("Removing state %d from UC list\n", state_id);
        auto it = std::find(m_uc_states.begin(), m_uc_states.end(), state_id);
        assert(it != m_uc_states.end());
        state->covered = true;
        m_uc_states.erase(it);
        m_eidx--;
    }
    SMPL_INFO("     Uncovered: %zu, Total %zu, Dirty %zu, Expands: %d\n ",
            m_uc_states.size(), m_init_uc_states.size(), m_dirty_states.size(), m_expand_count);

    return;
}

void HKeyDijkstra::removeStateFromDirtyList(int state_id)
{
    auto state = getSearchState(state_id);
    if (state->dirty) {
        SMPL_DEBUG("Removing state %d from Dirty list\n", state_id);
        state->dirty = false;
        auto it = std::find(m_dirty_states.begin(), m_dirty_states.end(), state_id);
        assert(it != m_dirty_states.end());
        m_dirty_states.erase(it);
    }
    return;
}

void HKeyDijkstra::addStateToSubregion(int subregion_id, int state_id)
{
    // state covered and not dirty, then add to subregion
    assert(subregion_id == m_subregions.size() - 1);
    auto state = getSearchState(state_id);
    if (!state->dirty) {
        m_subregions[subregion_id].push_back(state_id);
    }
}

// Expand states to improve the current solution until a solution within the
// current suboptimality bound is found, time runs out, or no solution exists.
int HKeyDijkstra::improvePath(
    const clock::time_point& start_time,
    SearchState* goal_state,
    int& elapsed_expansions,
    clock::duration& elapsed_time)
{
    while (!m_open.empty()) {
        SearchState* min_state = m_open.min();

        auto now = clock::now();
        elapsed_time = now - start_time;

        // path to goal found
        if (min_state->f >= goal_state->f || min_state == goal_state) {
            SMPL_DEBUG_NAMED(SLOG, "Found path to goal");
            return SUCCESS;
        }

        if (timedOut(elapsed_expansions, elapsed_time)) {
            SMPL_DEBUG_NAMED(SLOG, "Ran out of time");
            return TIMED_OUT;
        }

        SMPL_DEBUG_NAMED(SELOG, "Expand state %d", min_state->state_id);

        m_open.pop();

        assert(min_state->iteration_closed != m_iteration);
        assert(min_state->g != INFINITECOST);

        min_state->iteration_closed = m_iteration;
        min_state->eg = min_state->g;

        expand(min_state);

        ++elapsed_expansions;
    }

    return EXHAUSTED_OPEN_LIST;
}

// Expand a state, updating its successors and placing them into OPEN, CLOSED,
// and INCONS list appropriately.
void HKeyDijkstra::expand(SearchState* s)
{
    // printf("h val %d\n", s->f);
    m_succs.clear();
    m_costs.clear();
    m_space->GetSuccs(s->state_id, &m_succs, &m_costs);

    SMPL_DEBUG_NAMED(SELOG, "  %zu successors", m_succs.size());

    for (size_t sidx = 0; sidx < m_succs.size(); ++sidx) {
        int succ_state_id = m_succs[sidx];
        int cost = m_costs[sidx];

        SearchState* succ_state = getSearchState(succ_state_id);
        reinitSearchState(succ_state);
        // printf("    state %d h val %d\n", succ_state_id, succ_state->f);

        int new_cost = s->eg + cost;
        SMPL_DEBUG_NAMED(SELOG, "Compare new cost %d vs old cost %d", new_cost, succ_state->g);
        if (new_cost < succ_state->g) {
            succ_state->g = new_cost;
            succ_state->bp = s;
            if (succ_state->iteration_closed != m_iteration) {
                succ_state->f = computeKey(succ_state);
                if (m_open.contains(succ_state)) {
                    m_open.decrease(succ_state);
                } else {
                    auto it = std::find(m_uc_states.begin(), m_uc_states.end(), succ_state_id);
                    if(it != m_uc_states.end()) {
                        m_open.push(succ_state);
                    }
                }
            } else if (!succ_state->incons) {
                m_incons.push_back(succ_state);
            }
        }
    }
}

// Recompute the f-values of all states in OPEN and reorder OPEN.
void HKeyDijkstra::reorderOpen()
{
    for (auto it = m_open.begin(); it != m_open.end(); ++it) {
        (*it)->f = computeKey(*it);
    }
    m_open.make();
}

int HKeyDijkstra::computeKey(SearchState* s) const
{
    return (unsigned int)(s->h);
}

// Get the search state corresponding to a graph state, creating a new state if
// one has not been created yet.
HKeyDijkstra::SearchState* HKeyDijkstra::getSearchState(int state_id)
{
    if (m_states.size() <= state_id) {
        m_states.resize(state_id + 1, nullptr);
    }

    auto& state = m_states[state_id];
    if (state == NULL) {
        state = createState(state_id);
    }

    return state;
}

// Create a new search state for a graph state.
HKeyDijkstra::SearchState* HKeyDijkstra::createState(int state_id)
{
    assert(state_id < m_states.size());

    SearchState* ss = new SearchState;
    ss->state_id = state_id;
    ss->call_number = 0;
    ss->covered = false;
    ss->dirty = false;

    if (state_id != m_goal_state_id) {
        m_uc_states.push_back(state_id);
        // printf("state %d\n", state_id);
    }

    return ss;
}

// Lazily (re)initialize a search state.
void HKeyDijkstra::reinitSearchState(SearchState* state)
{
    if (state->call_number != m_call_number) {
        SMPL_DEBUG_NAMED(SELOG, "Reinitialize state %d", state->state_id);
        state->g = INFINITECOST;
        state->h = m_heur->GetGoalHeuristic(state->state_id);
        state->f = INFINITECOST;
        state->eg = INFINITECOST;
        state->iteration_closed = 0;
        state->call_number = m_call_number;
        state->bp = nullptr;
        state->incons = false;
    }
}

// Extract the path from the start state up to a new state.
void HKeyDijkstra::extractPath(
    SearchState* to_state,
    std::vector<int>& solution,
    int& cost) const
{
    for (SearchState* s = to_state; s; s = s->bp) {
        solution.push_back(s->state_id);
    }
    std::reverse(solution.begin(), solution.end());
    cost = to_state->g;
}

} // namespace smpl
