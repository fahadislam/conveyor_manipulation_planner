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

/// \author Andrew Dornbush

#ifndef SMPL_MANIP_LATTICE_EGRAPH_H
#define SMPL_MANIP_LATTICE_EGRAPH_H

#include <smpl/graph/experience_graph.h>
#include <smpl/graph/experience_graph_extension.h>
#include "conveyor_manip_lattice.h"

namespace smpl {

class ConveyorManipLatticeEgraph : public ConveyorManipLattice, public ExperienceGraphExtension
{
public:

    /// \name Reimplemented Public Functions from ConveyorManipLattice
    ///@{
    bool extractPath(
        const std::vector<int>& ids,
        std::vector<RobotState>& path) override;
    ///@}

    /// \name Required Public Functions from ExperienceGraphExtension
    ///@{
    bool loadExperienceGraph(const std::string& path) override;

    bool loadExperienceGraph(const std::vector<std::string>& paths);

    void getExperienceGraphNodes(
        int state_id,
        std::vector<ExperienceGraph::node_id>& nodes) override;

    bool shortcut(
        int first_id,
        int second_id,
        int& cost) override;

    bool snap(
        int first_id,
        int second_id,
        int& cost) override;

    const ExperienceGraph* getExperienceGraph() const override;
    ExperienceGraph* getExperienceGraph() override;

    int getStateID(ExperienceGraph::node_id n) const override;
    ///@}

    /// \name Reimplemented Public Functions from Extension
    ///@{
    Extension* getExtension(size_t class_code) override;
    ///@}
    void eraseExperienceGraph();
    // void clearStates();
    bool checkExperienceGraphState(int state_id) const;

private:

    struct RobotCoordHash
    {
        typedef std::vector<int> argument_type;
        typedef std::size_t result_type;

        result_type operator()(const argument_type& s) const;
    };

    typedef hash_map<
            RobotCoord,
            std::vector<ExperienceGraph::node_id>,
            RobotCoordHash>
    CoordToExperienceGraphNodeMap;

    CoordToExperienceGraphNodeMap m_coord_to_nodes;
    hash_map<int, ExperienceGraph::node_id> m_state_to_node;

    ExperienceGraph m_egraph;

    // map from experience graph node ids to state ids
    std::vector<int> m_egraph_state_ids;

    bool findShortestExperienceGraphPath(
        ExperienceGraph::node_id u,
        ExperienceGraph::node_id s,
        std::vector<ExperienceGraph::node_id>& path);

    bool parseExperienceGraphFile(
        const std::string& filepath,
        std::vector<RobotState>& egraph_states) const;

    void rasterizeExperienceGraph();

};

} // namespace smpl

#endif
