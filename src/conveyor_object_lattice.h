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

/// \author Fahad Islam

#ifndef SMPL_CONVEYOR_OBJECT_LATTICE_H
#define SMPL_CONVEYOR_OBJECT_LATTICE_H

#include <smpl/graph/manip_lattice.h>

namespace smpl {

template <typename T>
struct ValueHash
{
    typedef T argument_type;
    typedef std::size_t result_type;
    result_type operator()(argument_type s) const 
    {
        size_t seed = 0;
        boost::hash_combine(seed, boost::hash_range(s.begin(), s.end()));
        return seed;
    }
};

// helper struct to test for equality between two pointers by testing for
// equality between the objects they point to
template <typename T>
struct ValueEqual
{
    typedef T argument_type;
    bool operator()(argument_type a, argument_type b) const { return a == b; }
};

} // namespace std

namespace smpl {

/// \class Discrete space constructed by expliciting discretizing each joint
class ConveyorObjectLattice : public ManipLattice
{
public:

    ~ConveyorObjectLattice();

    void GetSuccs(
        int state_id,
        std::vector<int>* succs,
        std::vector<int>* costs) override;

    void setPathId(int start_id, int state_id, int path_id, bool singleton = false);
    std::pair<int, bool> getPathId(int start_id, const RobotState& state);
    bool saveStateToPathIdMap(std::string filepath);
    bool loadStateToPathIdMap(std::string filepath);

private:

    typedef ManipLatticeState StateKey2;
    typedef std::ValueHash<RobotCoord> StateHash2;
    typedef std::ValueEqual<RobotCoord> StateEqual2;
    typedef hash_map<RobotCoord, std::pair<int, bool>, StateHash2, StateEqual2> LookupTable;
    std::vector<LookupTable> m_state_to_pid;

    // LookupTable m_state_to_pid_home;
};

} // namespace smpl

#endif
