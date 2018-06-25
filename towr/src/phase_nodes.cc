/******************************************************************************
Copyright (c) 2018, Alexander W. Winkler. All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

* Redistributions of source code must retain the above copyright notice, this
  list of conditions and the following disclaimer.

* Redistributions in binary form must reproduce the above copyright notice,
  this list of conditions and the following disclaimer in the documentation
  and/or other materials provided with the distribution.

* Neither the name of the copyright holder nor the names of its
  contributors may be used to endorse or promote products derived from
  this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
******************************************************************************/

#include <towr/variables/phase_nodes.h>
#include <towr/variables/cartesian_dimensions.h>

namespace towr {

std::vector<PhaseNodes::PolyInfo>
BuildPolyInfos (int phase_count, bool is_in_contact_at_start,
                int n_polys_in_changing_phase, PhaseNodes::Type type)
{
  using PolyInfo = PhaseNodes::PolyInfo;
  std::vector<PolyInfo> polynomial_info;

  bool first_phase_constant = (is_in_contact_at_start  && type==PhaseNodes::Motion)
                           || (!is_in_contact_at_start && type==PhaseNodes::Force);


  bool phase_constant = first_phase_constant;

  for (int i=0; i<phase_count; ++i) {
    if (phase_constant)
      polynomial_info.push_back(PolyInfo(i,0,1, true));
    else
      for (int j=0; j<n_polys_in_changing_phase; ++j)
        polynomial_info.push_back(PolyInfo(i,j,n_polys_in_changing_phase, false));

    phase_constant = !phase_constant; // constant and non-constant phase alternate
  }

  return polynomial_info;
}

PhaseNodes::PhaseNodes (int phase_count,
                        bool is_in_contact_at_start,
                        const std::string& name,
                        int n_polys_in_changing_phase,
                        Type type)
    :Nodes(3, name)
{
  polynomial_info_ = BuildPolyInfos(phase_count, is_in_contact_at_start, n_polys_in_changing_phase, type);
  optnode_to_node_ = GetOptNodeToNodeMappings(polynomial_info_);

  int n_opt_variables = optnode_to_node_.size() * 2*GetDim();
  int n_nodes = polynomial_info_.size()+1;
  InitMembers(n_nodes, n_opt_variables);

  if (type == Motion)
    SetBoundsEEMotion();
  else if (type == Force)
    SetBoundsEEForce();
  else
    assert(false); // phase-node type not defined
}

PhaseNodes::VecDurations
PhaseNodes::ConvertPhaseToPolyDurations(const VecDurations& phase_durations) const
{
  VecDurations poly_durations;

  for (int i=0; i<GetPolynomialCount(); ++i) {
    auto info = polynomial_info_.at(i);
    poly_durations.push_back(phase_durations.at(info.phase_)/info.n_polys_in_phase_);
  }

  return poly_durations;
}

double
PhaseNodes::GetDerivativeOfPolyDurationWrtPhaseDuration (int poly_id) const
{
  int n_polys_in_phase = polynomial_info_.at(poly_id).n_polys_in_phase_;
  return 1./n_polys_in_phase;
}

int
PhaseNodes::GetNumberOfPrevPolynomialsInPhase(int poly_id) const
{
  return polynomial_info_.at(poly_id).poly_in_phase_;
}

std::map<PhaseNodes::OptNodeIs, PhaseNodes::NodeIds>
PhaseNodes::GetOptNodeToNodeMappings (const std::vector<PolyInfo>& polynomial_info)
{
  std::map<OptNodeIs, NodeIds> optnode_to_node;

  int opt_id = 0;
  for (int i=0; i<polynomial_info.size(); ++i) {
    int node_id_start = GetNodeId(i, Side::Start);

    optnode_to_node[opt_id].push_back(node_id_start);
    // use same value for next node if polynomial is constant
    if (!polynomial_info.at(i).is_constant_)
      opt_id++;
  }

  int last_node_id = polynomial_info.size();
  optnode_to_node[opt_id].push_back(last_node_id);

  return optnode_to_node;
}

std::vector<PhaseNodes::IndexInfo>
PhaseNodes::GetNodeInfoAtOptIndex(int idx) const
{
  std::vector<IndexInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*GetDim();
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  IndexInfo node;
  node.node_deriv_     = internal_id<GetDim()? kPos : kVel;
  node.node_dim_ = internal_id%GetDim();

  // this is different compared to standard node values
  // because one index can represent multiple node (during constant phase)
  int opt_node = std::floor(idx/n_opt_values_per_node_);
  for (auto node_id : optnode_to_node_.at(opt_node)) {
    node.node_id_ = node_id;
    nodes.push_back(node);
  }

  return nodes;
}

bool
PhaseNodes::IsConstantNode (int node_id) const
{
  bool is_constant = false;

  // node is considered constant if either left or right polynomial
  // belongs to a constant phase
  for (int poly_id : GetAdjacentPolyIds(node_id))
    if (IsInConstantPhase(poly_id))
      is_constant = true;

  return is_constant;
}

bool
PhaseNodes::IsInConstantPhase(int poly_id) const
{
  return polynomial_info_.at(poly_id).is_constant_;
}

PhaseNodes::NodeIds
PhaseNodes::GetIndicesOfNonConstantNodes() const
{
  NodeIds node_ids;

  for (int id=0; id<GetNodes().size(); ++id)
    if (!IsConstantNode(id))
      node_ids.push_back(id);

  return node_ids;
}

int
PhaseNodes::GetPhase (int node_id) const
{
  assert(!IsConstantNode(node_id)); // because otherwise it has two phases

  int poly_id = GetAdjacentPolyIds(node_id).front();
  return polynomial_info_.at(poly_id).phase_;
}

int
PhaseNodes::GetPolyIDAtStartOfPhase (int phase) const
{
  int poly_id=0;
  for (int i=0; i<polynomial_info_.size(); ++i)
    if (polynomial_info_.at(i).phase_ == phase)
      return i;
}

Eigen::Vector3d
PhaseNodes::GetValueAtStartOfPhase (int phase) const
{
  int node_id = GetNodeIDAtStartOfPhase(phase);
  return GetNodes().at(node_id).p();
}

int
PhaseNodes::GetNodeIDAtStartOfPhase (int phase) const
{
  int poly_id=GetPolyIDAtStartOfPhase(phase);
  return GetNodeId(poly_id, Side::Start);
}

std::vector<int>
PhaseNodes::GetAdjacentPolyIds (int node_id) const
{
  std::vector<int> poly_ids;
  int last_node_id = GetNodes().size()-1;

  if (node_id==0)
    poly_ids.push_back(0);
  else if (node_id==last_node_id)
    poly_ids.push_back(last_node_id-1);
  else {
    poly_ids.push_back(node_id-1);
    poly_ids.push_back(node_id);
  }

  return poly_ids;
}

void
PhaseNodes::SetBoundsEEMotion ()
{
  for (int i=0; i<GetRows(); ++i) {

    auto idx = GetNodeInfoAtOptIndex(i).front(); // bound idx by first node it represents

    // stance node
    if (IsConstantNode(idx.node_id_)) {

      // endeffector is not allowed to move if in stance phase
      if (idx.node_deriv_ == kVel)
        bounds_.at(i) = ifopt::BoundZero;

    // swing node
    } else {

      // zero velocity at top
      if (idx.node_deriv_ == kVel && idx.node_dim_ == Z)
        bounds_.at(i) = ifopt::BoundZero;

      /*
      // to not have really fast swing motions
      if (idx.node_deriv_ == kVel)
        bounds_.at(i) = ifopt::Bounds(-5.0, 5.0); // zero velocity at top
      */
    }
  }
}

void
PhaseNodes::SetBoundsEEForce ()
{
  for (int i=0; i<GetRows(); ++i) {

    IndexInfo idx = GetNodeInfoAtOptIndex(i).front(); // only one node anyway

    // stance node
    if (!IsConstantNode(idx.node_id_)) {

      /*
      // zero slope to never exceed zero force between nodes
      if (idx.node_deriv_ == kVel) {
        bounds_.at(i) = ifopt::BoundZero;
      }
      */

    // swing node
    } else {
      bounds_.at(i) = ifopt::BoundZero; // force must be zero
    }

  }
}

PhaseNodes::PolyInfo::PolyInfo(int phase, int poly_id_in_phase,
                               int num_polys_in_phase, bool is_constant)
    :phase_(phase),
     poly_in_phase_(poly_id_in_phase),
     n_polys_in_phase_(num_polys_in_phase),
     is_constant_(is_constant)
{
}

} /* namespace towr */
