/**
 @file    phase_nodes1.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 4, 2017
 @brief   Brief description
 */

#include <towr/variables/phase_nodes.h>

#include <cassert>

#include <xpp_states/cartesian_declarations.h>
#include <xpp_states/state.h>


namespace towr {

using namespace ifopt;
using namespace xpp;


PhaseNodes::PhaseNodes (int n_dim,
                        int phase_count,
                        bool is_in_contact_at_start,
                        const std::string& name,
                        int n_polys_in_changing_phase,
                        Type type)
    :NodeValues(n_dim,
                BuildPolyInfos(phase_count, is_in_contact_at_start, n_polys_in_changing_phase, type),
                name)
{
  SetNodeMappings();

  int n_opt_variables = optnode_to_node_.size() * 2*n_dim_;
  SetRows(n_opt_variables);

  // default, non initialized values
  nodes_  = std::vector<Node>(polynomial_info_.size()+1);
  bounds_ = VecBound(GetRows(), NoBound);
  CacheNodeInfoToIndexMappings();
}


void
PhaseNodes::SetNodeMappings ()
{
  int opt_id = 0;
  for (int i=0; i<polynomial_info_.size(); ++i) {
    int node_id_start = GetNodeId(i, CubicHermitePoly::Start);

    optnode_to_node_[opt_id].push_back(node_id_start);
    // use same value for next node if polynomial is constant
    if (!polynomial_info_.at(i).is_constant_)
      opt_id++;
  }

  int last_node_id = polynomial_info_.size();
  optnode_to_node_[opt_id].push_back(last_node_id);
}


std::vector<PhaseNodes::NodeInfo>
PhaseNodes::GetNodeInfoAtOptIndex(int idx) const
{
  std::vector<NodeInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*n_dim_;
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  NodeInfo node;
  node.deriv_ = internal_id<n_dim_? kPos : kVel;
  node.dim_   = internal_id%n_dim_;

  int opt_node = std::floor(idx/n_opt_values_per_node_);
  for (auto node_id : optnode_to_node_.at(opt_node)) {
    node.id_ = node_id;
    nodes.push_back(node);
  }

  return nodes;
}


PhaseNodes::PolyInfoVec
PhaseNodes::BuildPolyInfos (int phase_count,
                            bool is_in_contact_at_start,
                            int n_polys_in_changing_phase,
                            Type type) const
{
  PolyInfoVec polynomial_info;

  bool first_phase_constant = (is_in_contact_at_start  && type==Motion)
                           || (!is_in_contact_at_start && type==Force);


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

bool
PhaseNodes::IsConstantNode (int node_id) const
{
  bool is_constant = false;

  // node is considered constant if either left or right polynomial
  // belongs to a constant phase
  for (int poly_id : GetAdjacentPolyIds(node_id))
    if (polynomial_info_.at(poly_id).is_constant_)
      is_constant = true;

  return is_constant;
}

int
PhaseNodes::GetPolyIDAtStartOfPhase (int phase) const
{
  int poly_id=0;
  for (int i=0; i<polynomial_info_.size(); ++i)
    if (polynomial_info_.at(i).phase_ == phase)
      return i;
}

Vector3d
PhaseNodes::GetValueAtStartOfPhase (int phase) const
{
  int node_id = GetNodeIDAtStartOfPhase(phase);
  return nodes_.at(node_id).at(kPos);
}

int
PhaseNodes::GetNodeIDAtStartOfPhase (int phase) const
{
  int poly_id=GetPolyIDAtStartOfPhase(phase);
  return GetNodeId(poly_id, Side::Start);
}





EEMotionNodes::EEMotionNodes (int phase_count,
                              bool is_in_contact_at_start,
                              const std::string& name,
                              int n_polys)
    :PhaseNodes(kDim3d, phase_count, is_in_contact_at_start, name, n_polys, Motion)
{
}

bool
EEMotionNodes::IsContactNode (int node_id) const
{
  return IsConstantNode(node_id);
}

EEMotionNodes::VecBound
EEMotionNodes::GetBounds () const
{
  for (int idx=0; idx<GetRows(); ++idx) {

    auto node = GetNodeInfoAtOptIndex(idx).front(); // bound idx by first node it represents

    // endeffector is not allowed to move if in stance phase
    if (IsContactNode(node.id_)) {
      if (node.deriv_ == kVel)
        bounds_.at(idx) = BoundZero;
    }
    else { // node in pure swing-phase
      if (node.deriv_ == kVel && node.dim_ == Z)
        bounds_.at(idx) = BoundZero; // zero velocity at top
    }

  }

  return bounds_;
}




EEForceNodes::EEForceNodes (int phase_count,
                            bool is_in_contact_at_start,
                            const std::string& name, int n_polys)
    :PhaseNodes(kDim3d, phase_count, is_in_contact_at_start, name, n_polys, Force)
{
}

bool
EEForceNodes::IsStanceNode (int node_id) const
{
  return !IsConstantNode(node_id);
}

int
EEForceNodes::GetPhase (int node_id) const
{
  assert(IsStanceNode(node_id)); // because otherwise it has two phases

  int poly_id = GetAdjacentPolyIds(node_id).front();
  return polynomial_info_.at(poly_id).phase_;
}

EEForceNodes::VecBound
EEForceNodes::GetBounds () const
{
  for (int idx=0; idx<GetRows(); ++idx) {

    NodeInfo n0 = GetNodeInfoAtOptIndex(idx).front(); // only one node anyway

    if (IsStanceNode(n0.id_)) {

//      if (n0.deriv_ == kPos) {
//
//        if (n0.dim_ == X || n0.dim_ == Y)
//          bounds_.at(idx) = Bound(-f_max_, f_max_);
//
//        // unilateral contact forces ("pulling" on ground not possible)
//        if (n0.dim_ == Z)
//          bounds_.at(idx) = Bound(0.0, f_max_);
//      }
//
//      if (n0.deriv_ == kVel) {
//        bounds_.at(idx) = BoundZero; // zero slope to never exceed zero force between nodes
//      }

    } else { // swing node
      bounds_.at(idx) = BoundZero; // force must be zero
    }

  }

  return bounds_;
}

} /* namespace towr */
