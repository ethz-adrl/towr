/**
 @file    phase_nodes1.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 4, 2017
 @brief   Brief description
 */

#include <towr/variables/phase_nodes.h>

#include <cassert>

#include <towr/variables/cartesian_declarations.h>


namespace towr {


PhaseNodes::PhaseNodes (int phase_count,
                        bool is_in_contact_at_start,
                        const std::string& name,
                        int n_polys_in_changing_phase,
                        Type type)
    :NodeVariables(3, name)
{
  polynomial_info_ = BuildPolyInfos(phase_count, is_in_contact_at_start, n_polys_in_changing_phase, type);
  optnode_to_node_ = SetNodeMappings(polynomial_info_);

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
    poly_durations.push_back(phase_durations.at(info.phase_)/info.num_polys_in_phase_);
  }

  return poly_durations;
}

double
PhaseNodes::GetDerivativeOfPolyDurationWrtPhaseDuration (int poly_id) const
{
  int n_polys_in_phase = polynomial_info_.at(poly_id).num_polys_in_phase_;
  return 1./n_polys_in_phase;
}

int
PhaseNodes::GetNumberOfPrevPolynomialsInPhase(int poly_id) const
{
  return polynomial_info_.at(poly_id).poly_id_in_phase_;
}

std::map<PhaseNodes::OptNodeIs, PhaseNodes::NodeIds>
PhaseNodes::SetNodeMappings (const std::vector<PolyInfo>& polynomial_info)
{
  std::map<OptNodeIs, NodeIds> optnode_to_node;

  int opt_id = 0;
  for (int i=0; i<polynomial_info.size(); ++i) {
    int node_id_start = GetNodeId(i, CubicHermitePoly::Start);

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
  node.node_deriv_ = internal_id<GetDim()? kPos : kVel;
  node.node_deriv_dim_   = internal_id%GetDim();

  // this is different compared to standard node values
  // because one index can represent multiple node (during constant phase)
  int opt_node = std::floor(idx/n_opt_values_per_node_);
  for (auto node_id : optnode_to_node_.at(opt_node)) {
    node.node_id_ = node_id;
    nodes.push_back(node);
  }

  return nodes;
}

std::vector<PhaseNodes::PolyInfo>
PhaseNodes::BuildPolyInfos (int phase_count,
                            bool is_in_contact_at_start,
                            int n_polys_in_changing_phase,
                            Type type)
{
  std::vector<PolyInfo> polynomial_info;

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
  return GetNodes().at(node_id).val_;
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
  for (int idx=0; idx<GetRows(); ++idx) {

    auto node = GetNodeInfoAtOptIndex(idx).front(); // bound idx by first node it represents

    // endeffector is not allowed to move if in stance phase
    if (IsConstantNode(node.node_id_)) {
      if (node.node_deriv_ == kVel)
        bounds_.at(idx) = ifopt::BoundZero;
    }
    else { // node in pure swing-phase
      if (node.node_deriv_ == kVel && node.node_deriv_dim_ == Z)
        bounds_.at(idx) = ifopt::BoundZero; // zero velocity at top
    }

  }
}

void
PhaseNodes::SetBoundsEEForce ()
{
  for (int idx=0; idx<GetRows(); ++idx) {

    IndexInfo n0 = GetNodeInfoAtOptIndex(idx).front(); // only one node anyway

    if (!IsConstantNode(n0.node_id_)) {

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
      bounds_.at(idx) = ifopt::BoundZero; // force must be zero
    }

  }
}

PhaseNodes::PolyInfo::PolyInfo(int phase, int poly_id_in_phase,
                               int num_polys_in_phase, bool is_constant)
    :phase_(phase),
     poly_id_in_phase_(poly_id_in_phase),
     num_polys_in_phase_(num_polys_in_phase),
     is_constant_(is_constant)
{
}

} /* namespace towr */
