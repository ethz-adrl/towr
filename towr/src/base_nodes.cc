/*
 * base_nodes.cc
 *
 *  Created on: Jan 12, 2018
 *      Author: winklera
 */

#include <towr/variables/base_nodes.h>

namespace towr {

BaseNodes::BaseNodes (int n_dim, int n_nodes, const std::string& name)
    : NodeVariables(n_dim, name)
{
  int n_variables = n_nodes*2*n_dim;
  InitMembers(n_nodes, n_variables);
}

// reverse of the above
std::vector<BaseNodes::IndexInfo>
BaseNodes::GetNodeInfoAtOptIndex (int idx) const
{
  std::vector<IndexInfo> nodes;

  // always two consecutive node pairs are equal
  int n_opt_values_per_node_ = 2*GetDim();
  int internal_id = idx%n_opt_values_per_node_; // 0...6 (p.x, p.y, p.z, v.x, v.y. v.z)

  IndexInfo node;
  node.node_deriv_     = internal_id<GetDim()? kPos : kVel;
  node.node_deriv_dim_ = internal_id%GetDim();
  node.node_id_        = std::floor(idx/n_opt_values_per_node_);

  nodes.push_back(node);

  return nodes;
}

BaseNodes::VecDurations
BaseNodes::ConvertPhaseToPolyDurations (const VecDurations& phase_durations) const
{
  return phase_durations; // default is do nothing
}

double
BaseNodes::GetDerivativeOfPolyDurationWrtPhaseDuration (int polynomial_id) const
{
  return 1.0; // default every polynomial represents one phase
}

int
BaseNodes::GetNumberOfPrevPolynomialsInPhase(int polynomial_id) const
{
  return 0; // every phase is represented by single polynomial
}

bool
BaseNodes::IsInConstantPhase(int polynomial_id) const
{
  return false;
};



} /* namespace towr */
