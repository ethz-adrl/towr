/**
 @file    node_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 7, 2017
 @brief   Brief description
 */

#include <towr/costs/node_cost.h>

#include <cmath>
#include <Eigen/Eigen>

#include <towr/variables/cartesian_declarations.h>


namespace towr {

NodeCost::NodeCost (const std::string& nodes_id) : CostTerm("Node Cost")
{
  node_id_ = nodes_id;

  deriv_ = kPos;
  dim_   = Z;
}

void
NodeCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  nodes_ = std::dynamic_pointer_cast<NodeVariables>(x->GetComponent(node_id_));
}

double
NodeCost::GetCost () const
{
  double cost;
  for (auto n : nodes_->GetNodes()) {
    double val = n.at(deriv_)(dim_);
    cost += std::pow(val,2);
  }

  return cost;
}

void
NodeCost::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  if (var_set == node_id_) {

    for (int idx=0; idx<nodes_->GetRows(); ++idx)
      for (auto n : nodes_->GetNodeInfoAtOptIndex(idx))
        if (n.node_deriv_==deriv_ && n.node_deriv_dim_==dim_) {
          double val = nodes_->GetNodes().at(n.node_id_).at(deriv_)(dim_);
          jac.coeffRef(0, idx) += 2.0*val;
        }
  }
}

} /* namespace towr */


