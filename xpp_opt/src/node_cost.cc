/**
 @file    node_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 7, 2017
 @brief   Brief description
 */

#include <xpp_opt/costs/node_cost.h>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace xpp {

NodeCost::NodeCost (const std::string& nodes_id) : CostTerm("Node Cost")
{
//  SetName("Node Cost");

//  nodes_   = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(nodes_id));
  node_id_ = nodes_id;

  deriv_ = kPos;
  dim_   = Z;

//  SetRows(1); // because cost
//  AddOptimizationVariables(opt_vars);
}

void
NodeCost::InitVariableDependedQuantities (const VariablesPtr& x)
{
  nodes_ = std::dynamic_pointer_cast<NodeValues>(x->GetComponent(node_id_));
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
      for (auto n : nodes_->GetNodeInfo(idx))
        if (n.deriv_==deriv_ && n.dim_==dim_) {
          double val = nodes_->GetNodes().at(n.id_).at(deriv_)(dim_);
          jac.coeffRef(0, idx) += 2.0*val;
        }
  }
}

NodeCost::~NodeCost (){}

} /* namespace xpp */


