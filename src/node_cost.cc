/**
 @file    node_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 7, 2017
 @brief   Brief description
 */

#include <xpp/costs/node_cost.h>

#include <cmath>
#include <Eigen/Dense>
#include <Eigen/Sparse>


namespace xpp {
namespace opt {

NodeCost::NodeCost (const OptVarsPtr& opt_vars, const std::string& nodes_id)
{
  SetName("Node Cost");

  nodes_   = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(nodes_id));
  node_id_ = nodes_id;

  deriv_ = kPos;
  dim_   = Z;

  SetRows(1); // because cost
  AddOptimizationVariables(opt_vars);
}

VectorXd
NodeCost::GetValues () const
{
  VectorXd f = VectorXd::Zero(1);
  for (auto n : nodes_->GetNodes()) {
    double val = n.at(deriv_)(dim_);
    f(0) += std::pow(val,2);
  }

  return f;
}

void
NodeCost::FillJacobianWithRespectTo (std::string var_set,
                                     Jacobian& jac) const
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

} /* namespace opt */
} /* namespace xpp */
