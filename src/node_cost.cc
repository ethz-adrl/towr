/**
 @file    node_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Aug 7, 2017
 @brief   Brief description
 */

#include <xpp/opt/costs/node_cost.h>

namespace xpp {
namespace opt {

NodeCost::NodeCost (const OptVarsPtr& opt_vars, const std::string& nodes_id)
{
  SetName("Node Cost");

  nodes_   = std::dynamic_pointer_cast<NodeValues>(opt_vars->GetComponent(nodes_id));
  node_id_ = nodes_id;

  SetRows(1); // because cost
  AddOptimizationVariables(opt_vars);
}

NodeCost::~NodeCost ()
{
  // TODO Auto-generated destructor stub
}

VectorXd
NodeCost::GetValues () const
{
  VectorXd x = nodes_->GetValues();
  VectorXd cost = x.transpose()*x;
  return cost;
}

void
NodeCost::FillJacobianWithRespectTo (std::string var_set,
                                     Jacobian& jac) const
{
  if (var_set == node_id_) {
    VectorXd x = nodes_->GetValues();
    VectorXd grad = 2.0 * x;
    jac.row(0) =  grad.transpose().sparseView(1.0, -1.0);
  }
}

} /* namespace opt */
} /* namespace xpp */
