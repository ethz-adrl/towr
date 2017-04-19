/**
@file    a_cost.cc
@author  Alexander W. Winkler (winklera@ethz.ch)
@date    Oct 17, 2016
@brief   Defines Cost class
 */

#include <xpp/cost.h>

namespace xpp {
namespace opt {

Cost::Cost (const OptVarsPtr& opt_vars_container)
{
  weight_ = 1.0;
  opt_vars_ = opt_vars_container;
}

Cost::~Cost ()
{
}

Cost::VectorXd
Cost::GetWeightedCost () const
{
  VectorXd cost(1);
  cost(0) = weight_ * GetCost();
  return cost;
}

Cost::Jacobian
Cost::GetWeightedJacobian ()
{
  return weight_ * GetJacobian();
}

void
Cost::SetWeight (double weight)
{
  weight_ = weight;
}

int
Cost::GetVariableCount () const
{
  return opt_vars_->GetOptimizationVariableCount();
}

} // namespace opt
} // namespace xpp

