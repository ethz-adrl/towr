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

  n_variables_ = opt_vars_container->GetOptimizationVariableCount();

  all_variable_ids_.clear();
  for (const auto& var : opt_vars_container->GetOptVarsVec())
    all_variable_ids_.push_back(var->GetId());
}

Cost::~Cost ()
{
}

double
Cost::EvaluateWeightedCost () const
{
  return weight_ * EvaluateCost();
}

Cost::VectorXd
Cost::EvaluateCompleteGradient ()
{
  VectorXd grad = VectorXd::Zero(GetVariableCount());

  int row = 0;
  for (const auto& id : all_variable_ids_) {

    VectorXd grad_set = EvaluateGradientWrt(id);
    int n_set = grad_set.rows();
    grad.middleRows(row, n_set) = grad_set;
    row += n_set;
  }

  return weight_ * grad;
}

void
Cost::SetWeight (double weight)
{
  weight_ = weight;
}

int
Cost::GetVariableCount () const
{
  return n_variables_;
}

} // namespace opt
} // namespace xpp

