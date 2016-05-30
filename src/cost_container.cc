/**
 @file    cost_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/cost_container.h>

namespace xpp {
namespace zmp {

CostContainer::CostContainer (OptimizationVariables& subject)
    :EigenNumDiffFunctor(subject.GetOptimizationVariableCount(), 1)
{
  subject_ = &subject;
}

void
CostContainer::AddCost (const ACost& cost)
{
  costs_.push_back(&cost);
}

double
CostContainer::EvaluateTotalCost () const
{
 double total_cost = 0.0;
  for (const ACost* cost : costs_)
    total_cost += cost->EvaluateCost();

  return total_cost;
}

int
CostContainer::operator() (const InputType& x, ValueType& obj_value) const
{
  subject_->SetVariables(x);
  obj_value(0) = EvaluateTotalCost();
  return 1;
}

} /* namespace zmp */
} /* namespace xpp */
