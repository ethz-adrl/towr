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
  for (auto cost : costs_)
    total_cost += cost->EvaluateCost();

  return total_cost;
}

} /* namespace zmp */
} /* namespace xpp */
