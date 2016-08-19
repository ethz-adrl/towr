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
    :IObserver(subject)
{
}

CostContainer::~CostContainer ()
{
  // TODO Auto-generated destructor stub
}

void
CostContainer::ClearCosts ()
{
  costs_.clear();
}

void
CostContainer::AddCost (CostPtr cost)
{
  costs_.push_back(cost);
}

void
CostContainer::Update ()
{
//  spline_coeff_      = subject_->GetVariables(OptimizationVariables::kSplineCoeff);
//  VectorXd footholds = subject_->GetVariables(OptimizationVariables::kFootholds);
//
//  footholds_    = utils::ConvertEigToStd(footholds);
}

double
CostContainer::EvaluateTotalCost () const
{
  double total_cost = 0.0;
  for (const auto& cost : costs_) {
    cost->UpdateVariables(subject_);
    total_cost += cost->EvaluateCost();
  }

  return total_cost;
}

bool
xpp::zmp::CostContainer::IsEmpty () const
{
  return costs_.empty();
}

} /* namespace zmp */
} /* namespace xpp */

