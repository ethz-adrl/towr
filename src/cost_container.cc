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
CostContainer::AddCost (CostPtr cost, const std::string& name)
{
  costs_.emplace(name, cost);
}

void
CostContainer::Update ()
{
  spline_coeff_      = subject_->GetVariables(OptimizationVariables::kSplineCoeff);
  VectorXd footholds = subject_->GetVariables(OptimizationVariables::kFootholds);

  footholds_    = utils::ConvertEigToStd(footholds);
}

double
CostContainer::EvaluateTotalCost () const
{
  double total_cost = 0.0;
  for (const auto& cost : costs_) {
    cost.second->UpdateVariables(this);
    total_cost += cost.second->EvaluateCost();
  }

  return total_cost;
}

ACost&
CostContainer::GetCost (const std::string& name)
{
  return *costs_.at(name);
}

const CostContainer::FootholdsXY&
CostContainer::GetFootholds () const
{
  return footholds_;
}

const CostContainer::VectorXd&
CostContainer::GetSplineCoefficients () const
{
  return spline_coeff_;
}

bool
xpp::zmp::CostContainer::IsEmpty () const
{
  return costs_.empty();
}

} /* namespace zmp */
} /* namespace xpp */

