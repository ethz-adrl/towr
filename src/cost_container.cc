/**
 @file    cost_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/cost_container.h>

namespace xpp {
namespace opt {

CostContainer::CostContainer ()
{
}

CostContainer::~CostContainer ()
{
}

void
CostContainer::ClearCosts ()
{
  costs_.clear();
}

void
CostContainer::AddCost (CostPtr cost, double weight)
{
  cost->SetWeight(weight);
  costs_.push_back(cost);
  UpdateCosts();
}

void
CostContainer::SetWeights (const std::vector<double>& weights)
{
  assert(weights.size() == costs_.size());

  int i=0;
  for (auto& cost : costs_)
    cost->SetWeight(weights.at(i++));
}

double
CostContainer::EvaluateTotalCost () const
{
  double total_cost = 0.0;
  for (const auto& cost : costs_)
    total_cost += cost->EvaluateWeightedCost();

  return total_cost;
}

CostContainer::VectorXd
CostContainer::EvaluateGradient () const
{
  int n = costs_.front()->GetVariableCount();
  VectorXd gradient = VectorXd::Zero(n);

  for (const auto& cost : costs_)
    gradient += cost->EvaluateCompleteGradient();

  return gradient;
}


bool
CostContainer::IsEmpty () const
{
  return costs_.empty();
}

void
CostContainer::UpdateCosts ()
{
  for (auto& cost : costs_)
    cost->Update();
}

} /* namespace opt */
} /* namespace xpp */
