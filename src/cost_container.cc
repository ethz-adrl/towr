/**
 @file    cost_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/cost_container.h>
#include <cassert>

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
}

void
CostContainer::SetWeights (const std::vector<double>& weights)
{
  assert(weights.size() == costs_.size());

  int i=0;
  for (auto& cost : costs_)
    cost->SetWeight(weights.at(i++));
}

CostContainer::VectorXd
CostContainer::GetWeightedCost () const
{
  VectorXd total_cost(1);
  total_cost.setZero();
  for (const auto& cost : costs_)
    total_cost += cost->GetWeightedCost();

  return total_cost;
}

CostContainer::Jacobian
CostContainer::GetWeightedJacobian () const
{
  int n = costs_.front()->GetVariableCount();
  Jacobian jac(1,n);

  for (const auto& cost : costs_)
    jac += cost->GetWeightedJacobian();

  return jac;
}

bool
CostContainer::IsEmpty () const
{
  return costs_.empty();
}

} /* namespace opt */
} /* namespace xpp */
