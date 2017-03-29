/**
 @file    cost_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/cost_container.h>

namespace xpp {
namespace opt {

CostContainer::CostContainer (OptimizationVariables& subject)
    :Observer(subject)
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
CostContainer::AddCost (CostPtr cost, double weight)
{
  cost->SetWeight(weight);
  costs_.push_back(cost);
}

void
CostContainer::Update ()
{
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
  for (const auto& cost : costs_) {
    cost->UpdateVariables(subject_);
    total_cost += cost->EvaluateWeightedCost();
  }

  return total_cost;
}

CostContainer::VectorXd
CostContainer::EvaluateGradient () const
{
  int n = subject_->GetOptimizationVariableCount();
  VectorXd gradient = VectorXd::Zero(n);
  for (const auto& cost : costs_) {
    cost->UpdateVariables(subject_);

    int row = 0;
    for (const auto& set : subject_->GetVarSets()) {

      int n_set = set.GetVariables().rows();
      VectorXd grad_set = cost->EvaluateWeightedGradientWrt(set.GetId());

      if (grad_set.rows() != 0) {
        gradient.middleRows(row, n_set) += grad_set;
      }

      row += n_set;
    }
  }

  return gradient;
}


bool
CostContainer::IsEmpty () const
{
  return costs_.empty();
}

} /* namespace opt */
} /* namespace xpp */
