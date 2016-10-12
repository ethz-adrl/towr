/**
 @file    cost_container.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include "../include/xpp/opt/cost_container.h"

namespace xpp {
namespace opt {

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

      int n_set = set->GetVariables().rows();
      VectorXd grad_set = cost->EvaluateGradientWrt(set->GetId());

      if (grad_set.rows() != 0) {
        gradient.middleRows(row, n_set) += grad_set;
      }

      row += n_set;
    }
  }

  return gradient;
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
CostContainer::IsEmpty () const
{
  return costs_.empty();
}

} /* namespace zmp */
} /* namespace xpp */

