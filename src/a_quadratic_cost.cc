/**
 @file    a_linear_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include "../include/xpp/zmp/a_quadratic_cost.h"

namespace xpp {
namespace zmp {

AQuadraticCost::AQuadraticCost (OptimizationVariables& subject)
{
  subject_ = &subject;
  subject_->RegisterObserver(this);
}

void
AQuadraticCost::Init (const MatVec& linear_equation)
{
  linear_equation_ = linear_equation;
  Update();
}

void
AQuadraticCost::Update ()
{
  x_coeff_ = subject_->GetSplineCoefficients();
}

double
AQuadraticCost::EvaluateCost () const
{
  double cost = 0.0;

  cost += x_coeff_.transpose() * linear_equation_.M * x_coeff_;
  cost += linear_equation_.v.transpose() * x_coeff_;

  return cost;
}

} /* namespace zmp */
} /* namespace xpp */
