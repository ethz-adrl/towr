/**
 @file    a_linear_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Brief description
 */

#include <xpp/zmp/a_quadratic_cost.h>

namespace xpp {
namespace zmp {

AQuadraticCost::AQuadraticCost (OptimizationVariables& subject)
    :IObserver(subject)
{
}

void
AQuadraticCost::Init (const MatVec& quadratic_equation)
{
  quadratic_equation_ = quadratic_equation;
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

  cost += x_coeff_.transpose() * quadratic_equation_.M * x_coeff_;
  cost += quadratic_equation_.v.transpose() * x_coeff_;

  return cost;
}

} /* namespace zmp */
} /* namespace xpp */
