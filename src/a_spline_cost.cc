/**
 @file    a_spline_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Definition of the ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#include <xpp/opt/a_spline_cost.h>

namespace xpp {
namespace opt {


ASplineCost::ASplineCost ()
{
}

void
ASplineCost::Init (const MatVec& mat_vec, const ComMotion& com_motion)
{
  matrix_vector_ = mat_vec;
  com_motion_ = com_motion.clone();
}

void
ASplineCost::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x = opt_var->GetVariables(com_motion_->GetID());
  com_motion_->SetOptimizationParameters(x);
  spline_coeff_ = com_motion_->GetCoeffients();
}

// zmp_ same as soft constraint?
double
QuadraticSplineCost::EvaluateCost () const
{
  double cost = 0.0;

  cost += spline_coeff_.transpose() * matrix_vector_.M * spline_coeff_;
  cost += matrix_vector_.v.transpose() * spline_coeff_;

  return cost;
}

QuadraticSplineCost::VectorXd
QuadraticSplineCost::EvaluateGradientWrt(std::string var_set)
{
  VectorXd grad;

  if (var_set == com_motion_->GetID())
    grad =  2.0 * matrix_vector_.M * spline_coeff_;

  return grad;
}

double
SquaredSplineCost::EvaluateCost () const
{
  return (matrix_vector_.M*spline_coeff_ + matrix_vector_.v).norm();
}

} /* namespace opt */
} /* namespace xpp */
