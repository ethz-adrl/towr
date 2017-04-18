/**
 @file    polynomial_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Definition of the ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#include <xpp/opt/polynomial_cost.h>

#include <Eigen/Dense>

#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {


PolynomialCost::PolynomialCost (const OptVarsPtr& opt_vars) :Cost(opt_vars)
{
}

QuadraticPolynomialCost::QuadraticPolynomialCost (const OptVarsPtr& opt_vars,
                                          const MatVec& mat_vec)
    :PolynomialCost(opt_vars)
{
  matrix_vector_ = mat_vec;
  com_motion_    = std::dynamic_pointer_cast<BaseMotion>(opt_vars->GetSet("base_motion"));
}

QuadraticPolynomialCost::~QuadraticPolynomialCost ()
{
}

double
QuadraticPolynomialCost::EvaluateCost () const
{
  double cost = 0.0;
  VectorXd spline_coeff_ = com_motion_->GetXYSplineCoeffients();

  cost += spline_coeff_.transpose() * matrix_vector_.M * spline_coeff_;
  cost += matrix_vector_.v.transpose() * spline_coeff_;

  return cost;
}


void
QuadraticPolynomialCost::FillGradientWrt(std::string var_set, VectorXd& grad)
{
  if (var_set == com_motion_->GetId())
    grad =  2.0 * matrix_vector_.M * com_motion_->GetXYSplineCoeffients();
}

VectorXd
QuadraticPolynomialCost::EvaluateGradient ()
{
  VectorXd grad = VectorXd::Zero(GetVariableCount());

  int row = 0;
  for (const auto& vars : opt_vars_->GetOptVarsVec()) {

    int n_set = vars->GetOptVarCount();
    VectorXd grad_set = VectorXd::Zero(n_set); // default is not dependent
    FillGradientWrt(vars->GetId(), grad_set);
    grad.middleRows(row, n_set) = grad_set;
    row += n_set;
  }

  return grad;
}

//double
//SquaredSplineCost::EvaluateCost () const
//{
//  return (matrix_vector_.M*com_motion_->GetXYSplineCoeffients()
//          + matrix_vector_.v).norm();
//}

} /* namespace opt */
} /* namespace xpp */

