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
QuadraticPolynomialCost::GetCost () const
{
  double cost = 0.0;
  VectorXd spline_coeff_ = com_motion_->GetXYSplineCoeffients();

  cost += spline_coeff_.transpose() * matrix_vector_.M * spline_coeff_;
  cost += matrix_vector_.v.transpose() * spline_coeff_;

  return cost;
}

QuadraticPolynomialCost::Jacobian
QuadraticPolynomialCost::GetJacobian () const
{
  Jacobian grad(1,GetVariableCount());

  int col = 0;
  for (const auto& vars : opt_vars_->GetOptVarsVec()) {

    int n_set = vars->GetOptVarCount();
    Jacobian grad_set = Jacobian(1,n_set);    // default is not dependent
    FillGradientWrt(vars->GetId(), grad_set);

    // insert the derivative in the correct position in the overall Jacobian
    for (int k=0; k<grad_set.outerSize(); ++k)
      for (Jacobian::InnerIterator it(grad_set,k); it; ++it)
        grad.coeffRef(it.row(), col+it.col()) = it.value();

    col += n_set;
  }

  return grad;
}

void
QuadraticPolynomialCost::FillGradientWrt(std::string var_set, Jacobian& jac) const
{
  if (var_set == com_motion_->GetId()) {
    VectorXd grad = 2.0 * matrix_vector_.M * com_motion_->GetXYSplineCoeffients();
    jac.row(0) =  grad.transpose().sparseView();
  }
}
//double
//SquaredSplineCost::EvaluateCost () const
//{
//  return (matrix_vector_.M*com_motion_->GetXYSplineCoeffients()
//          + matrix_vector_.v).norm();
//}

} /* namespace opt */
} /* namespace xpp */

