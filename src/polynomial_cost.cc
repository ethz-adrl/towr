/**
 @file    polynomial_cost.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 30, 2016
 @brief   Definition of the ASplineCost, QuadraticSplineCost, SquaredSplineCost
 */

#include <xpp/opt/costs/polynomial_cost.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/opt/polynomial_spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {


QuadraticPolynomialCost::QuadraticPolynomialCost (const OptVarsPtr& opt_vars,
                                                  const MatVec& mat_vec,
                                                  double weight)
{
  matrix_vector_ = mat_vec;
  weight_        = weight;
  polynomial_    = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_linear));

  SetName("Polynomial Cost");
  SetRows(1); // because cost
  AddComposite(opt_vars);
}

QuadraticPolynomialCost::~QuadraticPolynomialCost ()
{
}

VectorXd
QuadraticPolynomialCost::GetValues () const
{
  VectorXd cost = VectorXd::Zero(GetRows());
  VectorXd spline_coeff_ = polynomial_->GetValues();

  cost += spline_coeff_.transpose() * matrix_vector_.M * spline_coeff_;
  cost += matrix_vector_.v.transpose() * spline_coeff_;

  return weight_*cost;
}

void
QuadraticPolynomialCost::FillJacobianWithRespectTo(std::string var_set, Jacobian& jac) const
{
  if (var_set == polynomial_->GetName()) {
    VectorXd grad = 2.0 * matrix_vector_.M * polynomial_->GetValues();
    jac.row(0) =  grad.transpose().sparseView();
  }
}

} /* namespace opt */
} /* namespace xpp */
