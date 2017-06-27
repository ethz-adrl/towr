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
                                                  const std::string& variables,
                                                  double weight)
{
  matrix_vector_ = mat_vec;
  variables_     = variables;
  weight_        = weight;
//  polynomial_    = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_linear));

  SetName("Polynomial Cost");
  SetRows(1); // because cost
  AddOptimizationVariables(opt_vars);
}

QuadraticPolynomialCost::~QuadraticPolynomialCost ()
{
}

VectorXd
QuadraticPolynomialCost::GetValues () const
{
  VectorXd cost = VectorXd::Zero(GetRows());
  VectorXd x    = GetOptVars()->GetComponent(variables_)->GetValues();     //polynomial_->GetValues();

  cost += x.transpose() * matrix_vector_.M * x;
  cost += matrix_vector_.v.transpose() * x;

  return weight_*cost;
}

void
QuadraticPolynomialCost::FillJacobianWithRespectTo(std::string var_set, Jacobian& jac) const
{
  if (var_set == variables_) {

    VectorXd x = GetOptVars()->GetComponent(variables_)->GetValues();     //polynomial_->GetValues();

    VectorXd grad = 2.0 * matrix_vector_.M * x;
    jac.row(0) =  grad.transpose().sparseView();
  }
}

} /* namespace opt */
} /* namespace xpp */
