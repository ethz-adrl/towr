/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/linear_constraint.h>

#include <Eigen/Dense>
#include <Eigen/Sparse>
#include <vector>

#include <xpp/opt/polynomial_spline.h>
#include <xpp/opt/variables/variable_names.h>

namespace xpp {
namespace opt {

LinearEqualityConstraint::LinearEqualityConstraint (
    const OptVarsPtr& opt_vars,
    const MatVec& linear_equation)
{
  linear_equation_ = linear_equation;

  // zmp_ make this more general
  com_motion_ = std::dynamic_pointer_cast<PolynomialSpline>(opt_vars->GetComponent(id::base_linear));
  opt_vars_ = opt_vars;

  int num_constraints = linear_equation_.v.rows();
  SetName("LinearEqualityConstraint");
  SetRows(num_constraints);
  AddComposite(opt_vars);
}

LinearEqualityConstraint::~LinearEqualityConstraint ()
{
}

VectorXd
LinearEqualityConstraint::GetValues () const
{
  VectorXd x = com_motion_->GetValues();
  return linear_equation_.M*x;
}

VecBound
LinearEqualityConstraint::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<GetRows(); ++i) {
    Bound bound(-linear_equation_.v[i],-linear_equation_.v[i]);
    bounds.push_back(bound);
  }

  return bounds;
}

void
LinearEqualityConstraint::FillJacobianWithRespectTo (std::string var_set, Jacobian& jac) const
{
  // the constraints are all linear w.r.t. the decision variables.
  // careful, .sparseView is only valid when the Jacobian is constant, e.g.

  if (var_set == com_motion_->GetName()) {

//    std::cout << "jac.cols(): " << jac.cols() << std::endl;
//    std::cout << "linear_equation.cols(): " << linear_equation_.M.cols() << std::endl;
    jac = linear_equation_.M.sparseView();
//    std::cout << "jac.cols()_after: " << jac.cols() << std::endl;
  }
}

} /* namespace opt */
} /* namespace xpp */

