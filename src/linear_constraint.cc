/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/constraints/linear_constraint.h>

#include <xpp/bound.h>
#include <xpp/composite.h>

namespace xpp {
namespace opt {

LinearEqualityConstraint::LinearEqualityConstraint (
    const OptVarsPtr& opt_vars,
    const MatVec& linear_equation,
    const std::string& variable_name)
{
  SetName("LinearEqualityConstraint-" + variable_name);
  linear_equation_ = linear_equation;
  variable_name_   = variable_name;

  int num_constraints = linear_equation_.v.rows();
  SetRows(num_constraints);
  AddOptimizationVariables(opt_vars);
}

LinearEqualityConstraint::~LinearEqualityConstraint ()
{
}

VectorXd
LinearEqualityConstraint::GetValues () const
{
  VectorXd x = GetOptVars()->GetComponent(variable_name_)->GetValues();
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
  // careful, .sparseView is only valid when the Jacobian is constant
  if (var_set == variable_name_)
    jac = linear_equation_.M.sparseView();
}

} /* namespace opt */
} /* namespace xpp */

