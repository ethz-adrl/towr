/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <opt_solve/linear_constraint.h>


namespace opt {

LinearEqualityConstraint::LinearEqualityConstraint (
  const Composite::Ptr& variables,
  const Eigen::MatrixXd& M,
  const Eigen::VectorXd& v,
  const std::string& variable_name)
    : Constraint(variables, v.rows(), "LinearEqualityConstraint-" + variable_name)
{
  M_ = M;
  v_ = v;
  variable_name_   = variable_name;
}

LinearEqualityConstraint::~LinearEqualityConstraint ()
{
}

LinearEqualityConstraint::VectorXd
LinearEqualityConstraint::GetValues () const
{
  VectorXd x = GetVariables()->GetComponent(variable_name_)->GetValues();
  return M_*x;
}

LinearEqualityConstraint::VecBound
LinearEqualityConstraint::GetBounds () const
{
  VecBound bounds;

  for (int i=0; i<GetRows(); ++i) {
    NLPBound bound(-v_[i],-v_[i]);
    bounds.push_back(bound);
  }

  return bounds;
}

void
LinearEqualityConstraint::FillJacobianBlock (std::string var_set, Jacobian& jac) const
{
  // the constraints are all linear w.r.t. the decision variables.
  // careful, sparseView is only valid when the Jacobian is constant
  if (var_set == variable_name_)
    jac = M_.sparseView();
}

} /* namespace opt */

