/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/opt/a_linear_constraint.h>

namespace xpp {
namespace opt {

ALinearConstraint::ALinearConstraint ()
{
}

ALinearConstraint::~ALinearConstraint ()
{
}

void
ALinearConstraint::Init (const ComMotionPtr& com_motion,
                         const MatVec& linear_equation, const std::string& name)
{
  linear_equation_ = linear_equation;
  name_ = name;

  com_motion_ = com_motion;

  int num_constraints = linear_equation_.v.rows();
  SetDependentVariables({com_motion}, num_constraints);

  Jacobian& jac = GetJacobianRefWithRespectTo(com_motion->GetID());
  // careful, .sparseView is only valid when the Jacobian is constant, e.g.
  // the constraints are all linear w.r.t. the decision variables.
  jac = linear_equation_.M.sparseView();
}

ALinearConstraint::VectorXd
ALinearConstraint::EvaluateConstraint () const
{
  VectorXd x = com_motion_->GetXYSplineCoeffients();
  return linear_equation_.M*x; // linear part respected in bounds
}

VecBound
LinearEqualityConstraint::GetBounds () const
{
  for (int i=0; i<num_constraints_; ++i) {
    Bound bound(-linear_equation_.v[i],-linear_equation_.v[i]);
    bounds_.at(i) = bound;
  }
  return bounds_;
}

VecBound
LinearInequalityConstraint::GetBounds () const
{
  for (int i=0; i<num_constraints_; ++i) {
    Bound bound(-linear_equation_.v[i], kNoBound_.upper_);
    bounds_.at(i) = bound;
  }
  return bounds_;
}

} /* namespace opt */
} /* namespace xpp */

