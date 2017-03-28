/**
 @file    linear_spline_equality_constraints.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    Mar 13, 2017
 @brief   Brief description
 */

#include <xpp/opt/linear_spline_equality_constraint.h>
#include <xpp/opt/com_motion.h>
#include <xpp/optimization_variables.h>

namespace xpp {
namespace opt {

LinearSplineEqualityConstraint::LinearSplineEqualityConstraint (
    const ComMotion& com_motion)
{
  com_motion_ = com_motion.clone();
}

LinearSplineEqualityConstraint::~LinearSplineEqualityConstraint ()
{
  // TODO Auto-generated destructor stub
}

void
LinearSplineEqualityConstraint::UpdateVariables (const OptimizationVariables* opt_var)
{
  VectorXd x = opt_var->GetVariables(com_motion_->GetID());
  com_motion_->SetOptimizationParameters(x);
  x_ = com_motion_->GetCoeffients();
}

LinearSplineEqualityConstraint::Jacobian
LinearSplineEqualityConstraint::GetJacobianWithRespectTo (std::string var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == com_motion_->GetID()) {
    // careful, .sparseView is only valid when the Jacobian is constant, e.g.
    // the constraints are all linear w.r.t. the decision variables.
    jac = linear_equation_.M.sparseView();
  }

  return jac;
}

} /* namespace opt */
} /* namespace xpp */
