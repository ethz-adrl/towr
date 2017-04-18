/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/opt/constraints/linear_constraint.h>

#include <vector>
#include <Eigen/Dense>
#include <Eigen/Sparse>

#include <xpp/bound.h>
#include <xpp/opt/variables/base_motion.h>

namespace xpp {
namespace opt {

LinearEqualityConstraint::LinearEqualityConstraint (
    const OptVarsPtr& opt_vars,
    const MatVec& linear_equation,
    const std::string& name)
{
  linear_equation_ = linear_equation;
  name_ = name;

  com_motion_ = std::dynamic_pointer_cast<BaseMotion>(opt_vars->GetSet("base_motion"));

  int num_constraints = linear_equation_.v.rows();
  SetDimensions(opt_vars->GetOptVarsVec(), num_constraints);

  Jacobian& jac = GetJacobianRefWithRespectTo(com_motion_->GetId());
  // careful, .sparseView is only valid when the Jacobian is constant, e.g.
  // the constraints are all linear w.r.t. the decision variables.
  jac = linear_equation_.M.sparseView();
}

LinearEqualityConstraint::~LinearEqualityConstraint ()
{
}

LinearEqualityConstraint::VectorXd
LinearEqualityConstraint::GetConstraintValues () const
{
  VectorXd x = com_motion_->GetXYSplineCoeffients();
  return linear_equation_.M*x;
}

void
LinearEqualityConstraint::UpdateBounds ()
{
  for (int i=0; i<GetNumberOfConstraints(); ++i) {
    Bound bound(-linear_equation_.v[i],-linear_equation_.v[i]);
    bounds_.at(i) = bound;
  }
}

} /* namespace opt */
} /* namespace xpp */

