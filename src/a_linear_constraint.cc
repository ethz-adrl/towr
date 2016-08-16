/**
 @file    a_linear_constraint.cc
 @author  Alexander W. Winkler (winklera@ethz.ch)
 @date    May 26, 2016
 @brief   Brief description
 */

#include <xpp/zmp/a_linear_constraint.h>
#include <xpp/zmp/constraint_container.h>

namespace xpp {
namespace zmp {

ALinearConstraint::ALinearConstraint ()
{
}

void
ALinearConstraint::Init (const MatVec& linear_equation)
{
  linear_equation_ = linear_equation;
}

ALinearConstraint::VectorXd
ALinearConstraint::EvaluateConstraint () const
{
  return linear_equation_.M*x_; // linear part respected in bounds
}

LinearEqualityConstraint::VecBound
LinearEqualityConstraint::GetBounds () const
{
  VecBound bounds;
  for (int i=0; i<linear_equation_.v.rows(); ++i) {
    Bound bound(-linear_equation_.v[i],-linear_equation_.v[i]);
    bounds.push_back(bound);
  }
  return bounds;
}

LinearInequalityConstraint::VecBound
LinearInequalityConstraint::GetBounds () const
{
  VecBound bounds;
  for (int i=0; i<linear_equation_.v.rows(); ++i) {
    Bound bound(-linear_equation_.v[i], kNoBound_.upper_);
    bounds.push_back(bound);
  }
  return bounds;
}

void
LinearSplineEqualityConstraint::UpdateVariables (const ConstraintContainer* container)
{
  x_ = container->GetSplineCoefficients();
}

LinearSplineEqualityConstraint::Jacobian
LinearSplineEqualityConstraint::GetJacobianWithRespectTo (int var_set) const
{
  Jacobian jac; // empy matrix

  if (var_set == OptimizationVariables::VariableSets::kSplineCoff)
    jac = linear_equation_.M;

  return jac;
}

} /* namespace zmp */
} /* namespace xpp */

